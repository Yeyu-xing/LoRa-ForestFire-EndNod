/**
  ******************************************************************************
  * @file    app_main.c
  * @brief   应用主框架 - 状态机与主循环实现
  ******************************************************************************
  */

#include "app_main.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "board_io.h"
#include "driver_button.h"
#include "driver_sht40.h"
#include "driver_gnss.h"
#include "driver_co.h"
#include "driver_eeprom.h"
#include "app_config.h"
#include "app_menu.h"
#include "app_lora.h"
#include "lora_protocol.h"
#include "radio.h"
#include "tim.h"
#include "i2c.h"
#include "oled.h"
#include "usart.h"
#include "timer_if.h"
#include "stm32wlxx_hal_pwr.h"

extern RTC_HandleTypeDef hrtc;

/* RTC闹钟唤醒标志 (ISR中设置, 主循环检查) */
volatile bool g_rtc_wake = false;

/*============================================================================
 *                              配置宏
 *===========================================================================*/
/* 注册阶段内部枚举 (仅 app_main 内部使用) */
enum { REG_PHASE_TX = 0, REG_PHASE_RX15, REG_PHASE_EXTEND };

/* SHT40重试参数 */
#define SHT40_INIT_RETRY_COUNT      3       // SHT40初始化重试次数
#define SHT40_INIT_RETRY_DELAY_MS   50      // 重试间隔(ms)
#define SHT40_MEASURE_RETRY_COUNT   2       // 测量失败重试次数

/* 首次测量延时 (上电后等待传感器稳定) */
#define FIRST_MEASURE_DELAY_MS      5000    // 5s

/* 进入设置的按键长按时间 (由driver_button处理) */

/*============================================================================
 *                              私有变量
 *===========================================================================*/
static app_state_t current_state = STATE_INIT;
static run_mode_t current_mode = RUN_MODE_NORMAL;
static sensor_data_t sensor_data = {0};
static uint16_t error_flags = 0;

/* 定时管理 */
static uint32_t last_measure_tick = 0;      // 上次采集时间
static uint32_t last_transmit_tick = 0;     // 上次发送时间

/* RECEIVE状态相关 */
static uint32_t rx_start_tick = 0;
static bool rx1_done = false;

/* 发送相关 */
static bool registration_done = false;
static uint32_t tx_start_tick = 0;
static uint8_t tx_retry_count = 0;          // 伪超时重试计数
#define TX_MAX_RETRY        3               // 最大重试次数

/* 协议帧缓冲区 */
#define PROTO_TX_BUF_SIZE   128
static uint8_t proto_tx_buf[PROTO_TX_BUF_SIZE];

/* 协议状态相关 */
static uint32_t scan_start_tick = 0;        // 扫描开始时间
static uint32_t reg_start_tick = 0;         // 注册发送时间
static uint32_t reg_phase_start_tick = 0;   // 注册当前阶段开始时间
static uint8_t  reg_phase = 0;              // 0=TX, 1=RX_15s, 2=RX_EXTENDED
static bool     reg_rx_started = false;     // RX窗口是否已开启
static bool     reg_got_proxy_ack = false;  // 已收到中继代理ACK
static uint16_t last_tx_seq = 0;            // 上次发送序列号
static bool     tx_awaiting_ack = false;    // 发送后等待ACK, 用于缓存决策

/* 上行附加信息 */
static bool cfg_report_pending = false;     // 待上报全配置

/* 指令执行状态队列 (每项2B: type+status) */
#define CMD_STATUS_MAX  8
static uint8_t cmd_status_queue[CMD_STATUS_MAX * 2];
static uint8_t cmd_status_count = 0;

/* GNSS相关 */
static bool gnss_location_saved = false;    // 位置是否已保存到EEPROM

/* TX超时 */
#define TX_TIMEOUT_MS   10000       // 发送超时10s

/* 休眠配置 */
#define SLEEP_STOP2_THRESHOLD_S  5   // 超过此秒数使用STOP2, 否则WFI
#define CO_WARMUP_ADVANCE_S      30  // CO传感器预热提前时间(s)

/* 休眠相关 */
static uint32_t sleep_wakeup_tick = 0;      // 预期唤醒的tick值
static bool     sleep_co_warmup = false;     // 本次唤醒是为了给CO预热
static uint32_t sleep_co_deadline_tick = 0;  // CO预热完成后的采集时间
static bool     sleep_from_stop2 = false;    // 刚从STOP2醒来 (HAL_GetTick未更新)

/*============================================================================
 *                              私有函数 - RTC闹钟
 *===========================================================================*/
/**
  * @brief  设置RTC闹钟在指定秒数后触发
  * @param  seconds: 延迟秒数
  */
static void rtc_set_alarm(uint32_t seconds)
{
    /* 获取当前RTC tick值(自增)并转换为down-counter匹配值 */
    uint32_t current_ticks = TIMER_IF_GetTimerValue();
    uint32_t delay_ticks   = TIMER_IF_Convert_ms2Tick(seconds * 1000);
    uint32_t alarm_ssr     = UINT32_MAX - (current_ticks + delay_ticks);

    RTC_AlarmTypeDef sAlarm = {0};
    sAlarm.Alarm = RTC_ALARM_A;
    sAlarm.AlarmTime.SubSeconds     = alarm_ssr;
    sAlarm.AlarmMask                = RTC_ALARMMASK_NONE;
    sAlarm.AlarmSubSecondMask        = RTC_ALARMSUBSECONDBINMASK_NONE;
    sAlarm.BinaryAutoClr            = RTC_ALARMSUBSECONDBIN_AUTOCLR_YES;
    HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD);

    g_rtc_wake = false;
}

static void rtc_stop_alarm(void)
{
    HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
}

/*============================================================================
 *                              私有函数 - STOP2模式
 *===========================================================================*/

/**
  * @brief  进入STOP2低功耗模式
  * @note   关闭非必要外设, 设置RTC闹钟后进入
  *         唤醒后需调用 enter_stop2_wakeup() 恢复
  */
static void enter_stop2(void)
{
    /* 关闭外设时钟以降低功耗 */
    board_pwr_set(PWR_ADC_EN, false);
    board_pwr_set(PWR_5V_EN, false);
    board_pwr_set(PWR_GNSS_EN, false);
    board_led_set(LED_OFF);

    /* 挂起无线电 */
    Radio.Sleep();

    /* 关闭OLED */
    OLED_DisPlay_Off();

    /* 失能未使用的外设时钟 */
    /* 注意: GPIOA和GPIOB保留 — PA15(BTN_DOWN)/PB2(BTN_OK)/PB12(BTN_UP)需要EXTI唤醒 */
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOH_CLK_DISABLE();
    __HAL_RCC_USART1_CLK_DISABLE();
    __HAL_RCC_I2C1_CLK_DISABLE();
    __HAL_RCC_I2C2_CLK_DISABLE();
    __HAL_RCC_ADC_CLK_DISABLE();
    __HAL_RCC_TIM16_CLK_DISABLE();

    /* 清除低功耗标志和唤醒标志 */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_STOP2);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

    /* 使能内部唤醒线 (RTC闹钟等可从STOP2唤醒) */
    HAL_PWREx_EnableInternalWakeUpLine();

    HAL_SuspendTick();
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
    /* ─── 从STOP2唤醒后从下一行继续执行 ─── */
    HAL_ResumeTick();
}

/**
  * @brief  STOP2唤醒后恢复外设
  */
static void exit_stop2(void)
{
    /* 重新使能时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();
    __HAL_RCC_ADC_CLK_ENABLE();
    __HAL_RCC_TIM16_CLK_ENABLE();

    /* 重新初始化I2C外设 (STOP2后寄存器丢失) */
    HAL_I2C_Init(&hi2c1);
    HAL_I2C_Init(&hi2c2);

    /* 重新初始化USART (GNSS模块, 如果正在使用) */
    /* 注意: USART2由CO驱动管理, 这里不初始化 */

    /* 重新初始OLED */
    OLED_Init();
    OLED_DisPlay_On();

    /* 恢复LED */
    board_led_set(LED_BLINK_SLOW);
}

/* 按键事件队列 (ISR写入，主循环读取) */
#define BTN_QUEUE_SIZE 8
typedef struct {
    board_btn_t btn;
    btn_event_t event;
} btn_event_entry_t;
static volatile btn_event_entry_t btn_queue[BTN_QUEUE_SIZE];
static volatile uint8_t btn_queue_head = 0;
static volatile uint8_t btn_queue_tail = 0;

/*============================================================================
 *                              私有函数 - 调试串口输出
 *===========================================================================*/
/**
  * @brief  通过USART2输出调试信息 (阻塞方式, 简单可靠)
  */
static void debug_print(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0) {
        if ((uint32_t)len >= sizeof(buf)) len = sizeof(buf) - 1;
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, 100);
    }
}

/**
  * @brief  输出浮点值 (整数+1位小数, 避免newlib-nano不支持%f)
  */
static void debug_print_float(const char *prefix, float val, const char *suffix)
{
    int ival = (int)(val * 10);
    if (ival < 0) {
        debug_print("%s-%d.%d%s", prefix, -ival / 10, -ival % 10, suffix);
    } else {
        debug_print("%s%d.%d%s", prefix, ival / 10, ival % 10, suffix);
    }
}

/*============================================================================
 *                              私有函数 - 报警判定
 *===========================================================================*/
/**
  * @brief  根据传感器数据更新运行模式 (含迟滞)
  */
static void update_run_mode(void)
{
    const app_config_t *cfg = app_config_get();
    run_mode_t new_mode = RUN_MODE_NORMAL;
    
    // 判定温度火警 (最高优先级, 含迟滞)
    if (sensor_data.sht40_valid) {
        if (current_mode == RUN_MODE_FIRE_ALARM) {
            if (sensor_data.temperature > cfg->temp_alarm_recovery) {
                new_mode = RUN_MODE_FIRE_ALARM;
            }
        } else {
            if (sensor_data.temperature > cfg->temp_alarm_threshold) {
                new_mode = RUN_MODE_FIRE_ALARM;
            }
        }
    }

    // 判定预警 (仅在非火警时)
    if (new_mode != RUN_MODE_FIRE_ALARM) {
        bool temp_warning = false;
        bool co_warning = false;
        bool hum_warning = false;

        // 温度预警 (含迟滞)
        if (sensor_data.sht40_valid) {
            if (current_mode == RUN_MODE_EARLY_WARNING) {
                if (sensor_data.temperature > cfg->temp_warning_recovery) {
                    temp_warning = true;
                }
            } else {
                if (sensor_data.temperature > cfg->temp_warning_threshold) {
                    temp_warning = true;
                }
            }
        }
        // CO预警
        if (sensor_data.co_valid) {
            if (current_mode == RUN_MODE_EARLY_WARNING) {
                if (sensor_data.co_concentration > cfg->co_warning_recovery) {
                    co_warning = true;
                }
            } else {
                if (sensor_data.co_concentration > cfg->co_warning_threshold) {
                    co_warning = true;
                }
            }
        }
        
        // 低湿预警
        if (sensor_data.sht40_valid) {
            if (current_mode == RUN_MODE_EARLY_WARNING) {
                if (sensor_data.humidity < cfg->hum_warning_recovery) {
                    hum_warning = true;
                }
            } else {
                if (sensor_data.humidity < cfg->hum_warning_threshold) {
                    hum_warning = true;
                }
            }
        }
        
        if (temp_warning || co_warning || hum_warning) {
            new_mode = RUN_MODE_EARLY_WARNING;
        }
    }
    
    // 模式变化时更新LED
    if (new_mode != current_mode) {
        // 预警/火警模式LED快闪
        if (new_mode == RUN_MODE_FIRE_ALARM || new_mode == RUN_MODE_EARLY_WARNING) {
            board_led_set(LED_BLINK_FAST);
        } else if (new_mode == RUN_MODE_NORMAL) {
            board_led_set(LED_BLINK_SLOW);
        }
    }
    
    current_mode = new_mode;
}

/*============================================================================
 *                              私有函数 - 报警标志
 *===========================================================================*/
static uint8_t calc_alarm_flag(void)
{
    const app_config_t *cfg = app_config_get();
    uint8_t flag = ALARM_FLAG_NONE;
    
    if (sensor_data.sht40_valid && sensor_data.temperature > cfg->temp_alarm_threshold) {
        flag |= ALARM_FLAG_TEMPERATURE;
    }
    if (sensor_data.co_valid && sensor_data.co_concentration > cfg->co_warning_threshold) {
        flag |= ALARM_FLAG_CO;
    }
    if (sensor_data.sht40_valid && sensor_data.humidity < cfg->hum_warning_threshold) {
        flag |= ALARM_FLAG_HUMIDITY;
    }
    
    return flag;
}

/*============================================================================
 *                              私有函数 - 传感器采集
 *===========================================================================*/
static bool do_measure(void)
{
    bool any_valid = false;

    // 采集SHT40温湿度（含重试）
    sht40_data_t sht40_data;
    bool sht40_ok = false;
    for (int retry = 0; retry <= SHT40_MEASURE_RETRY_COUNT; retry++) {
        if (sht40_read(&sht40_data, SHT40_PRECISION_HIGH)) {
            sht40_ok = true;
            break;
        }
        // 重试前复位传感器
        debug_print("[SHT40] Read failed (attempt %d/%d), resetting...\r\n",
                    retry + 1, SHT40_MEASURE_RETRY_COUNT + 1);
        sht40_reset();
        HAL_Delay(5);
    }
    if (sht40_ok) {
        sensor_data.temperature = sht40_data.temperature;
        sensor_data.humidity = sht40_data.humidity;
        sensor_data.sht40_valid = true;
        any_valid = true;
        error_flags &= ~ERROR_FLAG_SHT40;  // 清除之前的错误标志
        debug_print_float("[SHT40] T=", sht40_data.temperature, "C ");
        debug_print_float("H=", sht40_data.humidity, "%RH\r\n");
    } else {
        sensor_data.sht40_valid = false;
        error_flags |= ERROR_FLAG_SHT40;
        debug_print("[SHT40] Read FAILED after %d attempts\r\n", SHT40_MEASURE_RETRY_COUNT + 1);
    }
    
    // 采集CO浓度
    {
        const co_data_t *co = co_get_data();
        if (co->valid && !co->warming_up) {
            sensor_data.co_concentration = (float)co->concentration;
            sensor_data.co_valid = true;
            debug_print("[CO] %u ppm\r\n", co->concentration);
        } else {
            sensor_data.co_concentration = 0.0f;
            sensor_data.co_valid = false;
            if (co_get_state() == CO_STATE_WARMUP) {
                debug_print("[CO] Warming up...\r\n");
            } else if (!co_is_present()) {
                debug_print("[CO] No data received\r\n");
            }
        }
    }
    
    // 采集电池电压
    sensor_data.battery_voltage = board_adc_read_battery();
    
    // 调试输出ADC信息
    {
        uint16_t vrefint_cal = board_get_vrefint_cal();
        uint16_t vrefint_raw = board_adc_read_vrefint();
        int vref_mv = (int)(3300UL * vrefint_raw / 4096);
        int vref_typ_mv = (int)(BOARD_VREFINT_TYPICAL * 1000);
        debug_print("[ADC] vrefint_cal=%u vrefint_raw=%u\r\n", vrefint_cal, vrefint_raw);
        debug_print("[ADC] VREFINT_typ=%dmV VREFINT_meas=%dmV\r\n", vref_typ_mv, vref_mv);
        debug_print_float("[ADC] V_bat=", sensor_data.battery_voltage, "V\r\n");
    }
    
    // 更新运行模式
    update_run_mode();
    
    return any_valid;
}

/*============================================================================
 *                              私有函数 - 数据发送 (协议版)
 *===========================================================================*/
/**
  * @brief  使用新协议构建帧并通过LoRa发送
  * @retval true: 已提交发送, false: 失败
  */
static bool do_transmit(void)
{
    const app_config_t *cfg = app_config_get();

    proto_sensor_data_t sd;
    sd.temperature      = (int16_t)(sensor_data.temperature * 10);
    sd.humidity         = (uint16_t)(sensor_data.humidity * 10);
    sd.co_concentration = (uint16_t)(sensor_data.co_concentration);
    sd.battery_mv       = (uint16_t)(sensor_data.battery_voltage * 1000);
    sd.alarm_flag       = calc_alarm_flag();

    uint8_t sf_flag = (cfg->proto_sf_current == cfg->proto_sf_backup)
                      ? PROTO_SF_FLAG_BACKUP : PROTO_SF_FLAG_PRIMARY;
    uint16_t seq = app_config_next_msg_seq();

    /* 构建附加TLV: 配置上报 + 指令执行状态 */
    uint8_t extra[64];
    uint8_t extra_pos = 0;

    if (cfg_report_pending) {
        extra_pos += proto_tlv_config_report(extra + extra_pos, sizeof(extra) - extra_pos,
            cfg->measure_interval_normal, cfg->measure_interval_warning, cfg->measure_interval_alarm,
            cfg->transmit_interval_normal, cfg->transmit_interval_warning, cfg->transmit_interval_alarm,
            (int16_t)(cfg->temp_warning_threshold * 10), (int16_t)(cfg->temp_warning_recovery * 10),
            (int16_t)(cfg->temp_alarm_threshold * 10), (int16_t)(cfg->temp_alarm_recovery * 10),
            (uint16_t)cfg->co_warning_threshold, (uint16_t)cfg->co_warning_recovery,
            (uint16_t)(cfg->hum_warning_threshold * 10), (uint16_t)(cfg->hum_warning_recovery * 10));
        cfg_report_pending = false;
        debug_print("[PROTO] Config report appended (30B)\r\n");
    }
    if (cmd_status_count > 0) {
        /* 将队列中每项 (type+status) 编码为单个 0xFF TLV, 长度 = count × 2 */
        extra_pos += proto_tlv_encode(extra + extra_pos, sizeof(extra) - extra_pos,
                                      TLV_CMD_STATUS, cmd_status_queue, cmd_status_count * 2);
        debug_print("[PROTO] CMD status appended (%u cmds)\r\n", cmd_status_count);
        cmd_status_count = 0;
    }

    uint16_t frame_len = proto_build_data_frame(proto_tx_buf, PROTO_TX_BUF_SIZE,
        &sd, cfg->proto_parent_addr, cfg->proto_short_addr, seq,
        sf_flag, cfg->proto_power_level,
        extra_pos > 0 ? extra : NULL, extra_pos);
    if (frame_len == 0) return false;

    app_lora_configure(cfg->proto_sf_current,
                       proto_power_level_to_dbm(cfg->proto_power_level));

    if (!app_lora_send(proto_tx_buf, frame_len)) {
        debug_print("[PROTO] TX fail, cache to EEPROM\r\n");
        cache_record_t cr = { .timestamp = HAL_GetTick()/1000,
            .temperature = sensor_data.temperature,
            .humidity = sensor_data.humidity,
            .co_concentration = sensor_data.co_concentration,
            .battery_mv = sd.battery_mv,
            .alarm_flag = sd.alarm_flag, .retry_count = 0 };
        app_cache_push(&cr);
        return false;
    }

    tx_awaiting_ack = true;

    proto_retransmit_entry_t e;
    e.timestamp = HAL_GetTick(); e.temperature = sd.temperature;
    e.humidity = sd.humidity; e.co_concentration = sd.co_concentration;
    e.battery_mv = sd.battery_mv; e.alarm_flag = sd.alarm_flag;
    e.msg_seq = seq;
    proto_retransmit_push(&e);

    last_tx_seq = seq;
    return true;
}

/*============================================================================
 *                              状态处理函数
 *===========================================================================*/

/*--- INIT ---*/
static void state_init_enter(void)
{
    // 等待OLED上电（OLED需要时间初始化内部电路）
    HAL_Delay(200);
    
    // 初始化板级IO
    board_io_init();
    
    // 初始化按键驱动
    btn_driver_init();
    
    // 检测并初始化SHT40（含重试）
    {
        bool sht40_found = false;
        debug_print("[SHT40] Initializing...\r\n");
        for (int retry = 0; retry < SHT40_INIT_RETRY_COUNT; retry++) {
            debug_print("[SHT40] Init attempt %d/%d\r\n", retry + 1, SHT40_INIT_RETRY_COUNT);
            if (sht40_init()) {
                sht40_found = true;
                debug_print("[SHT40] Init OK\r\n");
                break;
            }
            debug_print("[SHT40] Init failed, retrying...\r\n");
            HAL_Delay(SHT40_INIT_RETRY_DELAY_MS);
        }
        if (!sht40_found) {
            error_flags |= ERROR_FLAG_SHT40;
            debug_print("[SHT40] Init FAILED after %d attempts\r\n", SHT40_INIT_RETRY_COUNT);
        }
        // 检测I2C1总线上的SHT40设备
        debug_print("[SHT40] IsDeviceReady check: %s\r\n",
                    sht40_is_present() ? "FOUND" : "NOT FOUND");
    }
    
    // 检测并初始化EEPROM
    if (!eeprom_is_present()) {
        error_flags |= ERROR_FLAG_EEPROM;
    }
    
    // 初始化配置模块
    app_config_init();
    
    // 初始化缓存队列
    app_cache_init();
    
    // 初始化OLED菜单
    HAL_I2C_DeInit(&hi2c2);
    HAL_I2C_Init(&hi2c2);
    app_menu_init();
    
    // 简单检测OLED
    if (HAL_I2C_IsDeviceReady(&hi2c2, 0x78, 1, 50) != HAL_OK) {
        error_flags |= ERROR_FLAG_OLED;
    }
    
    // 初始化LoRa
    if (!app_lora_init()) {
        error_flags |= ERROR_FLAG_LORA;
    }
    
    // 初始化协议模块 (重置重传队列, 候选列表等)
    proto_retransmit_init();
    proto_candidates_clear();
    
    // 初始化GNSS驱动
    gnss_init();
    
    // 检查是否已有保存的位置数据
    gnss_location_saved = app_config_has_location();
    if (gnss_location_saved) {
        debug_print("[GNSS] Location already saved in EEPROM\r\n");
    } else {
        debug_print("[GNSS] No location data, need to acquire position\r\n");
    }
    
    // LED指示：初始化完成，慢闪
    board_led_set(LED_BLINK_SLOW);
    
    // 启动TIM17 (按键扫描)
    HAL_TIM_Base_Start_IT(&htim17);

    // 启动CO传感器 (预热30s, 与注册流程并行)
    co_init();
    if (!co_start()) {
        debug_print("[CO] Start FAILED\r\n");
    } else {
        debug_print("[CO] Started, warming up (%ds)...\r\n", CO_WARMUP_MS / 1000);
    }
}

/*--- IDLE ---*/
static void state_idle_enter(void)
{
    // 首次进入IDLE时设置首次测量延时（上电后5s即测量，无需等待整个采集周期）
    if (last_measure_tick == 0) {
        last_measure_tick = HAL_GetTick() - (uint32_t)app_config_get_measure_interval(current_mode) * 1000 + FIRST_MEASURE_DELAY_MS;
    }
    
    if (last_transmit_tick == 0) {
        last_transmit_tick = HAL_GetTick();
    }
    
    // 刷新OLED主页显示
    app_menu_display();
}

static app_state_t state_idle_process(void)
{
    // LED指示：慢闪(正常待机)
    board_led_set(LED_BLINK_SLOW);
    
    // 未获短地址不允许发送传感器数据
    if (!app_config_has_short_addr()) {
        return STATE_IDLE;
    }
    
    // 刷新OLED主页显示
    app_menu_display();
    
    // 检查重新部署请求
    if (app_menu_has_redeploy_request()) {
        // 清除位置数据，重置注册状态，进入GNSS定位
        app_config_clear_location();
        gnss_location_saved = false;
        registration_done = false;
        debug_print("[REDEPLOY] Location cleared, starting GNSS...\r\n");
        return STATE_GNSS;
    }
    
    // 主页OK按键由app_menu处理，若进入设置则切换状态
    if (app_menu_is_settings_page()) {
        return STATE_SETTINGS;
    }
    
    // 检查是否到达采集时间
    uint16_t measure_interval = app_config_get_measure_interval(current_mode);
    uint32_t now = HAL_GetTick();
    
    if ((now - last_measure_tick) >= (uint32_t)measure_interval * 1000) {
        return STATE_MEASURE;
    }
    
    // 检查是否到达发送时间
    uint16_t transmit_interval = app_config_get_transmit_interval(current_mode);
    
    if ((now - last_transmit_tick) >= (uint32_t)transmit_interval * 1000) {
        return STATE_TRANSMIT;
    }
    
    // 无任务则进入休眠 (火警模式/屏幕亮起时不休眠)
    if (current_mode != RUN_MODE_FIRE_ALARM && !app_menu_is_screen_on()) {
        return STATE_SLEEP;
    }

    return STATE_IDLE;
}

/*--- SETTINGS ---*/
static void state_settings_enter(void)
{
    board_led_set(LED_ON);  // 设置时常亮LED
}

static app_state_t state_settings_process(void)
{
    // 刷新菜单显示
    app_menu_display();
    
    // 检查是否退出设置（返回主页）
    if (app_menu_is_main_page() && !app_menu_is_settings_page()) {
        return STATE_IDLE;
    }
    
    return STATE_SETTINGS;
}

/*--- MEASURE ---*/
static void state_measure_enter(void)
{
}

static app_state_t state_measure_process(void)
{
    do_measure();
    last_measure_tick = HAL_GetTick();
    
    // 采集完成后立即刷新显示（让用户看到最新数据）
    app_menu_display();
    
    // 采集完成后检查是否需要发送
    uint16_t transmit_interval = app_config_get_transmit_interval(current_mode);
    if ((HAL_GetTick() - last_transmit_tick) >= (uint32_t)transmit_interval * 1000) {
        return STATE_TRANSMIT;
    }
    
    return STATE_IDLE;
}

/*--- GNSS ---*/
static void state_gnss_enter(void)
{
    debug_print("[GNSS] Starting GNSS module...\r\n");
    
    // LED指示：快闪(定位中)
    board_led_set(LED_BLINK_FAST);
    
    if (!gnss_start()) {
        debug_print("[GNSS] Failed to start, skipping\r\n");
        error_flags |= ERROR_FLAG_GNSS;
        // 启动失败，直接跳到IDLE
    }
}

static app_state_t state_gnss_process(void)
{
    // 处理GNSS数据
    gnss_process();

    // 刷新OLED显示 (允许用户查看定位状态)
    app_menu_display();
    
    gnss_state_t gs = gnss_get_state();
    
    switch (gs) {
        case GNSS_STATE_FIXED: {
            // 定位成功，保存位置到EEPROM
            const gnss_location_t *loc = gnss_get_location();
            debug_print("[GNSS] Fixed! Sat=%d HDOP=%d.%d\r\n",
                        loc->satellites, loc->hdop_x10 / 10, loc->hdop_x10 % 10);
            debug_print("[GNSS] %c%d:%d.%04d  %c%d:%d.%04d\r\n",
                        loc->lat_dir, loc->lat_deg, loc->lat_min, loc->lat_frac,
                        loc->lon_dir, loc->lon_deg, loc->lon_min, loc->lon_frac);
            
            // 保存到EEPROM
            gnss_stored_location_t stored;
            stored.lat_dir  = loc->lat_dir;
            stored.lat_deg  = loc->lat_deg;
            stored.lat_min  = loc->lat_min;
            stored.lat_frac = loc->lat_frac;
            stored.lon_dir  = loc->lon_dir;
            stored.lon_deg  = loc->lon_deg;
            stored.lon_min  = loc->lon_min;
            stored.lon_frac = loc->lon_frac;
            
            if (app_config_save_location(&stored)) {
                gnss_location_saved = true;
                debug_print("[GNSS] Location saved to EEPROM\r\n");
            } else {
                debug_print("[GNSS] Failed to save location\r\n");
            }
            
            // 关闭GNSS模块
            gnss_stop();
            debug_print("[GNSS] Module stopped\r\n");
            
            // 进入协议扫描流程 (调试模式不保存短地址)
            return STATE_PROTO_SCAN;
        }
        
        case GNSS_STATE_TIMEOUT:
            debug_print("[GNSS] Fix timeout, retrying...\r\n");
            gnss_stop();
            gnss_start();
            return STATE_GNSS;

        case GNSS_STATE_ERROR:
            debug_print("[GNSS] Error, retrying...\r\n");
            error_flags |= ERROR_FLAG_GNSS;
            gnss_stop();
            HAL_Delay(1000);
            gnss_start();
            return STATE_GNSS;
        
        case GNSS_STATE_SEARCHING: {
            // 搜星中，定期输出状态
            static uint32_t last_gnss_print = 0;
            uint32_t now = HAL_GetTick();
            if ((now - last_gnss_print) >= 10000) {    // 每10s输出一次
                last_gnss_print = now;
                const gnss_location_t *loc = gnss_get_location();
                debug_print("[GNSS] Searching... sats=%d elapsed=%ds\r\n",
                            loc->satellites, (int)(gnss_get_elapsed_ms() / 1000));
            }
            // 允许按键中断定位 (进入设置)
            if (app_menu_is_settings_page()) {
                gnss_stop();
                return STATE_SETTINGS;
            }
            break;
        }
        
        case GNSS_STATE_BOOTING:
            // 等待模块启动
            break;
        
        default:
            break;
    }
    
    return STATE_GNSS;
}

/*--- TRANSMIT ---*/
static void state_transmit_enter(void)
{
    board_led_set(LED_BLINK_FAST);
    tx_start_tick = HAL_GetTick();
    
    // 提交发送
    if (!do_transmit()) {
        debug_print("[LORA] TX submit failed, back to IDLE\r\n");
        last_transmit_tick = HAL_GetTick();
    }
}

static app_state_t state_transmit_process(void)
{
    uint32_t elapsed = HAL_GetTick() - tx_start_tick;
    
    if (app_lora_tx_done()) {
        last_transmit_tick = HAL_GetTick();
        
        if (app_lora_tx_success()) {
            tx_retry_count = 0;
            debug_print("[LORA] TX success, enter RX\r\n");
            return STATE_RECEIVE;
        }
        
        if (elapsed < (TX_TIMEOUT_MS / 2)) {
            if (tx_retry_count < TX_MAX_RETRY) {
                tx_retry_count++;
                debug_print("[DBG] Spurious TX timeout at %lums, retry #%d\r\n",
                            (unsigned long)elapsed, tx_retry_count);
                state_transmit_enter();
                return STATE_TRANSMIT;
            }
            debug_print("[LORA] TX spurious timeout retries exhausted\r\n");
            return STATE_IDLE;
        }
        
        debug_print("[LORA] TX timeout, back to IDLE\r\n");
        return STATE_IDLE;
    }
    
    if (elapsed >= TX_TIMEOUT_MS) {
        debug_print("[LORA] TX timeout (%dms app)\r\n", TX_TIMEOUT_MS);
        tx_retry_count = 0;
        last_transmit_tick = HAL_GetTick();
        return STATE_IDLE;
    }
    
    return STATE_TRANSMIT;
}

/*============================================================================
 *                              私有函数 - RX分发 + 指令执行
 *===========================================================================*/

/* 执行单条下行指令, 返回状态码 (TLV 0xFF) */
static uint8_t execute_one_command(const proto_cmd_t *cmd)
{
    app_config_t cfg = *app_config_get();
    bool updated = false;

    switch (cmd->type) {
        case TLV_SET_INTERVAL:  /* 0x20 采集间隔 */
            if (cmd->len >= 6) {
                cfg.measure_interval_normal  = (uint16_t)cmd->value[0] | ((uint16_t)cmd->value[1] << 8);
                cfg.measure_interval_warning = (uint16_t)cmd->value[2] | ((uint16_t)cmd->value[3] << 8);
                cfg.measure_interval_alarm   = (uint16_t)cmd->value[4] | ((uint16_t)cmd->value[5] << 8);
                updated = true;
                debug_print("[CMD] Set measure intervals: %d/%d/%d\r\n",
                            cfg.measure_interval_normal,
                            cfg.measure_interval_warning,
                            cfg.measure_interval_alarm);
            }
            break;

        case TLV_SET_TX_INTERVAL:  /* 0x21 发送间隔 */
            if (cmd->len >= 6) {
                cfg.transmit_interval_normal  = (uint16_t)cmd->value[0] | ((uint16_t)cmd->value[1] << 8);
                cfg.transmit_interval_warning = (uint16_t)cmd->value[2] | ((uint16_t)cmd->value[3] << 8);
                cfg.transmit_interval_alarm   = (uint16_t)cmd->value[4] | ((uint16_t)cmd->value[5] << 8);
                updated = true;
                debug_print("[CMD] Set TX intervals: %d/%d/%d\r\n",
                            cfg.transmit_interval_normal,
                            cfg.transmit_interval_warning,
                            cfg.transmit_interval_alarm);
            }
            break;

        case TLV_SET_TEMP:  /* 0x22 温度阈值 */
            if (cmd->len >= 8) {
                float tw  = (int16_t)((uint16_t)cmd->value[0] | ((uint16_t)cmd->value[1] << 8)) * 0.1f;
                float twr = (int16_t)((uint16_t)cmd->value[2] | ((uint16_t)cmd->value[3] << 8)) * 0.1f;
                float ta  = (int16_t)((uint16_t)cmd->value[4] | ((uint16_t)cmd->value[5] << 8)) * 0.1f;
                float tar = (int16_t)((uint16_t)cmd->value[6] | ((uint16_t)cmd->value[7] << 8)) * 0.1f;
                cfg.temp_warning_threshold = tw;
                cfg.temp_warning_recovery  = twr;
                cfg.temp_alarm_threshold   = ta;
                cfg.temp_alarm_recovery    = tar;
                updated = true;
                debug_print("[CMD] Set temp: warn=%.1f/%.1f alarm=%.1f/%.1f\r\n", tw, twr, ta, tar);
            }
            break;

        case TLV_SET_CO:  /* 0x23 CO阈值 */
            if (cmd->len >= 4) {
                cfg.co_warning_threshold  = (float)((uint16_t)cmd->value[0] | ((uint16_t)cmd->value[1] << 8));
                cfg.co_warning_recovery   = (float)((uint16_t)cmd->value[2] | ((uint16_t)cmd->value[3] << 8));
                updated = true;
                debug_print("[CMD] Set CO: warn=%.0f ppm rec=%.0f ppm\r\n",
                            cfg.co_warning_threshold, cfg.co_warning_recovery);
            }
            break;

        case TLV_SET_HUMIDITY:  /* 0x24 湿度阈值 */
            if (cmd->len >= 4) {
                cfg.hum_warning_threshold  = (float)((uint16_t)cmd->value[0] | ((uint16_t)cmd->value[1] << 8)) * 0.1f;
                cfg.hum_warning_recovery   = (float)((uint16_t)cmd->value[2] | ((uint16_t)cmd->value[3] << 8)) * 0.1f;
                updated = true;
                debug_print("[CMD] Set humidity: warn=%.1f rec=%.1f\r\n",
                            cfg.hum_warning_threshold, cfg.hum_warning_recovery);
            }
            break;

        case TLV_SET_RF:  /* 0x25 RF参数 */
            if (cmd->len >= 3) {
                cfg.proto_sf_primary  = cmd->value[0];
                cfg.proto_sf_backup   = cmd->value[1];
                cfg.proto_sf_current  = cmd->value[0]; /* 切回主SF */
                cfg.proto_power_level = cmd->value[2];
                updated = true;
                debug_print("[CMD] Set RF: SF=%d/%d power=%d\r\n",
                            cfg.proto_sf_primary, cfg.proto_sf_backup, cfg.proto_power_level);
            }
            break;

        case TLV_IMMEDIATE_REPORT:  /* 0x26 立即上报 */
            debug_print("[CMD] Immediate report requested\r\n");
            if (cmd_status_count < CMD_STATUS_MAX) {
                cmd_status_queue[cmd_status_count * 2]     = cmd->type;
                cmd_status_queue[cmd_status_count * 2 + 1] = PROTO_ERR_SUCCESS;
                cmd_status_count++;
            }
            return PROTO_ERR_SUCCESS;

        case TLV_REQUEST_RETRANSMIT:  /* 0x27 补发 */
            if (cmd->len >= 2) {
                uint16_t seq = (uint16_t)cmd->value[0] | ((uint16_t)cmd->value[1] << 8);
                /* TODO: 从EEPROM缓存取出指定序号数据, 下次上行附带 */
                debug_print("[CMD] Retransmit requested seq=%d\r\n", seq);
            }
            if (cmd_status_count < CMD_STATUS_MAX) {
                cmd_status_queue[cmd_status_count * 2]     = cmd->type;
                cmd_status_queue[cmd_status_count * 2 + 1] = PROTO_ERR_SUCCESS;
                cmd_status_count++;
            }
            return PROTO_ERR_SUCCESS;

        case TLV_UPDATE_PARENT:  /* 0x28 更新父节点 */
            if (cmd->len >= 2) {
                cfg.proto_parent_addr = (uint16_t)cmd->value[0] | ((uint16_t)cmd->value[1] << 8);
                updated = true;
                debug_print("[CMD] Parent changed to 0x%04X\r\n", cfg.proto_parent_addr);
            }
            break;

        case TLV_QUERY_CONFIG:  /* 0x29 查询配置 */
            debug_print("[CMD] Config query requested\r\n");
            cfg_report_pending = true;
            if (cmd_status_count < CMD_STATUS_MAX) {
                cmd_status_queue[cmd_status_count * 2]     = cmd->type;
                cmd_status_queue[cmd_status_count * 2 + 1] = PROTO_ERR_SUCCESS;
                cmd_status_count++;
            }
            return PROTO_ERR_SUCCESS;

        case TLV_REMOTE_RESET:  /* 0x2A 远程复位 */
            debug_print("[CMD] Remote reset! (5s delay)\r\n");
            HAL_Delay(5000);
            NVIC_SystemReset();
            return PROTO_ERR_SUCCESS;

        default:
            return PROTO_ERR_PARAM;
    }

    if (updated) {
        app_config_set(&cfg);
        if (cmd->type == TLV_SET_RF) {
            app_lora_configure(cfg.proto_sf_current,
                               proto_power_level_to_dbm(cfg.proto_power_level));
        }
        if (cmd_status_count < CMD_STATUS_MAX) {
            cmd_status_queue[cmd_status_count * 2]     = cmd->type;
            cmd_status_queue[cmd_status_count * 2 + 1] = PROTO_ERR_SUCCESS;
            cmd_status_count++;
        }
        return PROTO_ERR_SUCCESS;
    }
    if (cmd_status_count < CMD_STATUS_MAX) {
        cmd_status_queue[cmd_status_count * 2]     = cmd->type;
        cmd_status_queue[cmd_status_count * 2 + 1] = PROTO_ERR_PARAM;
        cmd_status_count++;
    }
    return PROTO_ERR_PARAM;
}

/* 扫描帧中所有指令TLV并执行 */
static void execute_dl_commands(const uint8_t *buf, uint16_t len)
{
    proto_cmd_t cmds[8];
    proto_frame_t frame;
    if (!proto_parse_frame(buf, len, &frame)) return;

    uint8_t n = proto_parse_commands(frame.payload, frame.payload_len,
                                      cmds, 8);
    if (n == 0) return;

    debug_print("[CMD] %d command(s) received\r\n", n);
    for (uint8_t i = 0; i < n; i++) {
        uint8_t status = execute_one_command(&cmds[i]);
        if (status != PROTO_ERR_SUCCESS) {
            debug_print("[CMD] Command 0x%02X failed (status=%d)\r\n",
                        cmds[i].type, status);
        }
    }
}

static void handle_received_frame(const uint8_t *buf, uint16_t len,
                                   int16_t rssi, int8_t snr)
{
    proto_rx_action_t act = proto_dispatch_rx(buf, len, rssi, snr);

    switch (act) {
        case PROTO_RX_ACK_CONFIRMED: {
            const proto_ack_info_t *ack = proto_get_last_ack();
            debug_print("[PROTO] ACK last_seq=%d rssi=%d snr=%d\r\n",
                        ack->last_in_seq, ack->rssi, ack->snr);
            tx_awaiting_ack = false;
            execute_dl_commands(buf, len);

            /* ACK确认后, 若重传队列已空则从EEPROM缓存取一笔继续补发 */
            if (proto_retransmit_count() == 0) {
                cache_record_t cached;
                if (app_cache_pop(&cached)) {
                    uint16_t hist_seq = app_config_next_msg_seq();
                    proto_retransmit_entry_t hist;
                    hist.timestamp = cached.timestamp * 1000;
                    hist.temperature = (int16_t)(cached.temperature * 10);
                    hist.humidity = (uint16_t)(cached.humidity * 10);
                    hist.co_concentration = (uint16_t)(cached.co_concentration);
                    hist.battery_mv = cached.battery_mv;
                    hist.alarm_flag = cached.alarm_flag;
                    hist.msg_seq = hist_seq;
                    proto_retransmit_push(&hist);
                    debug_print("[CACHE] Draining: %u remaining\r\n",
                                app_cache_count());
                }
            }
            break;
        }
        case PROTO_RX_REGISTERED:
            debug_print("[PROTO] *** REGISTERED addr=0x%04X ***\r\n",
                        proto_get_assigned_addr());
            app_config_set_short_addr_ram(proto_get_assigned_addr());
            registration_done = true;
            /* 注册完成可能附有初始配置指令 */
            execute_dl_commands(buf, len);
            break;
        case PROTO_RX_PROXY_ACK:
            /* 代理ACK也可能携带指令 */
            execute_dl_commands(buf, len);
            break;
        case PROTO_RX_PARSE_ERROR:
            debug_print("[PROTO] RX parse error\r\n");
            break;
        default:
            break;
    }
}

/*--- RECEIVE ---*/
static void state_receive_enter(void)
{
    board_led_set(LED_BLINK_FAST);
    rx_start_tick = HAL_GetTick();
    rx1_done = false;
    
    app_lora_start_rx(PROTO_RX_WINDOW_MS);
}

static app_state_t state_receive_process(void)
{
    uint32_t elapsed = HAL_GetTick() - rx_start_tick;
    
    // RX1窗口
    if (!rx1_done) {
        if (app_lora_rx_available()) {
            uint8_t rx_buf[LORA_RX_BUF_SIZE];
            int16_t rssi;
            int8_t snr;
            uint16_t rx_len = app_lora_get_rx_data(rx_buf, sizeof(rx_buf), &rssi, &snr);
            if (rx_len > 0) {
                handle_received_frame(rx_buf, rx_len, rssi, snr);
            }
            tx_awaiting_ack = false;
            return STATE_IDLE;
        }
        
        if (elapsed >= (PROTO_RX_DELAY_MS + PROTO_RX_WINDOW_MS) || 
            app_lora_rx_finished()) {
            rx1_done = true;
        }
    }
    
    // RX2窗口
    if (rx1_done && elapsed >= PROTO_RX2_DELAY_MS) {
        static bool rx2_started = false;
        if (!rx2_started) {
            app_lora_start_rx(PROTO_RX_WINDOW_MS);
            rx2_started = true;
        }
        
        if (app_lora_rx_available()) {
            uint8_t rx_buf[LORA_RX_BUF_SIZE];
            int16_t rssi;
            int8_t snr;
            uint16_t rx_len = app_lora_get_rx_data(rx_buf, sizeof(rx_buf), &rssi, &snr);
            if (rx_len > 0) {
                handle_received_frame(rx_buf, rx_len, rssi, snr);
            }
            tx_awaiting_ack = false;
            rx2_started = false;
            return STATE_IDLE;
        }
        
        if (elapsed >= (PROTO_RX2_DELAY_MS + PROTO_RX_WINDOW_MS)) {
            rx2_started = false;
            app_lora_stop_rx();
            // 未收到ACK, 缓存到EEPROM
            if (tx_awaiting_ack) {
                debug_print("[PROTO] No ACK in RX windows, caching to EEPROM\r\n");
                cache_record_t cr;
                cr.timestamp = HAL_GetTick() / 1000;
                cr.temperature = sensor_data.temperature;
                cr.humidity = sensor_data.humidity;
                cr.co_concentration = sensor_data.co_concentration;
                cr.battery_mv = (uint16_t)(sensor_data.battery_voltage * 1000);
                cr.alarm_flag = calc_alarm_flag();
                cr.retry_count = 0;
                app_cache_push(&cr);
                tx_awaiting_ack = false;
            }
            return STATE_IDLE;
        }
    }
    
    return STATE_RECEIVE;
}

/*--- PROTO_SCAN ---*/
static void state_proto_scan_enter(void)
{
    debug_print("[PROTO] === SCAN start (dur=%ds, SF=%d) ===\r\n",
                PROTO_SCAN_DURATION_MS / 1000, app_config_get()->proto_sf_current);
    board_led_set(LED_BLINK_FAST);
    scan_start_tick = HAL_GetTick();
    proto_candidates_clear();
    proto_candidates_reset_exclusions();
    
    // Radio已在 app_lora_init 中配好(SF7/私网/CRC), 直接开启接收
    const app_config_t *cfg = app_config_get();
    debug_print("[PROTO] SCAN radio: SF=%d freq=%d sync=private\r\n",
                cfg->proto_sf_current, LORA_RF_FREQUENCY);
    app_lora_start_scan_rx(cfg->proto_sf_current);  // 连续接收(rxContinuous=true, 与test_lora_rx一致)
}

static app_state_t state_proto_scan_process(void)
{
    uint32_t elapsed = HAL_GetTick() - scan_start_tick;

    // 刷新OLED (允许按键亮屏查看状态)
    app_menu_display();

    // 收集Beacon — 由协议模块统一处理
    if (app_lora_rx_available()) {
        uint8_t rx_buf[LORA_RX_BUF_SIZE];
        int16_t rssi;
        int8_t snr;
        uint16_t rx_len = app_lora_get_rx_data(rx_buf, sizeof(rx_buf), &rssi, &snr);
        if (rx_len > 0) {
            // 调试: 打印原始数据前几个字节
            {
                char hex[64]; int hpos = 0;
                hpos += snprintf(hex + hpos, sizeof(hex) - hpos, "[PROTO] SCAN raw(%d):", rx_len);
                for (uint16_t i = 0; i < rx_len && i < 16; i++)
                    hpos += snprintf(hex + hpos, sizeof(hex) - hpos, " %02X", rx_buf[i]);
                debug_print("%s\r\n", hex);
            }

            proto_rx_action_t act = proto_dispatch_rx(rx_buf, rx_len, rssi, snr);
            if (act == PROTO_RX_BEACON) {
                debug_print("[PROTO] SCAN: Beacon collected\r\n");
            } else if (act == PROTO_RX_PARSE_ERROR) {
                debug_print("[PROTO] SCAN: parse err (len=%d fc=0x%02X)\r\n",
                            rx_len, rx_buf[0]);
            } else {
                debug_print("[PROTO] SCAN: act=%d (ignored)\r\n", act);
            }
            // rxContinuous=true: Radio自动保持RX模式，无需手动重启
        }
    }

    // rxContinuous=true时Radio持续监听, 错误也不会退出RX模式, 无需恢复逻辑

    // 扫描超时
    if (elapsed >= PROTO_SCAN_DURATION_MS) {
        app_lora_stop_rx();
        
        debug_print("[PROTO] SCAN done: %d beacons collected\r\n", proto_candidates_count());
        for (uint8_t i = 0; i < proto_candidates_count(); i++) {
            const proto_beacon_candidate_t *c = proto_candidates_get(i);
            debug_print("[PROTO]  cand[%d]: addr=0x%04X type=%d hops=%d valid=%d rssi=%d\r\n",
                        i, c->sender_addr, c->beacon_type, c->hop_count, c->is_valid, c->rssi);
        }
        
        // 选择父节点 (终端: require_valid=true, current_hops=255表示未注册)
        const app_config_t *cfg = app_config_get();
        int8_t my_max_power = proto_power_level_to_dbm(PROTO_POWER_LEVEL_MAX);
        int8_t sensitivity = app_lora_get_sensitivity(cfg->proto_sf_current);
        uint16_t selected_parent = PROTO_ADDR_BROADCAST;
        
        if (proto_candidates_select(my_max_power, sensitivity, 6, true, 255, &selected_parent)) {
            // 必须已有位置数据才能进入注册
            if (!app_config_has_location()) {
                debug_print("[PROTO] No location, skip register → GNSS\r\n");
                return STATE_GNSS;
            }
            debug_print("[PROTO] Selected parent: 0x%04X\r\n", selected_parent);
            app_config_t tmp = *cfg;
            tmp.proto_parent_addr = selected_parent;
            app_config_set(&tmp);
            return STATE_PROTO_REGISTERING;
        }
        
        debug_print("[PROTO] No reachable parent found\r\n");
        // TODO: 进入ISOLATED状态
        return STATE_IDLE;
    }
    
    return STATE_PROTO_SCAN;
}

/*--- PROTO_REGISTERING ---*/

static void state_proto_registering_enter(void)
{
    const app_config_t *cfg = app_config_get();

    // 确保有位置数据才发送注册
    if (!app_config_has_location()) {
        debug_print("[PROTO] No GNSS location, cannot register!\r\n");
        return;
    }

    debug_print("[PROTO] === REG enter (parent=0x%04X) ===",
                cfg->proto_parent_addr);
    
    board_led_set(LED_BLINK_FAST);
    reg_start_tick      = HAL_GetTick();
    reg_phase           = REG_PHASE_TX;
    reg_phase_start_tick = reg_start_tick;
    reg_rx_started      = false;
    reg_got_proxy_ack   = false;
    
    // 构建注册帧 (协议模块)
    uint8_t uid[12]; board_get_uid(uid);
    proto_gnss_location_t gloc;
    const proto_gnss_location_t *p_gloc = NULL;
    gnss_stored_location_t stored_loc;
    if (app_config_load_location(&stored_loc)) {
        gloc.lat_dir  = stored_loc.lat_dir;  gloc.lat_deg  = stored_loc.lat_deg;
        gloc.lat_min  = stored_loc.lat_min;  gloc.lat_frac = stored_loc.lat_frac;
        gloc.lon_dir  = stored_loc.lon_dir;  gloc.lon_deg  = stored_loc.lon_deg;
        gloc.lon_min  = stored_loc.lon_min;  gloc.lon_frac = stored_loc.lon_frac;
        p_gloc = &gloc;
    }

    uint16_t frame_len = proto_build_register_frame(proto_tx_buf, PROTO_TX_BUF_SIZE,
        uid, p_gloc, cfg->proto_parent_addr, cfg->proto_power_level);
    if (frame_len > 0) {
        debug_print("[PROTO] REG frame len=%d%s\r\n", frame_len,
                    p_gloc ? " (+location)" : " (no location)");
        app_lora_configure(cfg->proto_sf_current, 14);
        app_lora_send(proto_tx_buf, frame_len);
    }
}

static bool handle_reg_rx_data(void)
{
    uint8_t rx_buf[LORA_RX_BUF_SIZE];
    int16_t rssi;
    int8_t snr;
    uint16_t rx_len = app_lora_get_rx_data(rx_buf, sizeof(rx_buf), &rssi, &snr);
    if (rx_len == 0) return false;

    proto_rx_action_t act = proto_dispatch_rx(rx_buf, rx_len, rssi, snr);

    switch (act) {
        case PROTO_RX_REGISTERED:
            debug_print("[PROTO] *** REGISTERED! addr=0x%04X ***\r\n",
                        proto_get_assigned_addr());
            app_config_set_short_addr_ram(proto_get_assigned_addr());
            registration_done = true;
            return true;

        case PROTO_RX_PROXY_ACK:
            debug_print("[PROTO] Proxy ACK from relay, extend RX for final\r\n");
            reg_got_proxy_ack = true;
            reg_phase = REG_PHASE_EXTEND;
            reg_phase_start_tick = HAL_GetTick();
            reg_rx_started = false;
            return false;

        default:
            return false;
    }
}

static app_state_t state_proto_registering_process(void)
{
    uint32_t elapsed = HAL_GetTick() - reg_start_tick;
    uint32_t phase_elapsed = HAL_GetTick() - reg_phase_start_tick;

    // 刷新OLED (允许按键亮屏查看状态)
    app_menu_display();

    // Phase 0: 等待TX完成
    if (reg_phase == REG_PHASE_TX) {
        if (!app_lora_tx_done()) return STATE_PROTO_REGISTERING;
        
        if (!app_lora_tx_success()) {
            debug_print("[PROTO] REG TX failed/timeout, retry\r\n");
            // 重发
            state_proto_registering_enter();
            return STATE_PROTO_REGISTERING;
        }
        
        debug_print("[PROTO] REG TX done (%lu ms), enter RX15 phase\r\n",
                    (unsigned long)elapsed);
        reg_phase = REG_PHASE_RX15;
        reg_phase_start_tick = HAL_GetTick();
        reg_rx_started = false;
        return STATE_PROTO_REGISTERING;
    }

    // Phase 1: 15s RX窗口 (收到注册完成 → IDLE; 收到代理ACK → 扩展)
    if (reg_phase == REG_PHASE_RX15) {
        // 延迟10ms后开启RX
        if (!reg_rx_started && phase_elapsed >= PROTO_REG_RX_DELAY_MS) {
            debug_print("[PROTO] REG opening 15s RX window\r\n");
            app_lora_start_rx(PROTO_REG_RX15_MS);
            reg_rx_started = true;
        }

        if (app_lora_rx_available()) {
            if (handle_reg_rx_data()) {
                app_lora_stop_rx();
                return STATE_IDLE;  // 注册成功
            }
        }

        // 15s超时 → 尝试下一个候选
        if (phase_elapsed >= PROTO_REG_RX_DELAY_MS + PROTO_REG_RX15_MS) {
            debug_print("[PROTO] REG 15s RX timeout, exclude candidate 0x%04X\r\n",
                        app_config_get()->proto_parent_addr);
            app_lora_stop_rx();
            proto_candidates_exclude(app_config_get()->proto_parent_addr);
            return STATE_PROTO_SCAN; // 回SCAN重选
        }

        return STATE_PROTO_REGISTERING;
    }

    // Phase 2: EXTENDED 3分钟RX (中继代理ACK后等待最终确认)
    if (reg_phase == REG_PHASE_EXTEND) {
        if (!reg_rx_started) {
            debug_print("[PROTO] REG extended RX started (up to %ds)\r\n",
                        PROTO_REG_RX_EXTEND_MS / 1000);
            // 先关闭旧的, 重新开启
            app_lora_stop_rx();
            app_lora_start_rx(PROTO_REG_RX_EXTEND_MS);
            reg_rx_started = true;
        }

        if (app_lora_rx_available()) {
            if (handle_reg_rx_data()) {
                app_lora_stop_rx();
                return STATE_IDLE;  // 注册成功
            }
        }

        // 3min超时 → 排除此候选, 回SCAN重选
        if (phase_elapsed >= PROTO_REG_RX_EXTEND_MS) {
            debug_print("[PROTO] REG extended RX timeout, exclude 0x%04X\r\n",
                        app_config_get()->proto_parent_addr);
            app_lora_stop_rx();
            proto_candidates_exclude(app_config_get()->proto_parent_addr);
            return STATE_PROTO_SCAN;
        }

        return STATE_PROTO_REGISTERING;
    }

    return STATE_PROTO_REGISTERING;
}

/*--- SLEEP ---*/
static void state_sleep_enter(void)
{
    uint32_t now_tick = HAL_GetTick();
    co_state_t co_st = co_get_state();

    /* 计算下一次事件的时间 */
    uint16_t meas_interval = app_config_get_measure_interval(current_mode);
    uint16_t tx_interval = app_config_get_transmit_interval(current_mode);
    uint32_t next_measure = last_measure_tick + (uint32_t)meas_interval * 1000;
    uint32_t next_transmit = last_transmit_tick + (uint32_t)tx_interval * 1000;
    uint32_t next_event = (next_measure < next_transmit) ? next_measure : next_transmit;

    if (now_tick >= next_event) return;

    uint32_t duration_s = (next_event - now_tick) / 1000;

    /* 决定是否需要提前唤醒给CO预热 */
    sleep_co_warmup = false;
    if (co_st == CO_STATE_OFF && next_measure <= next_transmit) {
        if (duration_s > CO_WARMUP_ADVANCE_S) {
            sleep_co_warmup = true;
            sleep_co_deadline_tick = next_measure;
            duration_s -= CO_WARMUP_ADVANCE_S;
        }
    }

    sleep_wakeup_tick = next_event;

    /* 关闭共享外设 */
    board_pwr_set(PWR_ADC_EN, false);

    if (duration_s > SLEEP_STOP2_THRESHOLD_S && !app_menu_is_screen_on()) {
        /* 深睡: STOP2 */
        debug_print("[SLEEP] STOP2 %lus\r\n", (unsigned long)duration_s);
        rtc_set_alarm(duration_s);
        enter_stop2();
        /* --- 从STOP2唤醒 --- */
        exit_stop2();
        if (g_rtc_wake) {
            g_rtc_wake = false;
            debug_print("[SLEEP] STOP2 wake: RTC\r\n");
        } else {
            debug_print("[SLEEP] STOP2 wake: EXTI\r\n");
            app_menu_wake_screen();  // 按键唤醒时自动亮屏
        }
        sleep_from_stop2 = true;
    } else {
        /* 浅睡: WFI, 仅关LED */
        board_led_set(LED_OFF);
        debug_print("[SLEEP] WFI %lus\r\n", (unsigned long)duration_s);
    }
}

static app_state_t state_sleep_process(void)
{
    /* 刚从STOP2醒来: HAL_GetTick未更新, 直接切IDLE让它重新同步 */
    if (sleep_from_stop2) {
        sleep_from_stop2 = false;
        return STATE_IDLE;
    }

    uint32_t now = HAL_GetTick();

    /* 按键检查 (STOP2或正常按键唤醒) */
    if (app_menu_is_settings_page()) {
        rtc_stop_alarm(); sleep_co_warmup = false; return STATE_IDLE;
    }
    if (app_menu_is_screen_on()) {
        rtc_stop_alarm(); return STATE_IDLE;
    }

    /* CO预热: RTC提前唤醒, CO尚未上电, 用WFI等剩余时间 */
    if (sleep_co_warmup) {
        sleep_co_warmup = false;
        if (co_get_state() == CO_STATE_OFF) {
            debug_print("[SLEEP] CO warmup start, WFI %lums\r\n",
                        (unsigned long)(sleep_co_deadline_tick - now));
            co_start();
            while (HAL_GetTick() < sleep_co_deadline_tick) {
                __WFI();
                if (app_menu_is_settings_page()) return STATE_IDLE;
                if (app_menu_is_screen_on()) return STATE_IDLE;
            }
            debug_print("[SLEEP] CO warmup done\r\n");
        }
        return STATE_MEASURE;
    }

    /* 到达目标时间 */
    if (now >= sleep_wakeup_tick) return STATE_IDLE;

    /* WFI浅睡, 短延时保持响应 */
    HAL_Delay(10);
    return STATE_SLEEP;
}

/*--- ERROR ---*/
static void state_error_enter(void)
{
    board_led_set(LED_ON);  // 错误时常亮
}

static app_state_t state_error_process(void)
{
    // 错误状态下刷新显示
    app_menu_display();
    
    // 检查是否进入设置
    if (app_menu_is_settings_page()) {
        return STATE_SETTINGS;
    }
    
    return STATE_ERROR;
}

/*============================================================================
 *                              按键回调
 *===========================================================================*/
static void button_event_callback(board_btn_t btn, btn_event_t event)
{
    // ISR中只入队，不做任何I2C/菜单操作
    uint8_t next_head = (btn_queue_head + 1) % BTN_QUEUE_SIZE;
    if (next_head != btn_queue_tail) {  // 队列未满
        btn_queue[btn_queue_head].btn = btn;
        btn_queue[btn_queue_head].event = event;
        btn_queue_head = next_head;
    }
}

/* 主循环中处理按键事件 */
static void process_button_events(void)
{
    while (btn_queue_tail != btn_queue_head) {
        board_btn_t btn = btn_queue[btn_queue_tail].btn;
        btn_event_t event = btn_queue[btn_queue_tail].event;
        btn_queue_tail = (btn_queue_tail + 1) % BTN_QUEUE_SIZE;

        // 交给菜单处理
        app_menu_handle_button(btn, event);
    }
}

/*============================================================================
 *                              公开接口
 *===========================================================================*/
void app_init(void)
{
    // 进入INIT状态（内部会调用btn_driver_init，会清零回调）
    state_init_enter();

    // 注册按键回调（必须在btn_driver_init之后）
    btn_register_callback(button_event_callback);

    // 调试模式: 每次上电清除短地址, 强制重新注册
    if (app_config_has_short_addr()) {
        debug_print("[DBG] Clearing persisted short addr for debug mode\r\n");
        app_config_clear_short_addr();
    }

    // 先GNSS定位, 再进入协议SCAN扫描注册
    if (!gnss_location_saved && !registration_done) {
        debug_print("[DBG] Boot: GNSS first, then SCAN\r\n");
        current_state = STATE_GNSS;
        state_gnss_enter();
        return;
    }
    
    // 已有位置, 直接进入SCAN
    debug_print("[DBG] Boot: direct to SCAN\r\n");
    current_state = STATE_PROTO_SCAN;
    state_proto_scan_enter();
}

void app_process(void)
{
    app_state_t next_state = current_state;
    
    // 处理LED闪烁
    board_led_process();

    // 处理CO传感器数据
    co_process();

    // 处理按键事件（从ISR队列中取出，在主循环上下文处理）
    process_button_events();
    
    // 状态机处理
    switch (current_state) {
        case STATE_IDLE:
            next_state = state_idle_process();
            break;
            
        case STATE_SETTINGS:
            next_state = state_settings_process();
            break;
            
        case STATE_MEASURE:
            next_state = state_measure_process();
            break;
            
        case STATE_GNSS:
            next_state = state_gnss_process();
            break;
            
        case STATE_TRANSMIT:
            next_state = state_transmit_process();
            break;
            
        case STATE_RECEIVE:
            next_state = state_receive_process();
            break;
            
        case STATE_SLEEP:
            next_state = state_sleep_process();
            break;
            
        case STATE_ERROR:
            next_state = state_error_process();
            break;
            
        case STATE_PROTO_SCAN:
            next_state = state_proto_scan_process();
            break;
            
        case STATE_PROTO_REGISTERING:
            next_state = state_proto_registering_process();
            break;
            
        case STATE_INIT:
            // INIT在app_init中完成
            break;
    }
    
    // 状态切换
    if (next_state != current_state) {
        static const char * const state_names[] = {
            "INIT", "IDLE", "SETTINGS", "MEASURE", "GNSS",
            "TRANSMIT", "RECEIVE", "SLEEP", "ERROR",
            "SCAN", "REGISTER"
        };
        debug_print("[STATE] %s -> %s\r\n",
                    state_names[current_state], state_names[next_state]);
        current_state = next_state;
        
        switch (current_state) {
            case STATE_IDLE:      state_idle_enter();      break;
            case STATE_SETTINGS:  state_settings_enter();  break;
            case STATE_MEASURE:   state_measure_enter();   break;
            case STATE_GNSS:      state_gnss_enter();      break;
            case STATE_TRANSMIT:  state_transmit_enter();  break;
            case STATE_RECEIVE:   state_receive_enter();   break;
            case STATE_SLEEP:     state_sleep_enter();     break;
            case STATE_ERROR:     state_error_enter();     break;
            case STATE_PROTO_SCAN:        state_proto_scan_enter();        break;
            case STATE_PROTO_REGISTERING: state_proto_registering_enter(); break;
            default: break;
        }
    }
}

app_state_t app_get_state(void)
{
    return current_state;
}

run_mode_t app_get_run_mode(void)
{
    return current_mode;
}

uint16_t app_get_errors(void)
{
    return error_flags;
}

const sensor_data_t *app_get_sensor_data(void)
{
    return &sensor_data;
}
