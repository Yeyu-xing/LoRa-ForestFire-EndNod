/**
  ******************************************************************************
  * @file    board_io.c
  * @brief   板级IO抽象层 - 硬件操作封装实现
  ******************************************************************************
  */

#include "board_io.h"
#include "main.h"
#include "adc.h"

/*============================================================================
 *                              私有变量
 *===========================================================================*/
// ADC DMA缓冲区
static uint16_t adc_dma_buffer[2] = {0};  // [0]=VREFINT, [1]=Battery

// 当前射频模式
static board_rf_mode_t current_rf_mode = RF_MODE_IDLE;

// LED控制
static board_led_mode_t led_mode = LED_OFF;
static uint32_t led_last_toggle = 0;
static bool led_state = false;

/*============================================================================
 *                              私有常量 - 引脚映射表
 *===========================================================================*/
// 按键引脚映射
static const struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} btn_pins[BTN_COUNT] = {
    {BTN_UP_GPIO_Port,   BTN_UP_Pin},    // PB12
    {BTN_OK_GPIO_Port,   BTN_OK_Pin},    // PB2
    {BTN_DOWN_GPIO_Port, BTN_DOWN_Pin},  // PA15
};

// 电源控制引脚映射
static const struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} pwr_pins[PWR_COUNT] = {
    {ADC_EN_GPIO_Port,  ADC_EN_Pin},       // PB5 - 电池电压检测使能
    {VCC_5V_EN_GPIO_Port, VCC_5V_EN_Pin},  // PB3 - 5V电源使能
    {GNSSEN_GPIO_Port,  GNSSEN_Pin},        // PA8 - GNSS模块使能
};

/*============================================================================
 *                              初始化
 *===========================================================================*/
void board_io_init(void)
{
    // GPIO已在MX_GPIO_Init()中初始化
    // ADC已在MX_ADC_Init()中初始化
    
    // 设置初始状态
    // 关闭所有电源控制
    for (int i = 0; i < PWR_COUNT; i++) {
        board_pwr_set(i, false);
    }
    
    // 设置射频开关为空闲模式
    board_rf_set_mode(RF_MODE_IDLE);
}

/*============================================================================
 *                              按键操作
 *===========================================================================*/
bool board_btn_read(board_btn_t btn)
{
    if (btn >= BTN_COUNT) return false;
    
    // 按键低电平有效
    return (HAL_GPIO_ReadPin(btn_pins[btn].port, btn_pins[btn].pin) == GPIO_PIN_RESET);
}

/*============================================================================
 *                              电源控制
 *===========================================================================*/
void board_pwr_set(board_pwr_t pwr, bool on)
{
    if (pwr >= PWR_COUNT) return;
    
    // GNSS使能引脚极性处理
    bool pin_active = on;
    if (pwr == PWR_GNSS_EN && BOARD_GNSS_EN_ACTIVE_LOW) {
        pin_active = !on;   // 低电平有效时取反
    }
    
    HAL_GPIO_WritePin(pwr_pins[pwr].port, pwr_pins[pwr].pin,
                      pin_active ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool board_pwr_get(board_pwr_t pwr)
{
    if (pwr >= PWR_COUNT) return false;
    
    bool pin_set = (HAL_GPIO_ReadPin(pwr_pins[pwr].port, pwr_pins[pwr].pin) == GPIO_PIN_SET);
    
    // GNSS使能引脚极性处理
    if (pwr == PWR_GNSS_EN && BOARD_GNSS_EN_ACTIVE_LOW) {
        return !pin_set;    // 低电平有效时取反
    }
    
    return pin_set;
}

/*============================================================================
 *                              ADC采集 - 阻塞模式
 *===========================================================================*/
float board_adc_read_battery(void)
{
    // 使能ADC检测
    board_pwr_set(PWR_ADC_EN, true);
    
    // 等待电压稳定
    HAL_Delay(BOARD_ADC_SETTLE_DELAY_MS);
    
    // 启动ADC
    HAL_ADC_Start(&hadc);
    
    // 第1路: 读取VREFINT (Rank 1)
    uint16_t vref_raw = 0;
    if (HAL_ADC_PollForConversion(&hadc, BOARD_ADC_TIMEOUT_MS) == HAL_OK) {
        vref_raw = HAL_ADC_GetValue(&hadc);
    }
    
    // 第2路: 读取电池电压 (Rank 2)
    uint16_t bat_raw = 0;
    if (HAL_ADC_PollForConversion(&hadc, BOARD_ADC_TIMEOUT_MS) == HAL_OK) {
        bat_raw = HAL_ADC_GetValue(&hadc);
    }
    
    HAL_ADC_Stop(&hadc);
    
    // 使用校准值计算实际VDDA，再计算电池电压
    // VDDA_actual = 3.3V * VREFINT_CAL / VREFINT_DATA
    // V_bat = (bat_raw / 4096) * VDDA_actual * 分压比
    uint16_t vrefint_cal = board_get_vrefint_cal();
    float vdda;
    if (vref_raw > 0 && vrefint_cal > 0) {
        vdda = BOARD_VDDA_VOLTAGE * (float)vrefint_cal / (float)vref_raw;
    } else {
        vdda = BOARD_VDDA_VOLTAGE;  // 后备：使用标称值
    }
    
    float voltage = ((float)bat_raw / BOARD_ADC_NUM_CODES) * vdda * BOARD_BAT_DIVIDER * BOARD_BAT_CORRECTION;
    
    return voltage;
}

uint16_t board_adc_read_vrefint(void)
{
    HAL_ADC_Start(&hadc);
    
    if (HAL_ADC_PollForConversion(&hadc, BOARD_ADC_TIMEOUT_MS) == HAL_OK) {
        uint16_t vref_raw = HAL_ADC_GetValue(&hadc);
        return vref_raw;
    }
    
    HAL_ADC_Stop(&hadc);
    return 0;
}

/*============================================================================
 *                              ADC采集 - DMA模式
 *===========================================================================*/
void board_adc_start(void)
{
    // 使能ADC检测
    board_pwr_set(PWR_ADC_EN, true);
    
    // 启动ADC DMA
    HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc_dma_buffer, 2);
}

void board_adc_stop(void)
{
    HAL_ADC_Stop_DMA(&hadc);
    board_pwr_set(PWR_ADC_EN, false);
}

float board_adc_get_battery(void)
{
    // 电池电压在第二个通道 (ADC_CHANNEL_3, Rank 2)
    uint16_t bat_raw = adc_dma_buffer[1];
    
    // 计算电池电压: V = ADC值 / NUM_CODES * VDDA * 分压比
    float voltage = (bat_raw / BOARD_ADC_NUM_CODES) * BOARD_VDDA_VOLTAGE * BOARD_BAT_DIVIDER;
    
    return voltage;
}

uint16_t board_adc_get_vrefint(void)
{
    // 内部参考电压在第一个通道 (VREFINT, Rank 1)
    return adc_dma_buffer[0];
}

/*============================================================================
 *                              射频开关控制
 *===========================================================================*/
/**
  * @brief  射频开关真值表
  * | 模式 | TXEN(PA6) | RXEN(PA7) |
  * |------|-----------|-----------|
  * | IDLE |    0      |    0      |
  * | TX   |    1      |    0      |
  * | RX   |    0      |    1      |
  *
  * 注：TXEN和RXEN均为高电平有效
  */
void board_rf_set_mode(board_rf_mode_t mode)
{
    switch (mode) {
        case RF_MODE_IDLE:
            // TX=OFF(PA6=0), RX=OFF(PA7=0)
            HAL_GPIO_WritePin(RF_TXEN_GPIO_Port, RF_TXEN_Pin, GPIO_PIN_RESET);    // TX disable
            HAL_GPIO_WritePin(RF_RXEN_GPIO_Port, RF_RXEN_Pin, GPIO_PIN_RESET);    // RX disable
            break;

        case RF_MODE_TX:
            // TX=ON(PA6=1), RX=OFF(PA7=0)
            HAL_GPIO_WritePin(RF_TXEN_GPIO_Port, RF_TXEN_Pin, GPIO_PIN_SET);      // TX enable
            HAL_GPIO_WritePin(RF_RXEN_GPIO_Port, RF_RXEN_Pin, GPIO_PIN_RESET);    // RX disable
            break;

        case RF_MODE_RX:
            // TX=OFF(PA6=0), RX=ON(PA7=1)
            HAL_GPIO_WritePin(RF_TXEN_GPIO_Port, RF_TXEN_Pin, GPIO_PIN_RESET);    // TX disable
            HAL_GPIO_WritePin(RF_RXEN_GPIO_Port, RF_RXEN_Pin, GPIO_PIN_SET);      // RX enable
            break;
    }

    current_rf_mode = mode;
}

board_rf_mode_t board_rf_get_mode(void)
{
    return current_rf_mode;
}

/*============================================================================
 *                              系统信息
 *===========================================================================*/
void board_get_uid(uint8_t *uid)
{
    // 使用HAL库函数获取UID (96位 = 12字节)
    uint32_t uid_words[3];
    uid_words[0] = HAL_GetUIDw0();
    uid_words[1] = HAL_GetUIDw1();
    uid_words[2] = HAL_GetUIDw2();
    
    // 转换为字节数组
    for (int i = 0; i < 3; i++) {
        uid[i*4 + 0] = (uid_words[i] >> 0)  & 0xFF;
        uid[i*4 + 1] = (uid_words[i] >> 8)  & 0xFF;
        uid[i*4 + 2] = (uid_words[i] >> 16) & 0xFF;
        uid[i*4 + 3] = (uid_words[i] >> 24) & 0xFF;
    }
}

uint16_t board_get_vrefint_cal(void)
{
    // 从系统存储区读取VREFINT校准值
    // 校准条件: VDDA = 3.3V, TA = 30°C
    return *BOARD_VREFINT_CAL_ADDR;
}

float board_get_mcu_temp(void)
{
    // TODO: 读取内部温度传感器
    // STM32WL有内部温度传感器，需要额外配置ADC通道
    return 0.0f;
}

/*============================================================================
 *                              LED指示灯控制
 *===========================================================================*/
void board_led_set(board_led_mode_t mode)
{
    led_mode = mode;

    switch (mode) {
        case LED_OFF:
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
            led_state = false;
            break;
        case LED_ON:
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            led_state = true;
            break;
        case LED_BLINK_SLOW:
        case LED_BLINK_FAST:
            led_last_toggle = HAL_GetTick();
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            led_state = true;
            break;
    }
}

void board_led_process(void)
{
    if (led_mode != LED_BLINK_SLOW && led_mode != LED_BLINK_FAST) {
        return;
    }

    uint32_t interval = (led_mode == LED_BLINK_SLOW) ? 500 : 125;
    uint32_t now = HAL_GetTick();

    if ((now - led_last_toggle) >= interval) {
        led_state = !led_state;
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        led_last_toggle = now;
    }
}
