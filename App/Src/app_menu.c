/**
  ******************************************************************************
  * @file    app_menu.c
  * @brief   应用菜单模块 - 直接使用OLED驱动实现完整菜单
  * @note    绕过Easy_Menu框架，每帧完整重绘，支持中文和数值编辑
  *          OLED: 128x64, 4行x16像素, font16x16(中文16x16 + ASCII 16x8)
 *          已有中文字库: 设置间隔常规预警火阈值温度报警恢复湿备信息
 *                       默认返回主页页低功率扩频因子编辑修改保存确认取消℃
 *                       重新定位经未除清部署电条数缓压池纬状态中
 *          缺少字模:
  ******************************************************************************
  */

#include "app_menu.h"

#include <stdio.h>
#include <string.h>

#include "stm32wlxx_hal.h"
#include "oled.h"
#include "font.h"
#include "app_config.h"
#include "lora_protocol.h"
#include "driver_gnss.h"

/*============================================================================
 *                              菜单页面枚举
 *===========================================================================*/
typedef enum {
    PAGE_HOME = 0,
    PAGE_SETTINGS,      /* 设置 */
    PAGE_MEAS_INTERVAL, /* 采集间隔设置 */
    PAGE_TX_INTERVAL,   /* 发送间隔设置 */
    PAGE_THRESHOLD,     /* 阈值设置 */
    PAGE_LORA,          /* LoRa设置 */
    PAGE_DEVICE,        /* 设备信息 */
    PAGE_REDEPLOY,      /* 重新定位确认 */
    PAGE_COUNT
} menu_page_t;

/* 编辑页类型 (哪个参数正在编辑) */
typedef enum {
    EDIT_NONE = 0,
    EDIT_MEAS_INTERVAL_NORMAL,   /* 采集-常规 */
    EDIT_MEAS_INTERVAL_WARNING,  /* 采集-预警 */
    EDIT_MEAS_INTERVAL_ALARM,    /* 采集-火警 */
    EDIT_TX_INTERVAL_NORMAL,     /* 发送-常规 */
    EDIT_TX_INTERVAL_WARNING,    /* 发送-预警 */
    EDIT_TX_INTERVAL_ALARM,      /* 发送-火警 */
    EDIT_TEMP_WARNING,           /* 温度预警阈值 */
    EDIT_TEMP_WARNING_RECOVERY,  /* 温度预警恢复 */
    EDIT_TEMP_ALARM,             /* 温度报警阈值 */
    EDIT_TEMP_RECOVERY,          /* 温度恢复阈值 */
    EDIT_CO_WARNING,             /* CO预警阈值 */
    EDIT_CO_RECOVERY,            /* CO恢复阈值 */
    EDIT_HUM_WARNING,            /* 湿度预警阈值 */
    EDIT_HUM_RECOVERY,           /* 湿度恢复阈值 */
    EDIT_LORA_POWER_LEVEL,       /* LoRa功率档位 */
    EDIT_LORA_SF_PRI,            /* LoRa主SF */
    EDIT_LORA_SF_BAK,            /* LoRa备用SF */
    EDIT_TYPE_COUNT
} edit_type_t;

/*============================================================================
 *                              主页条目定义
 *===========================================================================*/
#define HOME_ITEM_COUNT  11   /* 模式温度/湿度/CO/电池/短地址/父节点/缓存/间隔/定位/纬度/经度 */

/*============================================================================
 *                              私有变量
 *===========================================================================*/
static bool in_settings = false;
static bool screen_on = false;       /* 屏幕亮起状态 */
static menu_page_t current_page = PAGE_HOME;
static uint8_t cursor = 0;
static edit_type_t edit_type = EDIT_NONE;
static uint8_t home_scroll = 0;  /* 主页滚动偏移 */
static bool redeploy_requested = false;  /* 重新定位请求标志 */
static uint32_t screen_off_tick = 0;      /* 息屏时刻, 用于防抖 */
static uint32_t edit_hold_tick = 0;       /* 编辑时长按开始时刻 */
static bool     edit_hold_active = false; /* 正在长按加速 */

/* 各页面条目数 (含返回项) */
static const uint8_t page_item_count[PAGE_COUNT] = {
    0,  /* PAGE_HOME (不使用cursor机制，使用home_scroll) */
    8,  /* PAGE_SETTINGS: 采集间隔, 发送间隔, 阈值, LoRa, 设备, 重新定位, 恢复默认, 返回 */
    4,  /* PAGE_MEAS_INTERVAL: 常规, 预警, 火警, 返回 */
    4,  /* PAGE_TX_INTERVAL: 常规, 预警, 火警, 返回 */
    9,  /* PAGE_THRESHOLD: 温度预警, 温度预警恢复, 温度报警, 温度恢复, CO预警, CO恢复, 湿度预警, 湿度恢复, 返回 */
    4,  /* PAGE_LORA: 功率档位, 主SF, 备用SF, 返回 */
    1,  /* PAGE_DEVICE: 返回 (前三行是信息) */
    2   /* PAGE_REDEPLOY: 确认, 取消 */
};

/* 编辑用的配置副本 */
static app_config_t edit_config;

/*============================================================================
 *                              私有函数 - 配置同步
 *===========================================================================*/
static void load_edit_config(void)
{
    const app_config_t *cfg = app_config_get();
    edit_config = *cfg;
}

static void save_edit_config(void)
{
    app_config_set(&edit_config);
}

/*============================================================================
 *                              私有函数 - OLED绘制辅助
 *===========================================================================*/
/**
  * @brief  在指定位置绘制混合字符串（中文16x16 + ASCII 16x8 统一高度）
  * @param  x: X起始像素
  * @param  line: 行号 0-3
  * @param  str: 字符串（可含中文UTF-8）
  * @param  reverse: 是否反色
  */
static void draw_str(uint8_t x, uint8_t line, const char *str, bool reverse)
{
    OLED_ColorMode color = reverse ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL;
    OLED_PrintString(x, line * 16, (char *)str, &font16x16, color);
}

/**
  * @brief  绘制一行菜单项（带光标反色高亮）
  * @param  line: 行号 0-3
  * @param  text: 菜单文本（可含中文，统一16px高）
  * @param  selected: 是否被光标选中
  */
static void draw_menu_line(uint8_t line, const char *text, bool selected)
{
    if (selected) {
        OLED_DrawFilledRectangle(0, line * 16, 128, 16, OLED_COLOR_REVERSED);
    }
    draw_str(8, line, text, selected);
}

/**
  * @brief  绘制纯ASCII行 (16x8字体)
  */
static void draw_ascii_line(uint8_t line, const char *text, bool selected)
{
    OLED_ColorMode color = selected ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL;
    if (selected) {
        OLED_DrawFilledRectangle(0, line * 16, 128, 16, OLED_COLOR_REVERSED);
    }
    OLED_PrintASCIIString(4, line * 16, (char *)text, &afont16x8, color);
}

/*============================================================================
 *                              私有函数 - 通用滚动计算
 *===========================================================================*/
static uint8_t calc_scroll_start(uint8_t cursor_val, uint8_t total, uint8_t visible)
{
    uint8_t start = 0;
    if (cursor_val >= visible - 1) {
        start = cursor_val - (visible - 2);
    }
    if (start + visible > total) {
        start = total - visible;
    }
    return start;
}

/*============================================================================
 *                              私有函数 - 数值编辑
 *===========================================================================*/
static void adjust_uint16(uint16_t *val, int16_t delta, uint16_t min, uint16_t max)
{
    int32_t v = (int32_t)(*val) + delta;
    if (v < (int32_t)min) v = (int32_t)min;
    if (v > (int32_t)max) v = (int32_t)max;
    *val = (uint16_t)v;
}

static void adjust_uint8(uint8_t *val, int8_t delta, uint8_t min, uint8_t max)
{
    int16_t v = (int16_t)(*val) + delta;
    if (v < (int16_t)min) v = (int16_t)min;
    if (v > (int16_t)max) v = (int16_t)max;
    *val = (uint8_t)v;
}

static void adjust_float(float *val, float delta, float min, float max)
{
    float v = *val + delta;
    if (v < min) v = min;
    if (v > max) v = max;
    *val = v;
}

/*============================================================================
 *                              私有函数 - 格式化辅助
 *===========================================================================*/
/** 格式化float: 无效值显示 N/A (避免使用%f, newlib-nano不支持) */
static void fmt_temp(char *buf, uint8_t len, float val, bool valid)
{
    if (valid) {
        int ival = (int)(val * 10);
        if (ival < 0) {
            snprintf(buf, len, "-%d.%d℃", -ival / 10, -ival % 10);
        } else {
            snprintf(buf, len, "%d.%d℃", ival / 10, ival % 10);
        }
    } else {
        snprintf(buf, len, "N/A");
    }
}

static void fmt_hum(char *buf, uint8_t len, float val, bool valid)
{
    if (valid) {
        int ival = (int)(val + 0.5f);
        if (ival < 0) ival = 0;
        if (ival > 100) ival = 100;
        snprintf(buf, len, "%d%%", ival);
    } else {
        snprintf(buf, len, "N/A");
    }
}

static void fmt_co(char *buf, uint8_t len, float val, bool valid)
{
    if (valid) {
        int ival = (int)(val + 0.5f);
        if (ival < 0) ival = 0;
        snprintf(buf, len, "%d", ival);
    } else {
        snprintf(buf, len, "N/A");
    }
}

/*============================================================================
 *                              私有函数 - 编辑页面
 *===========================================================================*/
static const char *edit_type_name(edit_type_t t)
{
    switch (t) {
        case EDIT_MEAS_INTERVAL_NORMAL:  return "采集-常规";
        case EDIT_MEAS_INTERVAL_WARNING: return "采集-预警";
        case EDIT_MEAS_INTERVAL_ALARM:   return "采集-火警";
        case EDIT_TX_INTERVAL_NORMAL:    return "发送-常规";
        case EDIT_TX_INTERVAL_WARNING:   return "发送-预警";
        case EDIT_TX_INTERVAL_ALARM:     return "发送-火警";
        case EDIT_TEMP_WARNING:          return "温度预警";
        case EDIT_TEMP_WARNING_RECOVERY: return "温度预恢";
        case EDIT_TEMP_ALARM:            return "温度报警";
        case EDIT_TEMP_RECOVERY:         return "温度恢复";
        case EDIT_CO_WARNING:            return "CO预警";
        case EDIT_CO_RECOVERY:           return "CO恢复";
        case EDIT_HUM_WARNING:           return "湿度预警";
        case EDIT_HUM_RECOVERY:          return "湿度恢复";
        case EDIT_LORA_POWER_LEVEL:      return "功率档位";
        case EDIT_LORA_SF_PRI:           return "主扩频因子";
        case EDIT_LORA_SF_BAK:           return "备用扩频因子";
        default: return "";
    }
}

static void edit_type_value_str(edit_type_t t, char *buf, uint8_t len)
{
    switch (t) {
        case EDIT_MEAS_INTERVAL_NORMAL:
            snprintf(buf, len, "%us", edit_config.measure_interval_normal);
            break;
        case EDIT_MEAS_INTERVAL_WARNING:
            snprintf(buf, len, "%us", edit_config.measure_interval_warning);
            break;
        case EDIT_MEAS_INTERVAL_ALARM:
            snprintf(buf, len, "%us", edit_config.measure_interval_alarm);
            break;
        case EDIT_TX_INTERVAL_NORMAL:
            snprintf(buf, len, "%us", edit_config.transmit_interval_normal);
            break;
        case EDIT_TX_INTERVAL_WARNING:
            snprintf(buf, len, "%us", edit_config.transmit_interval_warning);
            break;
        case EDIT_TX_INTERVAL_ALARM:
            snprintf(buf, len, "%us", edit_config.transmit_interval_alarm);
            break;
        case EDIT_TEMP_WARNING: {
            int v = (int)(edit_config.temp_warning_threshold * 10);
            snprintf(buf, len, "%d.%d℃", v / 10, v % 10);
            break;
        }
        case EDIT_TEMP_WARNING_RECOVERY: {
            int v = (int)(edit_config.temp_warning_recovery * 10);
            snprintf(buf, len, "%d.%d℃", v / 10, v % 10);
            break;
        }
        case EDIT_TEMP_ALARM: {
            int v = (int)(edit_config.temp_alarm_threshold * 10);
            snprintf(buf, len, "%d.%d℃", v / 10, v % 10);
            break;
        }
        case EDIT_TEMP_RECOVERY: {
            int v = (int)(edit_config.temp_alarm_recovery * 10);
            snprintf(buf, len, "%d.%d℃", v / 10, v % 10);
            break;
        }
        case EDIT_CO_WARNING: {
            int v = (int)(edit_config.co_warning_threshold * 10);
            snprintf(buf, len, "%d.%d", v / 10, v % 10);
            break;
        }
        case EDIT_CO_RECOVERY: {
            int v = (int)(edit_config.co_warning_recovery * 10);
            snprintf(buf, len, "%d.%d", v / 10, v % 10);
            break;
        }
        case EDIT_HUM_WARNING: {
            int v = (int)(edit_config.hum_warning_threshold + 0.5f);
            snprintf(buf, len, "%d%%", v);
            break;
        }
        case EDIT_HUM_RECOVERY: {
            int v = (int)(edit_config.hum_warning_recovery + 0.5f);
            snprintf(buf, len, "%d%%", v);
            break;
        }
        case EDIT_LORA_POWER_LEVEL: {
            int dbm = proto_power_level_to_dbm(edit_config.proto_power_level);
            snprintf(buf, len, "档%u(%ddBm)", edit_config.proto_power_level, dbm);
            break;
        }
        case EDIT_LORA_SF_PRI:
            snprintf(buf, len, "SF%u", edit_config.proto_sf_primary);
            break;
        case EDIT_LORA_SF_BAK:
            snprintf(buf, len, "SF%u", edit_config.proto_sf_backup);
            break;
        default:
            buf[0] = '\0';
            break;
    }
}

/**
  * @brief  绘制编辑页面 (独立页面，一次只编辑一个参数)
  *         第0行: 参数名称
  *         第1行: 当前值 (反色高亮)
  *         第2行: UP/DOWN修改提示
  *         第3行: OK确认保存
  */
static void draw_edit_page(void)
{
    char buf[22];

    /* 第0行: 参数名称 */
    draw_str(4, 0, edit_type_name(edit_type), false);

    /* 第1行: 当前值 - 反色高亮显示 */
    edit_type_value_str(edit_type, buf, sizeof(buf));
    OLED_DrawFilledRectangle(16, 16, 96, 16, OLED_COLOR_REVERSED);
    OLED_PrintString(32, 16, buf, &font16x16, OLED_COLOR_REVERSED);

    /* 第2行: UP/DOWN修改提示 */
    draw_str(8, 2, "修改", false);

    /* 第3行: OK确认保存 */
    draw_str(8, 3, "OK确认保存", false);
}

/**
  * @brief  处理编辑页面的按键
  */
static bool edit_handle_button(board_btn_t btn, btn_event_t event)
{
    if (event == BTN_EVENT_SHORT_CLICK) {
        switch (btn) {
            case BTN_UP: {
                switch (edit_type) {
                    case EDIT_MEAS_INTERVAL_NORMAL:
                        adjust_uint16(&edit_config.measure_interval_normal, 1, 10, 3600); break;
                    case EDIT_MEAS_INTERVAL_WARNING:
                        adjust_uint16(&edit_config.measure_interval_warning, 1, 10, 3600); break;
                    case EDIT_MEAS_INTERVAL_ALARM:
                        adjust_uint16(&edit_config.measure_interval_alarm, 1, 5, 300); break;
                    case EDIT_TX_INTERVAL_NORMAL:
                        adjust_uint16(&edit_config.transmit_interval_normal, 1, 10, 3600); break;
                    case EDIT_TX_INTERVAL_WARNING:
                        adjust_uint16(&edit_config.transmit_interval_warning, 1, 10, 3600); break;
                    case EDIT_TX_INTERVAL_ALARM:
                        adjust_uint16(&edit_config.transmit_interval_alarm, 1, 5, 300); break;
                    case EDIT_TEMP_WARNING:
                        adjust_float(&edit_config.temp_warning_threshold, 1.0f, 20.0f, 80.0f); break;
                    case EDIT_TEMP_WARNING_RECOVERY:
                        adjust_float(&edit_config.temp_warning_recovery, 1.0f, 15.0f, 70.0f); break;
                    case EDIT_TEMP_ALARM:
                        adjust_float(&edit_config.temp_alarm_threshold, 1.0f, 30.0f, 100.0f); break;
                    case EDIT_TEMP_RECOVERY:
                        adjust_float(&edit_config.temp_alarm_recovery, 1.0f, 20.0f, 90.0f); break;
                    case EDIT_CO_WARNING:
                        adjust_float(&edit_config.co_warning_threshold, 1.0f, 5.0f, 200.0f); break;
                    case EDIT_CO_RECOVERY:
                        adjust_float(&edit_config.co_warning_recovery, 1.0f, 5.0f, 150.0f); break;
                    case EDIT_HUM_WARNING:
                        adjust_float(&edit_config.hum_warning_threshold, 1.0f, 1.0f, 50.0f); break;
                    case EDIT_HUM_RECOVERY:
                        adjust_float(&edit_config.hum_warning_recovery, 1.0f, 1.0f, 60.0f); break;
                    case EDIT_LORA_POWER_LEVEL:
                        adjust_uint8(&edit_config.proto_power_level, 1, 0, PROTO_POWER_LEVEL_MAX); break;
                    case EDIT_LORA_SF_PRI:
                        adjust_uint8(&edit_config.proto_sf_primary, 1, 7, 12); break;
                    case EDIT_LORA_SF_BAK:
                        adjust_uint8(&edit_config.proto_sf_backup, 1, 7, 12); break;
                    default: break;
                }
                return true;
            }
            case BTN_DOWN: {
                switch (edit_type) {
                    case EDIT_MEAS_INTERVAL_NORMAL:
                        adjust_uint16(&edit_config.measure_interval_normal, -1, 10, 3600); break;
                    case EDIT_MEAS_INTERVAL_WARNING:
                        adjust_uint16(&edit_config.measure_interval_warning, -1, 10, 3600); break;
                    case EDIT_MEAS_INTERVAL_ALARM:
                        adjust_uint16(&edit_config.measure_interval_alarm, -1, 5, 300); break;
                    case EDIT_TX_INTERVAL_NORMAL:
                        adjust_uint16(&edit_config.transmit_interval_normal, -1, 10, 3600); break;
                    case EDIT_TX_INTERVAL_WARNING:
                        adjust_uint16(&edit_config.transmit_interval_warning, -1, 10, 3600); break;
                    case EDIT_TX_INTERVAL_ALARM:
                        adjust_uint16(&edit_config.transmit_interval_alarm, -1, 5, 300); break;
                    case EDIT_TEMP_WARNING:
                        adjust_float(&edit_config.temp_warning_threshold, -1.0f, 20.0f, 80.0f); break;
                    case EDIT_TEMP_WARNING_RECOVERY:
                        adjust_float(&edit_config.temp_warning_recovery, -1.0f, 15.0f, 70.0f); break;
                    case EDIT_TEMP_ALARM:
                        adjust_float(&edit_config.temp_alarm_threshold, -1.0f, 30.0f, 100.0f); break;
                    case EDIT_TEMP_RECOVERY:
                        adjust_float(&edit_config.temp_alarm_recovery, -1.0f, 20.0f, 90.0f); break;
                    case EDIT_CO_WARNING:
                        adjust_float(&edit_config.co_warning_threshold, -1.0f, 5.0f, 200.0f); break;
                    case EDIT_CO_RECOVERY:
                        adjust_float(&edit_config.co_warning_recovery, -1.0f, 5.0f, 150.0f); break;
                    case EDIT_HUM_WARNING:
                        adjust_float(&edit_config.hum_warning_threshold, -1.0f, 1.0f, 50.0f); break;
                    case EDIT_HUM_RECOVERY:
                        adjust_float(&edit_config.hum_warning_recovery, -1.0f, 1.0f, 60.0f); break;
                    case EDIT_LORA_POWER_LEVEL:
                        adjust_uint8(&edit_config.proto_power_level, -1, 0, PROTO_POWER_LEVEL_MAX); break;
                    case EDIT_LORA_SF_PRI:
                        adjust_uint8(&edit_config.proto_sf_primary, -1, 7, 12); break;
                    case EDIT_LORA_SF_BAK:
                        adjust_uint8(&edit_config.proto_sf_backup, -1, 7, 12); break;
                    default: break;
                }
                return true;
            }
            case BTN_OK:
                /* OK = 保存并退出编辑 */
                save_edit_config();
                edit_type = EDIT_NONE;
                return true;
            default:
                break;
        }
    }

    /* 长按UP/DOWN = 开启加速调节 */
    if ((btn == BTN_UP || btn == BTN_DOWN) && event == BTN_EVENT_LONG_PRESS) {
        edit_hold_active = true;
        edit_hold_tick = HAL_GetTick();
        return true;
    }
    /* 释放时停止加速 */
    if ((btn == BTN_UP || btn == BTN_DOWN) && event == BTN_EVENT_RELEASE) {
        edit_hold_active = false;
        return true;
    }

    /* 长按OK = 取消编辑 (不保存) */
    if (btn == BTN_OK && event == BTN_EVENT_LONG_PRESS) {
        load_edit_config();
        edit_type = EDIT_NONE;
        return true;
    }

    return false;
}

/*============================================================================
 *                              私有函数 - 各页面绘制
 *===========================================================================*/
static void draw_home_page(void)
{
    const sensor_data_t *data = app_get_sensor_data();
    run_mode_t mode = app_get_run_mode();
    const app_config_t *cfg = app_config_get();

    char buf[22];

    /* 主页9条数据, 4行可见, 自动滚动 */
    const uint8_t total = HOME_ITEM_COUNT;
    const uint8_t visible = 4;
    uint8_t start = home_scroll;
    if (start + visible > total) {
        start = total - visible;
    }

    for (uint8_t i = 0; i < visible; i++) {
        uint8_t item = start + i;
        if (item >= total) break;

        switch (item) {
            case 0: /* 温度 + 运行模式 */
                switch (mode) {
                    case RUN_MODE_FIRE_ALARM:    draw_str(0, i, "火", false); break;
                    case RUN_MODE_EARLY_WARNING: draw_str(0, i, "预", false); break;
                    default:                     draw_str(0, i, "常", false); break;
                }
                fmt_temp(buf, sizeof(buf), data->temperature, data->sht40_valid);
                draw_str(16, i, buf, false);
                break;

            case 1: /* 湿度 */
                draw_str(0, i, "湿度", false);
                fmt_hum(buf, sizeof(buf), data->humidity, data->sht40_valid);
                draw_str(32, i, buf, false);
                break;

            case 2: /* CO浓度 */
                draw_str(0, i, "CO:", false);
                fmt_co(buf, sizeof(buf), data->co_concentration, data->co_valid);
                draw_str(24, i, buf, false);
                break;

            case 3: /* 电池电压 */
                draw_str(0, i, "电池电压", false);
                {
                    int bv = (int)(data->battery_voltage * 100);
                    snprintf(buf, sizeof(buf), "%d.%02dV", bv / 100, bv % 100);
                }
                draw_str(80, i, buf, false);
                break;

            case 4: /* 短地址 */
                draw_str(0, i, "短地址:", false);
                if (cfg->proto_has_short_addr) {
                    snprintf(buf, sizeof(buf), "0x%04X", cfg->proto_short_addr);
                } else {
                    snprintf(buf, sizeof(buf), "未注册");
                }
                draw_str(56, i, buf, false);
                break;

            case 5: /* 父节点 */
                draw_str(0, i, "父节点:", false);
                if (cfg->proto_parent_addr != PROTO_ADDR_BROADCAST) {
                    snprintf(buf, sizeof(buf), "0x%04X", cfg->proto_parent_addr);
                } else {
                    snprintf(buf, sizeof(buf), "无");
                }
                draw_str(56, i, buf, false);
                break;

            case 6: /* 缓存条数 */
                draw_str(0, i, "缓存条数", false);
                snprintf(buf, sizeof(buf), "%u", app_cache_count());
                draw_str(80, i, buf, false);
                break;

            case 7: /* 采集间隔 */
                draw_str(0, i, "间隔", false);
                snprintf(buf, sizeof(buf), "%us", app_config_get_measure_interval(mode));
                draw_str(32, i, buf, false);
                break;

            case 8: /* 定位状态 */
                draw_str(0, i, "定位状态:", false);
                if (app_config_has_location()) {
                    const gnss_location_t *loc = gnss_get_location();
                    snprintf(buf, sizeof(buf), "OK S%d", loc->satellites);
                } else {
                    gnss_state_t gs = gnss_get_state();
                    if (gs == GNSS_STATE_OFF) {
                        snprintf(buf, sizeof(buf), "未定位");
                    } else {
                        snprintf(buf, sizeof(buf), "定位中");
                    }
                }
                draw_str(72, i, buf, false);
                break;

            case 9: /* 纬度 */
                draw_str(0, i, "纬度", false);
                if (app_config_has_location()) {
                    gnss_stored_location_t loc;
                    if (app_config_load_location(&loc)) {
                        snprintf(buf, sizeof(buf), "%c%d:%02d.%04d",
                                 loc.lat_dir, loc.lat_deg, loc.lat_min, loc.lat_frac);
                    } else {
                        snprintf(buf, sizeof(buf), "ERR");
                    }
                } else {
                    snprintf(buf, sizeof(buf), "--");
                }
                draw_str(32, i, buf, false);
                break;

            case 10: /* 经度 */
                draw_str(0, i, "经度", false);
                if (app_config_has_location()) {
                    gnss_stored_location_t loc;
                    if (app_config_load_location(&loc)) {
                        snprintf(buf, sizeof(buf), "%c%d:%02d.%04d",
                                 loc.lon_dir, loc.lon_deg, loc.lon_min, loc.lon_frac);
                    } else {
                        snprintf(buf, sizeof(buf), "ERR");
                    }
                } else {
                    snprintf(buf, sizeof(buf), "--");
                }
                draw_str(32, i, buf, false);
                break;
        }
    }
}

static void draw_settings_page(void)
{
    static const char *items[] = {
        "采集间隔",
        "发送间隔",
        "阈值设置",
        "LoRa设置",
        "设备信息",
        "重新定位",
        "恢复默认",
        "返回主页"
    };
    const uint8_t total = 8;
    const uint8_t visible = 4;
    uint8_t start = calc_scroll_start(cursor, total, visible);

    for (uint8_t i = 0; i < visible; i++) {
        uint8_t item = start + i;
        if (item < total) {
            draw_menu_line(i, items[item], item == cursor);
        }
    }
}

static void draw_meas_interval_page(void)
{
    char buf[22];

    snprintf(buf, sizeof(buf), "常规采集 %us", edit_config.measure_interval_normal);
    draw_menu_line(0, buf, cursor == 0);

    snprintf(buf, sizeof(buf), "预警采集 %us", edit_config.measure_interval_warning);
    draw_menu_line(1, buf, cursor == 1);

    snprintf(buf, sizeof(buf), "火警采集 %us", edit_config.measure_interval_alarm);
    draw_menu_line(2, buf, cursor == 2);

    draw_menu_line(3, "返回", cursor == 3);
}

static void draw_tx_interval_page(void)
{
    char buf[22];

    snprintf(buf, sizeof(buf), "常规发送 %us", edit_config.transmit_interval_normal);
    draw_menu_line(0, buf, cursor == 0);

    snprintf(buf, sizeof(buf), "预警发送 %us", edit_config.transmit_interval_warning);
    draw_menu_line(1, buf, cursor == 1);

    snprintf(buf, sizeof(buf), "火警发送 %us", edit_config.transmit_interval_alarm);
    draw_menu_line(2, buf, cursor == 2);

    draw_menu_line(3, "返回", cursor == 3);
}

static void draw_threshold_page(void)
{
    char buf[22];
    const uint8_t total = 9;
    const uint8_t visible = 4;
    uint8_t start = calc_scroll_start(cursor, total, visible);

    for (uint8_t i = 0; i < visible; i++) {
        uint8_t item = start + i;
        bool sel = (item == cursor);

        switch (item) {
            case 0: {
                int v = (int)(edit_config.temp_warning_threshold * 10);
                snprintf(buf, sizeof(buf), "温预 %d.%d℃", v / 10, v % 10);
                draw_menu_line(i, buf, sel);
                break;
            }
            case 1: {
                int v = (int)(edit_config.temp_warning_recovery * 10);
                snprintf(buf, sizeof(buf), "温预恢 %d.%d℃", v / 10, v % 10);
                draw_menu_line(i, buf, sel);
                break;
            }
            case 2: {
                int v = (int)(edit_config.temp_alarm_threshold * 10);
                snprintf(buf, sizeof(buf), "温报 %d.%d℃", v / 10, v % 10);
                draw_menu_line(i, buf, sel);
                break;
            }
            case 3: {
                int v = (int)(edit_config.temp_alarm_recovery * 10);
                snprintf(buf, sizeof(buf), "温报恢 %d.%d℃", v / 10, v % 10);
                draw_menu_line(i, buf, sel);
                break;
            }
            case 4: {
                int v = (int)(edit_config.co_warning_threshold * 10);
                snprintf(buf, sizeof(buf), "CO预 %d.%d", v / 10, v % 10);
                draw_menu_line(i, buf, sel);
                break;
            }
            case 5: {
                int v = (int)(edit_config.co_warning_recovery * 10);
                snprintf(buf, sizeof(buf), "CO恢 %d.%d", v / 10, v % 10);
                draw_menu_line(i, buf, sel);
                break;
            }
            case 6: {
                int v = (int)(edit_config.hum_warning_threshold + 0.5f);
                snprintf(buf, sizeof(buf), "湿预 %d%%", v);
                draw_menu_line(i, buf, sel);
                break;
            }
            case 7: {
                int v = (int)(edit_config.hum_warning_recovery + 0.5f);
                snprintf(buf, sizeof(buf), "湿恢 %d%%", v);
                draw_menu_line(i, buf, sel);
                break;
            }
            case 8:
                draw_menu_line(i, "返回", sel);
                break;
        }
    }
}

static void draw_lora_page(void)
{
    char buf[22];
    int8_t dbm = proto_power_level_to_dbm(edit_config.proto_power_level);

    snprintf(buf, sizeof(buf), "功率档位 %d(%ddBm)", edit_config.proto_power_level, dbm);
    draw_menu_line(0, buf, cursor == 0);

    snprintf(buf, sizeof(buf), "主扩频 SF%u", edit_config.proto_sf_primary);
    draw_menu_line(1, buf, cursor == 1);

    snprintf(buf, sizeof(buf), "备用扩频 SF%u", edit_config.proto_sf_backup);
    draw_menu_line(2, buf, cursor == 2);

    draw_menu_line(3, "返回", cursor == 3);
}

static void draw_device_page(void)
{
    const app_config_t *cfg = app_config_get();
    char buf[22];

    draw_str(0, 0, "设备信息", false);

    if (cfg->proto_has_short_addr) {
        snprintf(buf, sizeof(buf), "短地址 0x%04X", cfg->proto_short_addr);
    } else {
        snprintf(buf, sizeof(buf), "短地址 未注册");
    }
    draw_str(4, 1, buf, false);

    if (cfg->proto_parent_addr != PROTO_ADDR_BROADCAST) {
        snprintf(buf, sizeof(buf), "父节点 0x%04X", cfg->proto_parent_addr);
    } else {
        snprintf(buf, sizeof(buf), "父节点 无");
    }
    draw_str(4, 2, buf, false);

    draw_menu_line(3, "返回", cursor == 0);
}

static void draw_redeploy_page(void)
{
    draw_str(16, 0, "重新定位", false);
    draw_str(8, 1, "清除位置重新定位", false);

    draw_menu_line(2, "确认", cursor == 0);
    draw_menu_line(3, "取消", cursor == 1);
}

/*============================================================================
 *                              公开接口实现
 *===========================================================================*/
void app_menu_init(void)
{
    OLED_Init();

    in_settings = false;
    screen_on = false;
    current_page = PAGE_HOME;
    cursor = 0;
    edit_type = EDIT_NONE;
    home_scroll = 0;

    OLED_DisPlay_Off();
}

void app_menu_display(void)
{
    if (!screen_on) return;

    OLED_NewFrame();

    if (edit_type != EDIT_NONE) {
        draw_edit_page();
    } else {
        switch (current_page) {
            case PAGE_HOME:           draw_home_page();            break;
            case PAGE_SETTINGS:       draw_settings_page();        break;
            case PAGE_MEAS_INTERVAL:  draw_meas_interval_page();   break;
            case PAGE_TX_INTERVAL:    draw_tx_interval_page();     break;
            case PAGE_THRESHOLD:      draw_threshold_page();       break;
            case PAGE_LORA:           draw_lora_page();            break;
            case PAGE_DEVICE:         draw_device_page();          break;
            case PAGE_REDEPLOY:       draw_redeploy_page();        break;
            default: break;
        }
    }

    /* 编辑时长按UP/DOWN自动加减 */
    if (edit_type != EDIT_NONE && edit_hold_active) {
        bool up_held   = btn_is_pressed(BTN_UP);
        bool down_held = btn_is_pressed(BTN_DOWN);
        if (!up_held && !down_held) {
            edit_hold_active = false;
        } else {
            uint32_t held_ms = HAL_GetTick() - edit_hold_tick;
            /* 速度递增: <1.5s: 200ms/步, 1.5~4s: 100ms, >4s: 50ms */
            uint32_t interval = (held_ms < 1500) ? 200 : ((held_ms < 4000) ? 100 : 50);
            static uint32_t last_repeat = 0;
            if ((HAL_GetTick() - last_repeat) >= interval) {
                last_repeat = HAL_GetTick();
                edit_handle_button(up_held ? BTN_UP : BTN_DOWN, BTN_EVENT_SHORT_CLICK);
            }
        }
    }

    OLED_ShowFrame();
}

bool app_menu_handle_button(board_btn_t btn, btn_event_t event)
{
    /* 熄屏状态：长按OK亮屏，忽略其他所有按键 */
    if (!screen_on) {
        if (btn == BTN_OK && event == BTN_EVENT_LONG_PRESS) {
            screen_on = true;
            OLED_DisPlay_On();
            return true;
        }
        return false;
    }

    /* 编辑页面优先处理 */
    if (edit_type != EDIT_NONE) {
        return edit_handle_button(btn, event);
    }

    /* 主页：UP/DOWN滚动数据，OK短按进入设置，OK长按熄屏 */
    if (!in_settings) {
        if (event == BTN_EVENT_SHORT_CLICK) {
            switch (btn) {
                case BTN_UP:
                    if (home_scroll > 0) home_scroll--;
                    return true;
                case BTN_DOWN:
                    if (home_scroll < HOME_ITEM_COUNT - 4) home_scroll++;
                    return true;
                case BTN_OK:
                    app_menu_enter_settings();
                    return true;
                default:
                    break;
            }
        }
        /* 主页长按OK → 熄屏 */
        if (btn == BTN_OK && event == BTN_EVENT_LONG_PRESS) {
            screen_on = false;
            OLED_DisPlay_Off();
            screen_off_tick = HAL_GetTick();
            return true;
        }
        return false;
    }

    /* 设置菜单中的按键处理 */
    if (event == BTN_EVENT_SHORT_CLICK) {
        switch (btn) {
            case BTN_UP:
                if (cursor > 0) cursor--;
                return true;

            case BTN_DOWN:
                if (cursor < page_item_count[current_page] - 1) cursor++;
                return true;

            case BTN_OK: {
                switch (current_page) {
                    case PAGE_SETTINGS:
                        switch (cursor) {
                            case 0: current_page = PAGE_MEAS_INTERVAL; cursor = 0; break;
                            case 1: current_page = PAGE_TX_INTERVAL;   cursor = 0; break;
                            case 2: current_page = PAGE_THRESHOLD;     cursor = 0; break;
                            case 3: current_page = PAGE_LORA;          cursor = 0; break;
                            case 4: current_page = PAGE_DEVICE;        cursor = 0; break;
                            case 5: current_page = PAGE_REDEPLOY;      cursor = 0; break;
                            case 6: /* 恢复默认 */
                                app_config_reset();
                                load_edit_config();
                                break;
                            case 7: /* 返回主页 */
                                app_menu_exit_settings();
                                break;
                        }
                        return true;

                    case PAGE_MEAS_INTERVAL:
                        switch (cursor) {
                            case 0: edit_type = EDIT_MEAS_INTERVAL_NORMAL;  break;
                            case 1: edit_type = EDIT_MEAS_INTERVAL_WARNING; break;
                            case 2: edit_type = EDIT_MEAS_INTERVAL_ALARM;   break;
                            case 3: /* 返回 */
                                save_edit_config();
                                current_page = PAGE_SETTINGS;
                                cursor = 0;
                                break;
                        }
                        return true;

                    case PAGE_TX_INTERVAL:
                        switch (cursor) {
                            case 0: edit_type = EDIT_TX_INTERVAL_NORMAL;  break;
                            case 1: edit_type = EDIT_TX_INTERVAL_WARNING; break;
                            case 2: edit_type = EDIT_TX_INTERVAL_ALARM;   break;
                            case 3: /* 返回 */
                                save_edit_config();
                                current_page = PAGE_SETTINGS;
                                cursor = 1;
                                break;
                        }
                        return true;

                    case PAGE_THRESHOLD:
                        switch (cursor) {
                            case 0: edit_type = EDIT_TEMP_WARNING;          break;
                            case 1: edit_type = EDIT_TEMP_WARNING_RECOVERY; break;
                            case 2: edit_type = EDIT_TEMP_ALARM;            break;
                            case 3: edit_type = EDIT_TEMP_RECOVERY;         break;
                            case 4: edit_type = EDIT_CO_WARNING;            break;
                            case 5: edit_type = EDIT_CO_RECOVERY;           break;
                            case 6: edit_type = EDIT_HUM_WARNING;           break;
                            case 7: edit_type = EDIT_HUM_RECOVERY;          break;
                            case 8: /* 返回 */
                                save_edit_config();
                                current_page = PAGE_SETTINGS;
                                cursor = 2;
                                break;
                        }
                        return true;

                    case PAGE_LORA:
                        switch (cursor) {
                            case 0: edit_type = EDIT_LORA_POWER_LEVEL; break;
                            case 1: edit_type = EDIT_LORA_SF_PRI;      break;
                            case 2: edit_type = EDIT_LORA_SF_BAK;      break;
                            case 3: /* 返回 */
                                save_edit_config();
                                current_page = PAGE_SETTINGS;
                                cursor = 3;
                                break;
                        }
                        return true;

                    case PAGE_DEVICE:
                        current_page = PAGE_SETTINGS;
                        cursor = 4;
                        return true;

                    case PAGE_REDEPLOY:
                        switch (cursor) {
                            case 0: /* 确认重新定位 */
                                redeploy_requested = true;
                                app_menu_exit_settings();
                                break;
                            case 1: /* 取消 */
                                current_page = PAGE_SETTINGS;
                                cursor = 5;
                                break;
                        }
                        return true;

                    default:
                        break;
                }
                return true;
            }

            default:
                break;
        }
    }

    /* 长按OK - 返回上级/退出设置 */
    if (btn == BTN_OK && event == BTN_EVENT_LONG_PRESS) {
        if (current_page == PAGE_SETTINGS) {
            app_menu_exit_settings();
        } else {
            save_edit_config();
            current_page = PAGE_SETTINGS;
            cursor = 0;
        }
        return true;
    }

    return false;
}

void app_menu_enter_settings(void)
{
    in_settings = true;
    current_page = PAGE_SETTINGS;
    cursor = 0;
    edit_type = EDIT_NONE;
    load_edit_config();
}

void app_menu_exit_settings(void)
{
    in_settings = false;
    current_page = PAGE_HOME;
    cursor = 0;
    edit_type = EDIT_NONE;
}

bool app_menu_is_main_page(void)
{
    return (current_page == PAGE_HOME);
}

bool app_menu_is_settings_page(void)
{
    return in_settings;
}

bool app_menu_is_screen_on(void)
{
    return screen_on;
}

void app_menu_request_redeploy(void)
{
    redeploy_requested = true;
}

bool app_menu_has_redeploy_request(void)
{
    if (redeploy_requested) {
        redeploy_requested = false;
        return true;
    }
    return false;
}

void app_menu_wake_screen(void)
{
    /* 防抖: 息屏后 500ms 内忽略唤醒, 避免长按 OK 的释放沿触发 */
    if (screen_off_tick != 0 && (HAL_GetTick() - screen_off_tick) < 500) {
        screen_off_tick = 0;
        return;
    }
    screen_off_tick = 0;
    screen_on = true;
    OLED_DisPlay_On();
}
