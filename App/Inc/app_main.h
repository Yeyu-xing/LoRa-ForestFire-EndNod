/**
  ******************************************************************************
  * @file    app_main.h
  * @brief   应用主框架 - 状态机与主循环
  ******************************************************************************
  */

#ifndef APP_MAIN_H_
#define APP_MAIN_H_

#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"

/*============================================================================
 *                              状态定义
 *===========================================================================*/
typedef enum {
    STATE_INIT = 0,         // 系统初始化
    STATE_IDLE,             // 空闲待机
    STATE_SETTINGS,         // 设置模式 (OLED菜单操作)
    STATE_MEASURE,          // 传感器采集
    STATE_GNSS,             // GNSS定位 (部署时获取位置)
    STATE_TRANSMIT,         // LoRa发送
    STATE_RECEIVE,          // LoRa接收窗口 (Class A)
    STATE_SLEEP,            // 低功耗休眠
    STATE_ERROR,            // 错误处理
    STATE_PROTO_SCAN,       // 协议: 扫描Beacon
    STATE_PROTO_REGISTERING,// 协议: 注册
} app_state_t;

/*============================================================================
 *                              传感器数据
 *===========================================================================*/
typedef struct {
    float temperature;          // 温度(°C)
    float humidity;             // 湿度(%RH)
    float co_concentration;     // CO浓度(ppm)
    float battery_voltage;      // 电池电压(V)
    bool  sht40_valid;          // SHT40数据有效
    bool  co_valid;             // CO数据有效
} sensor_data_t;

/*============================================================================
 *                              错误标志
 *===========================================================================*/
#define ERROR_FLAG_SHT40        0x0001      // SHT40故障
#define ERROR_FLAG_EEPROM       0x0002      // EEPROM故障
#define ERROR_FLAG_OLED         0x0004      // OLED故障
#define ERROR_FLAG_LORA         0x0008      // LoRa故障
#define ERROR_FLAG_ADC          0x0010      // ADC故障
#define ERROR_FLAG_GNSS         0x0020      // GNSS故障

/*============================================================================
 *                              接口函数
 *===========================================================================*/
/**
  * @brief  应用初始化
  * @note   初始化所有模块，加载配置
  */
void app_init(void);

/**
  * @brief  应用主循环
  * @note   在main()的while(1)中调用
  */
void app_process(void);

/**
  * @brief  获取当前状态
  * @retval 状态枚举
  */
app_state_t app_get_state(void);

/**
  * @brief  获取当前运行模式
  * @retval 运行模式枚举
  */
run_mode_t app_get_run_mode(void);

/**
  * @brief  获取错误标志
  * @retval 错误标志位
  */
uint16_t app_get_errors(void);

/**
  * @brief  获取最新传感器数据 (只读)
  * @retval 传感器数据指针
  */
const sensor_data_t *app_get_sensor_data(void);

#endif /* APP_MAIN_H_ */
