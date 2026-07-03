/**
  ******************************************************************************
  * @file    driver_sht40.h
  * @brief   SHT40温湿度传感器驱动
  * @note    I2C接口，支持CRC-8校验
  ******************************************************************************
  */

#ifndef DRIVER_SHT40_H_
#define DRIVER_SHT40_H_

#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 *                              配置参数
 *===========================================================================*/
/* SHT40 I2C地址 (7位地址) */
#define SHT40_I2C_ADDR          0x44

/* 测量命令 */
#define SHT40_CMD_HIGH_PRECISION    0xFD    // 高精度测量，耗时~8.2ms
#define SHT40_CMD_MED_PRECISION     0xF6    // 中精度测量，耗时~4.5ms
#define SHT40_CMD_LOW_PRECISION     0xE0    // 低精度测量，耗时~1.6ms

/* 其他命令 */
#define SHT40_CMD_READ_SERIAL       0x89    // 读取序列号
#define SHT40_CMD_SOFT_RESET        0x94    // 软复位

/* 测量等待时间 (ms) */
#define SHT40_MEASURE_TIME_HIGH_MS  10
#define SHT40_MEASURE_TIME_MED_MS   5
#define SHT40_MEASURE_TIME_LOW_MS   2

/* I2C超时时间 (ms) */
#define SHT40_I2C_TIMEOUT_MS        100

/*============================================================================
 *                              类型定义
 *===========================================================================*/
/* 温湿度数据结构 */
typedef struct {
    float temperature;      // 温度 (°C)
    float humidity;         // 相对湿度 (%RH)
    bool valid;             // 数据有效标志
} sht40_data_t;

/* 测量精度枚举 */
typedef enum {
    SHT40_PRECISION_HIGH = 0,   // 高精度
    SHT40_PRECISION_MED,        // 中精度
    SHT40_PRECISION_LOW,        // 低精度
} sht40_precision_t;

/*============================================================================
 *                              接口函数
 *===========================================================================*/
/**
  * @brief  初始化SHT40传感器
  * @note   通过软复位检测传感器是否存在
  * @retval true: 初始化成功, false: 传感器未响应
  */
bool sht40_init(void);

/**
  * @brief  检测传感器是否存在
  * @retval true: 存在, false: 不存在
  */
bool sht40_is_present(void);

/**
  * @brief  读取温湿度数据
  * @param  data: 输出数据结构指针
  * @param  precision: 测量精度
  * @retval true: 成功, false: 失败
  */
bool sht40_read(sht40_data_t *data, sht40_precision_t precision);

/**
  * @brief  读取序列号
  * @param  serial: 输出序列号 (32位)
  * @retval true: 成功, false: 失败
  */
bool sht40_read_serial(uint32_t *serial);

/**
  * @brief  软复位传感器
  * @retval true: 成功, false: 失败
  */
bool sht40_reset(void);

#endif /* DRIVER_SHT40_H_ */
