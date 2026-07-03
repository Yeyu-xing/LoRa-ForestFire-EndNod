/**
  ******************************************************************************
  * @file    driver_eeprom.h
  * @brief   BL24C256A EEPROM驱动 (I2C接口)
  * @note    容量: 32KB, 页大小: 64字节, 写周期: 3ms
  ******************************************************************************
  */

#ifndef DRIVER_EEPROM_H_
#define DRIVER_EEPROM_H_

#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 *                              配置参数
 *===========================================================================*/
/* EEPROM I2C地址 (7位地址, A0=A1=A2=0) */
#define EEPROM_I2C_ADDR         0x50

/* EEPROM容量和页大小 */
#define EEPROM_SIZE             32768       // 32KB
#define EEPROM_PAGE_SIZE        64          // 页大小
#define EEPROM_PAGE_MASK        (EEPROM_PAGE_SIZE - 1)

/* 写周期时间 (ms) */
#define EEPROM_WRITE_CYCLE_MS   3

/* I2C超时时间 (ms) */
#define EEPROM_I2C_TIMEOUT_MS   100

/*============================================================================
 *                              接口函数
 *===========================================================================*/
/**
  * @brief  检测EEPROM是否存在
  * @retval true: 存在, false: 不存在
  */
bool eeprom_is_present(void);

/**
  * @brief  等待EEPROM写入完成 (ACK轮询)
  * @param  timeout_ms: 超时时间
  * @retval true: 就绪, false: 超时
  */
bool eeprom_wait_ready(uint32_t timeout_ms);

/**
  * @brief  写入单个字节
  * @param  addr: 地址 (0 ~ 32767)
  * @param  data: 数据
  * @retval true: 成功, false: 失败
  */
bool eeprom_write_byte(uint16_t addr, uint8_t data);

/**
  * @brief  读取单个字节
  * @param  addr: 地址 (0 ~ 32767)
  * @param  data: 数据输出指针
  * @retval true: 成功, false: 失败
  */
bool eeprom_read_byte(uint16_t addr, uint8_t *data);

/**
  * @brief  写入多字节数据 (自动处理跨页)
  * @param  addr: 起始地址
  * @param  data: 数据指针
  * @param  len: 数据长度
  * @retval true: 成功, false: 失败
  * @note   自动处理页边界，确保数据正确写入
  */
bool eeprom_write(uint16_t addr, const uint8_t *data, uint16_t len);

/**
  * @brief  读取多字节数据
  * @param  addr: 起始地址
  * @param  data: 数据输出缓冲区
  * @param  len: 读取长度
  * @retval true: 成功, false: 失败
  */
bool eeprom_read(uint16_t addr, uint8_t *data, uint16_t len);

/**
  * @brief  填充指定区域
  * @param  addr: 起始地址
  * @param  value: 填充值
  * @param  len: 填充长度
  * @retval true: 成功, false: 失败
  */
bool eeprom_fill(uint16_t addr, uint8_t value, uint16_t len);

#endif /* DRIVER_EEPROM_H_ */
