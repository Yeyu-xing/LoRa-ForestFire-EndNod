/**
  ******************************************************************************
  * @file    test_sht40.c
  * @brief   SHT40温湿度传感器测试实现
  ******************************************************************************
  */

#include "test_sht40.h"

#include <stdio.h>

#include "driver_sht40.h"
#include "usart.h"
#include <string.h>

/*============================================================================
 *                              私有变量
 *===========================================================================*/
static uint32_t test_interval_ms = 0;

/*============================================================================
 *                              私有函数
 *===========================================================================*/
static void uart2_print(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
}

/*============================================================================
 *                              测试函数
 *===========================================================================*/
void test_sht40_start(void)
{
    uart2_print("\r\n=== SHT40 Sensor Test ===\r\n");
    
    // 初始化传感器
    if (sht40_init()) {
        uart2_print("SHT40 initialized OK\r\n");
    } else {
        uart2_print("SHT40 not found!\r\n");
        return;
    }
    
    // 读取序列号
    uint32_t serial;
    if (sht40_read_serial(&serial)) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Serial: 0x%08lX\r\n", serial);
        uart2_print(msg);
    }
    
    uart2_print("\r\nReading sensor data...\r\n");
    test_interval_ms = 0;
}

void test_sht40_process(void)
{
    sht40_data_t data;
    
    // 每2秒读取一次
    test_interval_ms++;
    if (test_interval_ms < 2000) {
        return;
    }
    test_interval_ms = 0;
    
    // 读取温湿度（高精度）
    if (sht40_read(&data, SHT40_PRECISION_HIGH)) {
        char msg[64];
        snprintf(msg, sizeof(msg), "T: %.2f C, RH: %.1f %%\r\n",
                 data.temperature, data.humidity);
        uart2_print(msg);
    } else {
        uart2_print("Read failed!\r\n");
    }
}
