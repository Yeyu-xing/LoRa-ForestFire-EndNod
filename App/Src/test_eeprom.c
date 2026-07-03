/**
  ******************************************************************************
  * @file    test_eeprom.c
  * @brief   EEPROM驱动测试实现
  ******************************************************************************
  */

#include "test_eeprom.h"
#include "driver_eeprom.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

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
void test_eeprom_start(void)
{
    char msg[128];
    uint8_t test_buf[64];
    uint8_t read_buf[64];
    
    uart2_print("\r\n=== EEPROM Test ===\r\n");
    
    // 检测EEPROM是否存在
    if (!eeprom_is_present()) {
        uart2_print("EEPROM not found!\r\n");
        return;
    }
    uart2_print("EEPROM detected OK\r\n");
    
    // 测试1: 单字节读写
    uart2_print("\r\n[Test 1] Single byte write/read...\r\n");
    uint8_t test_val = 0xA5;
    if (eeprom_write_byte(0x0000, test_val)) {
        if (eeprom_read_byte(0x0000, &test_buf[0])) {
            if (test_buf[0] == test_val) {
                uart2_print("  Single byte: PASS\r\n");
            } else {
                uart2_print("  Single byte: FAIL (data mismatch)\r\n");
            }
        } else {
            uart2_print("  Single byte: FAIL (read error)\r\n");
        }
    } else {
        uart2_print("  Single byte: FAIL (write error)\r\n");
    }
    
    // 测试2: 页写入 (不跨页)
    uart2_print("\r\n[Test 2] Page write (within page)...\r\n");
    for (int i = 0; i < 32; i++) {
        test_buf[i] = (uint8_t)(i * 2);
    }
    if (eeprom_write(0x0040, test_buf, 32)) {
        if (eeprom_read(0x0040, read_buf, 32)) {
            bool match = true;
            for (int i = 0; i < 32; i++) {
                if (read_buf[i] != test_buf[i]) {
                    match = false;
                    break;
                }
            }
            if (match) {
                uart2_print("  Page write: PASS\r\n");
            } else {
                uart2_print("  Page write: FAIL (data mismatch)\r\n");
            }
        } else {
            uart2_print("  Page write: FAIL (read error)\r\n");
        }
    } else {
        uart2_print("  Page write: FAIL (write error)\r\n");
    }
    
    // 测试3: 跨页写入
    uart2_print("\r\n[Test 3] Cross-page write...\r\n");
    // 从地址0x0030写入64字节，会跨越页边界(0x0040)
    for (int i = 0; i < 64; i++) {
        test_buf[i] = (uint8_t)(0x55 + i);
    }
    if (eeprom_write(0x0030, test_buf, 64)) {
        if (eeprom_read(0x0030, read_buf, 64)) {
            bool match = true;
            for (int i = 0; i < 64; i++) {
                if (read_buf[i] != test_buf[i]) {
                    match = false;
                    snprintf(msg, sizeof(msg), "  Mismatch at %d: W=0x%02X, R=0x%02X\r\n",
                             i, test_buf[i], read_buf[i]);
                    uart2_print(msg);
                    break;
                }
            }
            if (match) {
                uart2_print("  Cross-page write: PASS\r\n");
            } else {
                uart2_print("  Cross-page write: FAIL\r\n");
            }
        } else {
            uart2_print("  Cross-page write: FAIL (read error)\r\n");
        }
    } else {
        uart2_print("  Cross-page write: FAIL (write error)\r\n");
    }
    
    // 测试4: 填充测试
    uart2_print("\r\n[Test 4] Fill operation...\r\n");
    if (eeprom_fill(0x0100, 0xFF, 128)) {
        if (eeprom_read(0x0100, read_buf, 128)) {
            bool match = true;
            for (int i = 0; i < 128; i++) {
                if (read_buf[i] != 0xFF) {
                    match = false;
                    break;
                }
            }
            if (match) {
                uart2_print("  Fill: PASS\r\n");
            } else {
                uart2_print("  Fill: FAIL (data mismatch)\r\n");
            }
        } else {
            uart2_print("  Fill: FAIL (read error)\r\n");
        }
    } else {
        uart2_print("  Fill: FAIL (write error)\r\n");
    }
    
    uart2_print("\r\n=== EEPROM Test Complete ===\r\n");
}
