/**
  ******************************************************************************
  * @file    test_oled.c
  * @brief   OLED屏幕测试 - 排查显示不亮问题
  * @note    测试步骤:
  *         1. I2C2总线扫描，查找OLED设备地址
  *         2. 延时等待OLED上电稳定
  *         3. 尝试直接I2C通信发送命令
  *         4. 调用OLED_Init()初始化
  *         5. 显示测试图案(全屏填充/文字/几何图形)
  *         6. 通过UART2输出每步结果
  ******************************************************************************
  */

#include "test_oled.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>

#include "i2c.h"
#include "usart.h"
#include "oled.h"
#include "stm32wlxx_hal.h"

/*============================================================================
 *                              私有函数 - UART输出
 *===========================================================================*/
static void uart_print(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
}

static void uart_printf(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    uart_print(buf);
}

/*============================================================================
 *                              私有函数 - I2C扫描
 *===========================================================================*/
/**
  * @brief  扫描I2C2总线上的设备
  * @retval 找到的设备数量
  */
static uint8_t i2c2_scan(void)
{
    uint8_t found = 0;

    uart_print("\r\n--- I2C2 Bus Scan ---\r\n");

    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        uint8_t data = 0;
        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)(addr << 1),
                                                            &data, 0, 50);
        if (status == HAL_OK) {
            uart_printf("  Found device at 0x%02X (7-bit: 0x%02X)\r\n",
                        addr << 1, addr);
            found++;
        }
    }

    if (found == 0) {
        uart_print("  No I2C device found!\r\n");
    }
    uart_printf("Scan done, %d device(s) found\r\n", found);

    return found;
}

/*============================================================================
 *                              私有函数 - I2C2直接通信测试
 *===========================================================================*/
/**
  * @brief  直接通过HAL I2C函数与OLED通信测试
  * @retval true: 通信成功, false: 通信失败
  */
static bool i2c2_direct_test(void)
{
    uint8_t cmd_buf[2] = {0x00, 0xAE};  // Control byte + 关闭显示命令

    uart_print("\r\n--- I2C2 Direct Communication Test ---\r\n");

    // 测试写0x78地址 (OLED地址)
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, 0x78, cmd_buf, 2, 100);
    if (status == HAL_OK) {
        uart_print("  Write to 0x78: OK\r\n");
    } else {
        uart_printf("  Write to 0x78: FAIL (status=%d)\r\n", status);
        return false;
    }

    // 测试写0x3C地址 (7位地址)
    status = HAL_I2C_Master_Transmit(&hi2c2, 0x3C, cmd_buf, 2, 100);
    if (status == HAL_OK) {
        uart_print("  Write to 0x3C: OK\r\n");
    } else {
        uart_printf("  Write to 0x3C: FAIL (status=%d)\r\n", status);
    }

    return true;
}

/*============================================================================
 *                              私有函数 - I2C2引脚状态检查
 *===========================================================================*/
static void check_i2c2_pins(void)
{
    uart_print("\r\n--- I2C2 Pin State Check ---\r\n");

    GPIO_TypeDef *scl_port = GPIOA;
    uint16_t scl_pin = GPIO_PIN_12;
    GPIO_TypeDef *sda_port = GPIOA;
    uint16_t sda_pin = GPIO_PIN_11;

    // 读取引脚当前状态
    GPIO_PinState scl_state = HAL_GPIO_ReadPin(scl_port, scl_pin);
    GPIO_PinState sda_state = HAL_GPIO_ReadPin(sda_port, sda_pin);

    uart_printf("  SCL (PA12): %s\r\n", scl_state ? "HIGH" : "LOW");
    uart_printf("  SDA (PA11): %s\r\n", sda_state ? "HIGH" : "LOW");

    // I2C空闲时两根线都应该是HIGH (开漏+外部上拉)
    if (scl_state == GPIO_PIN_RESET) {
        uart_print("  WARNING: SCL is LOW! Check pull-up resistors.\r\n");
    }
    if (sda_state == GPIO_PIN_RESET) {
        uart_print("  WARNING: SDA is LOW! Check pull-up resistors or bus stuck.\r\n");
    }
}

/*============================================================================
 *                              私有函数 - 简易SSD1306初始化
 *===========================================================================*/
/**
  * @brief  最小化SSD1306初始化，不依赖oled.c
  * @retval true: 成功, false: 失败
  */
static bool ssd1306_minimal_init(void)
{
    uart_print("\r\n--- Minimal SSD1306 Init ---\r\n");

    // 通过I2C直接发送SSD1306命令序列
    // 命令格式: [0x00(控制字节), 命令]
    uint8_t cmds[] = {
        0xAE,       // 关闭显示
        0xD5, 0x80, // 设置时钟分频
        0xA8, 0x3F, // 设置复用率 1/64
        0xD3, 0x00, // 设置显示偏移
        0x40,       // 设置起始行
        0x8D, 0x14, // 启用电荷泵
        0x20, 0x00, // 设置寻址模式为水平
        0xA1,       // 设置段映射
        0xC8,       // 设置COM扫描方向
        0xDA, 0x12, // 设置COM引脚配置
        0x81, 0xCF, // 设置对比度
        0xD9, 0xF1, // 设置预充电周期
        0xDB, 0x40, // 设置VCOMH电压
        0xA4,       // 输出跟随RAM
        0xA6,       // 正常显示
        0xAF,       // 开启显示
    };

    for (uint16_t i = 0; i < sizeof(cmds); i++) {
        uint8_t buf[2] = {0x00, cmds[i]};
        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, 0x78, buf, 2, 100);
        if (status != HAL_OK) {
            uart_printf("  Cmd 0x%02X failed (status=%d)\r\n", cmds[i], status);
            return false;
        }
    }

    uart_print("  All commands sent OK\r\n");
    return true;
}

/*============================================================================
 *                              私有函数 - 全屏填充测试
 *===========================================================================*/
static bool ssd1306_fill_all(uint8_t pattern)
{
    // 设置列范围和页范围 (水平寻址模式)
    uint8_t range_cmds[] = {
        0x21, 0x00, 0x7F,   // 设置列范围 0-127
        0x22, 0x00, 0x07,   // 设置页范围 0-7
    };
    for (uint16_t i = 0; i < sizeof(range_cmds); i++) {
        uint8_t buf[2] = {0x00, range_cmds[i]};
        if (HAL_I2C_Master_Transmit(&hi2c2, 0x78, buf, 2, 100) != HAL_OK) {
            return false;
        }
    }

    // 发送数据: 128列 x 8页 = 1024字节
    uint8_t data_buf[129];  // 1控制字节 + 128数据字节
    data_buf[0] = 0x40;     // 数据控制字节
    memset(&data_buf[1], pattern, 128);

    for (uint8_t page = 0; page < 8; page++) {
        if (HAL_I2C_Master_Transmit(&hi2c2, 0x78, data_buf, 129, 200) != HAL_OK) {
            return false;
        }
    }
    return true;
}

/*============================================================================
 *                              测试状态
 *===========================================================================*/
typedef enum {
    TEST_STEP_SCAN = 0,
    TEST_STEP_PINS,
    TEST_STEP_DIRECT,
    TEST_STEP_DELAY,
    TEST_STEP_OLED_INIT,
    TEST_STEP_MINIMAL_INIT,
    TEST_STEP_FILL_WHITE,
    TEST_STEP_FILL_BLACK,
    TEST_STEP_LIB_INIT,
    TEST_STEP_LIB_TEXT,
    TEST_STEP_DONE,
} test_step_t;

static test_step_t current_step = TEST_STEP_SCAN;
static uint32_t step_tick = 0;
static bool test_running = false;

/*============================================================================
 *                              公开接口
 *===========================================================================*/
void test_oled_start(void)
{
    uart_print("\r\n========================================\r\n");
    uart_print(  "       OLED Display Test Program        \r\n");
    uart_print(  "========================================\r\n");
    uart_printf( "I2C2: PA12(SCL) PA11(SDA), OLED addr: 0x78\r\n");

    current_step = TEST_STEP_SCAN;
    step_tick = HAL_GetTick();
    test_running = true;
}

void test_oled_process(void)
{
    if (!test_running) return;

    switch (current_step) {
        case TEST_STEP_SCAN:
            i2c2_scan();
            current_step = TEST_STEP_PINS;
            break;

        case TEST_STEP_PINS:
            check_i2c2_pins();
            current_step = TEST_STEP_DIRECT;
            break;

        case TEST_STEP_DIRECT:
            i2c2_direct_test();
            uart_print("\r\nWaiting 200ms for OLED power-up...\r\n");
            step_tick = HAL_GetTick();
            current_step = TEST_STEP_DELAY;
            break;

        case TEST_STEP_DELAY:
            if ((HAL_GetTick() - step_tick) >= 200) {
                current_step = TEST_STEP_OLED_INIT;
            }
            break;

        case TEST_STEP_OLED_INIT: {
            uart_print("\r\n--- OLED_Init() Test ---\r\n");
            OLED_Init();
            uart_print("  OLED_Init() completed (check if screen is on)\r\n");

            step_tick = HAL_GetTick();
            current_step = TEST_STEP_MINIMAL_INIT;
            break;
        }

        case TEST_STEP_MINIMAL_INIT:
            if ((HAL_GetTick() - step_tick) >= 2000) {
                uart_print("\r\n--- Minimal Init + Fill White (2s) ---\r\n");
                if (ssd1306_minimal_init()) {
                    HAL_Delay(100);
                    if (ssd1306_fill_all(0xFF)) {
                        uart_print("  Screen should be all white now!\r\n");
                    } else {
                        uart_print("  Fill failed!\r\n");
                    }
                }
                step_tick = HAL_GetTick();
                current_step = TEST_STEP_FILL_WHITE;
            }
            break;

        case TEST_STEP_FILL_WHITE:
            // 白屏显示2秒
            if ((HAL_GetTick() - step_tick) >= 2000) {
                uart_print("\r\n--- Fill Black (2s) ---\r\n");
                ssd1306_fill_all(0x00);
                uart_print("  Screen should be all black now!\r\n");
                step_tick = HAL_GetTick();
                current_step = TEST_STEP_FILL_BLACK;
            }
            break;

        case TEST_STEP_FILL_BLACK:
            // 黑屏显示2秒
            if ((HAL_GetTick() - step_tick) >= 2000) {
                uart_print("\r\n--- Library Init + Text Display ---\r\n");
                // 使用驱动库重新初始化
                OLED_Init();
                OLED_DisPlay_On();

                // 用库绘制测试文字
                OLED_NewFrame();
                OLED_PrintASCIIString(0, 0, "OLED TEST OK!", &afont8x6, OLED_COLOR_NORMAL);
                OLED_PrintASCIIString(0, 16, "Line 2", &afont8x6, OLED_COLOR_NORMAL);
                OLED_PrintASCIIString(0, 32, "Line 3", &afont8x6, OLED_COLOR_NORMAL);
                OLED_PrintASCIIString(0, 48, "Hello World!", &afont8x6, OLED_COLOR_NORMAL);
                OLED_ShowFrame();
                uart_print("  Text displayed (check screen)\r\n");

                step_tick = HAL_GetTick();
                current_step = TEST_STEP_LIB_TEXT;
            }
            break;

        case TEST_STEP_LIB_TEXT:
            // 文字显示持续，每3秒交替黑屏/文字
            if ((HAL_GetTick() - step_tick) >= 3000) {
                static bool toggle = false;
                toggle = !toggle;

                if (toggle) {
                    OLED_NewFrame();
                    OLED_DrawFilledRectangle(0, 0, 127, 63, OLED_COLOR_NORMAL);
                    OLED_ShowFrame();
                } else {
                    OLED_NewFrame();
                    OLED_PrintASCIIString(8, 8, "OLED OK!", &afont8x6, OLED_COLOR_REVERSED);
                    OLED_PrintASCIIString(8, 24, "Toggle test", &afont8x6, OLED_COLOR_REVERSED);
                    OLED_ShowFrame();
                }

                step_tick = HAL_GetTick();
            }
            break;

        default:
            break;
    }
}
