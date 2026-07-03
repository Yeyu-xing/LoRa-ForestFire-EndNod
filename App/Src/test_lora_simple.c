/**
 * @file    test_lora_simple.c
 * @brief   完全参照ST官方案例PING_PONG的极简LoRa收发测试
 */
#include "test_lora_simple.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "radio.h"
#include "radio_driver.h"
#include "usart.h"

/*============================================================================
 *                              参数定义
 *===========================================================================*/
#define RF_FREQUENCY            471500000
/* 射频参数配置 - 与主程序app_lora.h保持一致 */
#define TX_OUTPUT_POWER         14
#define LORA_BANDWIDTH          0           // 0: 125kHz
#define LORA_SPREADING_FACTOR   7           // SF7
#define LORA_CODINGRATE         4           // 4/8（与主程序一致）
#define LORA_PREAMBLE_LENGTH    8
#define LORA_SYMBOL_TIMEOUT     5
#define LORA_IQ_INVERSION_ON    false
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define TX_TIMEOUT_VALUE        5000
#define RX_TIMEOUT_VALUE        3000
#define PAYLOAD_LEN             24
#define MAX_BUF_SIZE            255

/*============================================================================
 *                              状态与标志
 *===========================================================================*/
typedef enum {
    TEST_IDLE = 0,
    TEST_TX_WAIT,
} test_state_t;

/* ISR只置位这些标志，主循环处理 */
static volatile bool tx_done_flag = false;
static volatile bool tx_timeout_flag = false;

static test_state_t test_state = TEST_IDLE;
static RadioEvents_t radio_events;
static uint8_t tx_buffer[MAX_BUF_SIZE];
static uint32_t test_tx_count = 0;
static uint32_t test_timeout_count = 0;
static uint32_t last_tx_tick = 0;
#define TX_INTERVAL_MS  5000

/*============================================================================
 *                              UART调试输出
 *===========================================================================*/
static void test_printf(const char *fmt, ...)
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

/*============================================================================
 *                              Radio回调（ISR上下文）
 *              只设标志，不做其他任何操作
 *===========================================================================*/
static void OnTxDone(void)
{
    tx_done_flag = true;
}

static void OnTxTimeout(void)
{
    tx_timeout_flag = true;
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    (void)payload; (void)size; (void)rssi; (void)snr;
}

static void OnRxTimeout(void) {}
static void OnRxError(void) {}

/*============================================================================
 *                              初始化
 *===========================================================================*/
void test_lora_simple_init(void)
{
    radio_events.TxDone    = OnTxDone;
    radio_events.RxDone    = OnRxDone;
    radio_events.TxTimeout = OnTxTimeout;
    radio_events.RxTimeout = OnRxTimeout;
    radio_events.RxError   = OnRxError;

    Radio.Init(&radio_events);
    Radio.SetChannel(RF_FREQUENCY);

    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, false, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, false, 0, LORA_IQ_INVERSION_ON, true);

    Radio.SetMaxPayloadLength(MODEM_LORA, MAX_BUF_SIZE);

    /* 私网同步字 0x1424 - 与主程序/SX1268网关一致 */
    Radio.SetPublicNetwork(false);

    memset(tx_buffer, 0, PAYLOAD_LEN);
    memcpy(tx_buffer, "PING", 4);

    test_printf("[TEST] Simple LoRa test initialized\r\n");
    test_printf("[TEST] Freq=%d SF=%d PWR=%d CR=4/8 Sync=0x1424(Private)\r\n",
                RF_FREQUENCY, LORA_SPREADING_FACTOR, TX_OUTPUT_POWER);

    test_state = TEST_IDLE;
    last_tx_tick = HAL_GetTick();
}

/*============================================================================
 *                              过程处理（主循环）
 *===========================================================================*/
void test_lora_simple_process(void)
{
    if (test_state == TEST_TX_WAIT) {
        // 检查回调标志（ISR只设标志，主循环处理）
        if (tx_done_flag) {
            tx_done_flag = false;
            test_printf("[TEST] #%lu TX SUCCESS! timeout_count=%lu\r\n",
                        (unsigned long)test_tx_count, (unsigned long)test_timeout_count);
            test_state = TEST_IDLE;
            return;
        }
        if (tx_timeout_flag) {
            tx_timeout_flag = false;
            test_timeout_count++;
            test_printf("[TEST] #%lu TX TIMEOUT (total=%lu)\r\n",
                        (unsigned long)test_tx_count, (unsigned long)test_timeout_count);
            test_state = TEST_IDLE;
            return;
        }
        // 应用层超时保护
        if ((HAL_GetTick() - last_tx_tick) > TX_TIMEOUT_VALUE + 2000) {
            test_printf("[TEST] App timeout, resetting\r\n");
            test_state = TEST_IDLE;
        }
        return;
    }

    if (test_state == TEST_IDLE) {
        uint32_t now = HAL_GetTick();
        if ((now - last_tx_tick) >= TX_INTERVAL_MS) {
            // 发送前先清残留标志
            tx_done_flag = false;
            tx_timeout_flag = false;

            last_tx_tick = now;
            test_tx_count++;

            // 参考官方案例：先Sleep再Send
            Radio.Sleep();
            Radio.Send(tx_buffer, PAYLOAD_LEN);

            test_printf("[TEST] #%lu TX start, tick=%lu\r\n",
                        (unsigned long)test_tx_count, (unsigned long)now);
            test_state = TEST_TX_WAIT;
        }
        return;
    }
}
