/**
 * @file    test_lora_rx.c
 * @brief   LoRa连续接收测试 - 纯RX模式，连续监听
 */
#include "test_lora_rx.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "radio.h"
#include "radio_driver.h"
#include "usart.h"

/*============================================================================
 *                              参数定义
 *===========================================================================*/
#define RF_FREQUENCY            471500000   // 471.5 MHz
#define TX_OUTPUT_POWER         14
#define LORA_BANDWIDTH          0           // 0: 125kHz
#define LORA_SPREADING_FACTOR   7           // SF7
#define LORA_CODINGRATE         4           // 4/8
#define LORA_PREAMBLE_LENGTH    8
#define LORA_SYMBOL_TIMEOUT     5
#define LORA_IQ_INVERSION_ON    false
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define MAX_BUF_SIZE            255

/*============================================================================
 *                              标志
 *===========================================================================*/
static volatile bool rx_done_flag = false;
static RadioEvents_t radio_events;

static uint8_t rx_buffer[MAX_BUF_SIZE];
static uint16_t rx_size = 0;
static int16_t rx_rssi = 0;
static int8_t rx_snr = 0;

/*============================================================================
 *                              UART调试输出
 *===========================================================================*/
static void test_printf(const char *fmt, ...)
{
    char buf[192];
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
 *===========================================================================*/
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    if (size <= MAX_BUF_SIZE) {
        memcpy(rx_buffer, payload, size);
        rx_size = size;
        rx_rssi = rssi;
        rx_snr = snr;
    }
    rx_done_flag = true;
}

static void OnRxTimeout(void) {}
static void OnRxError(void)   {}
static void OnTxDone(void)    {}
static void OnTxTimeout(void) {}

/*============================================================================
 *                              初始化
 *===========================================================================*/
void test_lora_rx_init(void)
{
    radio_events.TxDone    = OnTxDone;
    radio_events.TxTimeout = OnTxTimeout;
    radio_events.RxDone    = OnRxDone;
    radio_events.RxTimeout = OnRxTimeout;
    radio_events.RxError   = OnRxError;

    Radio.Init(&radio_events);
    Radio.SetChannel(RF_FREQUENCY);

    Radio.SetRxConfig(MODEM_LORA,
                      LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE,
                      0,
                      LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT,
                      LORA_FIX_LENGTH_PAYLOAD_ON,
                      0,
                      true,       // crcOn
                      false,      // freqHopOn
                      0,
                      LORA_IQ_INVERSION_ON,
                      true);      // rxContinuous = true → 连续接收，永不超时

    Radio.SetTxConfig(MODEM_LORA,
                      TX_OUTPUT_POWER, 0,
                      LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
                      LORA_FIX_LENGTH_PAYLOAD_ON, true, false, 0,
                      LORA_IQ_INVERSION_ON, 5000);

    Radio.SetMaxPayloadLength(MODEM_LORA, MAX_BUF_SIZE);
    Radio.SetPublicNetwork(false);

    test_printf("\r\n=== LoRa RX Test ===\r\n");
    test_printf("Freq=%d MHz  SF=%d  BW=125k  SyncWord=0x1424\r\n",
                RF_FREQUENCY / 1000000, LORA_SPREADING_FACTOR);

    /* 进入连续接收 */
    Radio.Rx(0);
    test_printf("[TEST_RX] Listening (continuous)...\r\n");
}

/*============================================================================
 *                              主循环处理
 *===========================================================================*/
void test_lora_rx_process(void)
{
    if (rx_done_flag) {
        rx_done_flag = false;

        if (rx_size == 0) {
            /* 连续模式下 radio 自动保持 RX */
            return;
        }

        test_printf("\r\n>>> PACKET RECEIVED <<<\r\n");
        test_printf("Size=%d  RSSI=%d  SNR=%d\r\n", rx_size, (int)rx_rssi, (int)rx_snr);

        test_printf("HEX:");
        for (uint16_t i = 0; i < rx_size && i < 32; i++)
            test_printf(" %02X", rx_buffer[i]);
        test_printf("\r\n");

        rx_size = 0;
        /* 连续模式下 radio 自动保持 RX，无需重进 */
    }
}