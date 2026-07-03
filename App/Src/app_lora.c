/**
 ******************************************************************************
 * @file    app_lora.c
 * @brief   LoRa Radio 硬件抽象层实现
 ******************************************************************************
 */

#include "app_lora.h"

#include <string.h>
#include <stdio.h>

#include "radio.h"
#include "radio_driver.h"
#include "usart.h"
#include "stm32wlxx_hal.h"

/*============================================================================
 *                              私有类型
 *===========================================================================*/
typedef enum {
    TX_IDLE = 0,
    TX_RUNNING,
    TX_DONE,
    TX_TIMEOUT,
} tx_state_t;

typedef enum {
    RX_IDLE = 0,
    RX_RUNNING,
    RX_DONE,
    RX_TIMEOUT,
    RX_ERROR,
} rx_state_t;

/*============================================================================
 *                              私有变量
 *===========================================================================*/
static bool lora_initialized = false;
static tx_state_t tx_state = TX_IDLE;
static rx_state_t rx_state = RX_IDLE;

static volatile bool irq_tx_done = false;
static volatile bool irq_tx_timeout = false;
static volatile bool irq_rx_done = false;
static volatile bool irq_rx_timeout = false;
static volatile bool irq_rx_error = false;

/* 接收缓冲区 */
static uint8_t rx_buffer[LORA_RX_BUF_SIZE];
static uint16_t rx_len = 0;
static int16_t rx_rssi = 0;
static int8_t rx_snr = 0;

/* Radio事件回调 */
static RadioEvents_t radio_events;

/*============================================================================
 *                              私有函数 - UART调试
 *===========================================================================*/
static void debug_print(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
}

/*============================================================================
 *                              私有函数 - Radio回调
 *===========================================================================*/
static void on_tx_done(void)
{
    irq_tx_done = true;
}

static void on_tx_timeout(void)
{
    irq_tx_timeout = true;
}

static void on_rx_done(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
    if (size <= LORA_RX_BUF_SIZE) {
        memcpy(rx_buffer, payload, size);
        rx_len = size;
        rx_rssi = rssi;
        rx_snr = LoraSnr_FskCfo;
    }
    irq_rx_done = true;
}

static void on_rx_timeout(void)
{
    irq_rx_timeout = true;
}

static void on_rx_error(void)
{
    irq_rx_error = true;
}

/*============================================================================
 *                              私有函数 - Radio配置
 *===========================================================================*/
static void do_configure(uint8_t sf, int8_t power_dbm)
{
    Radio.SetChannel(LORA_RF_FREQUENCY);
    Radio.SetModem(MODEM_LORA);

    Radio.SetRxConfig(MODEM_LORA,
                      LORA_BANDWIDTH,
                      sf,
                      LORA_CODINGRATE,
                      0,
                      LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT,
                      false,      // fixLen
                      0,          // payloadLen
                      true,       // crcOn
                      false,      // freqHopOn
                      0,          // hopPeriod
                      false,      // iqInverted
                      true);      // rxContinuous ← 连续接收模式

    Radio.SetTxConfig(MODEM_LORA,
                      power_dbm,
                      0,          // fdev
                      LORA_BANDWIDTH,
                      sf,
                      LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH,
                      false,      // fixLen
                      true,       // crcOn
                      false,      // freqHopOn
                      0,          // hopPeriod
                      false,      // iqInverted
                      LORA_TX_TIMEOUT_MS);

    Radio.SetMaxPayloadLength(MODEM_LORA, LORA_RX_BUF_SIZE);
    Radio.SetPublicNetwork(false);
}

/*============================================================================
 *                              公开接口
 *===========================================================================*/
bool app_lora_init(void)
{
    radio_events.TxDone    = on_tx_done;
    radio_events.TxTimeout = on_tx_timeout;
    radio_events.RxDone    = on_rx_done;
    radio_events.RxTimeout = on_rx_timeout;
    radio_events.RxError   = on_rx_error;

    Radio.Init(&radio_events);

    do_configure(7, 14);

    Radio.Standby();

    RadioPhyStatus_t status = SUBGRF_GetStatus();
    if (status.Fields.ChipMode != 2) {
        debug_print("[LORA] Init FAILED\r\n");
        return false;
    }

    lora_initialized = true;
    irq_tx_done = false;
    irq_tx_timeout = false;
    irq_rx_done = false;
    irq_rx_timeout = false;
    irq_rx_error = false;
    tx_state = TX_IDLE;
    rx_state = RX_IDLE;
    rx_len = 0;

    debug_print("[LORA] Init OK\r\n");
    return true;
}

void app_lora_configure(uint8_t sf, int8_t power_dbm)
{
    if (!lora_initialized) return;

    Radio.Standby();
    do_configure(sf, power_dbm);

    char msg[64];
    snprintf(msg, sizeof(msg), "[LORA] Config SF=%d PWR=%ddBm\r\n", sf, (int)power_dbm);
    debug_print(msg);
}

bool app_lora_send(const uint8_t *buf, uint16_t len)
{
    if (!lora_initialized || buf == NULL || len == 0) return false;
    if (tx_state == TX_RUNNING) return false;

    irq_tx_done = false;
    irq_tx_timeout = false;
    tx_state = TX_RUNNING;

    Radio.Sleep();
    radio_status_t status = Radio.Send((uint8_t *)buf, (uint8_t)len);
    if (status != RADIO_STATUS_OK) {
        tx_state = TX_IDLE;
        debug_print("[LORA] Send failed\r\n");
        return false;
    }

    {
        char msg[64];
        snprintf(msg, sizeof(msg), "[LORA] TX len=%d\r\n", len);
        debug_print(msg);
    }
    return true;
}

bool app_lora_tx_done(void)
{
    if (irq_tx_done) {
        irq_tx_done = false;
        tx_state = TX_DONE;
        return true;
    }
    if (irq_tx_timeout) {
        irq_tx_timeout = false;
        tx_state = TX_TIMEOUT;
        return true;
    }
    return (tx_state == TX_DONE || tx_state == TX_TIMEOUT);
}

bool app_lora_tx_success(void)
{
    return (tx_state == TX_DONE);
}

void app_lora_reset_tx(void)
{
    irq_tx_done = false;
    irq_tx_timeout = false;
    tx_state = TX_RUNNING;
}

bool app_lora_start_rx(uint32_t timeout_ms)
{
    if (!lora_initialized) return false;
    if (rx_state == RX_RUNNING) return false;

    irq_rx_done = false;
    irq_rx_timeout = false;
    irq_rx_error = false;
    rx_len = 0;
    rx_state = RX_RUNNING;

    Radio.Rx(timeout_ms);
    return true;
}

bool app_lora_start_scan_rx(uint8_t sf)
{
    if (!lora_initialized) return false;

    /* Stop any ongoing RX, then reconfigure for continuous mode */
    Radio.Standby();

    Radio.SetRxConfig(MODEM_LORA,
                      LORA_BANDWIDTH,
                      sf,
                      LORA_CODINGRATE,
                      0,
                      LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT,
                      false,      // fixLen
                      0,          // payloadLen
                      true,       // crcOn
                      false,      // freqHopOn
                      0,          // hopPeriod
                      false,      // iqInverted
                      true);      // rxContinuous = true  (与test_lora_rx一致)

    irq_rx_done = false;
    irq_rx_timeout = false;
    irq_rx_error = false;
    rx_len = 0;
    rx_state = RX_RUNNING;

    Radio.Rx(0);  /* 0 + rxContinuous=true → 持续接收, 永不超时 */
    return true;
}

bool app_lora_rx_available(void)
{
    if (irq_rx_done) {
        irq_rx_done = false;
        rx_state = RX_DONE;
        return true;
    }
    return (rx_state == RX_DONE);
}

uint16_t app_lora_get_rx_data(uint8_t *buf, uint16_t buf_size,
                               int16_t *rssi, int8_t *snr)
{
    if (rx_state != RX_DONE || buf == NULL) return 0;
    uint16_t copy_len = (rx_len < buf_size) ? rx_len : buf_size;
    memcpy(buf, rx_buffer, copy_len);
    if (rssi) *rssi = rx_rssi;
    if (snr)  *snr  = rx_snr;
    rx_state = RX_IDLE;
    return copy_len;
}

bool app_lora_rx_finished(void)
{
    if (irq_rx_timeout) {
        irq_rx_timeout = false;
        rx_state = RX_TIMEOUT;
        return true;
    }
    if (irq_rx_error) {
        irq_rx_error = false;
        rx_state = RX_ERROR;
        return true;
    }
    return (rx_state != RX_RUNNING);
}

void app_lora_stop_rx(void)
{
    irq_rx_done = false;
    irq_rx_timeout = false;
    irq_rx_error = false;
    rx_state = RX_IDLE;
    Radio.Standby();
}

bool app_lora_is_ready(void)
{
    return lora_initialized;
}

int8_t app_lora_get_sensitivity(uint8_t sf)
{
    switch (sf) {
        case 7:  return LORA_SF_SENSITIVITY_7;
        case 8:  return LORA_SF_SENSITIVITY_8;
        case 9:  return LORA_SF_SENSITIVITY_9;
        case 10: return LORA_SF_SENSITIVITY_10;
        case 11: return LORA_SF_SENSITIVITY_11;
        case 12: return LORA_SF_SENSITIVITY_12;
        default: return -120;
    }
}
