/**
 ******************************************************************************
 * @file    app_lora.h
 * @brief   LoRa Radio 硬件抽象层 - 仅负责射频收发，不包含协议
 ******************************************************************************
 */

#ifndef APP_LORA_H_
#define APP_LORA_H_

#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 *                              LoRa射频参数
 *===========================================================================*/
#define LORA_RF_FREQUENCY       471500000   // 471.5 MHz (CN470)
#define LORA_BANDWIDTH          0           // 0: 125kHz
#define LORA_CODINGRATE         4           // 4/8
#define LORA_PREAMBLE_LENGTH    8
#define LORA_SYMBOL_TIMEOUT     5
#define LORA_TX_TIMEOUT_MS      5000
#define LORA_RX_BUF_SIZE        128         // 接收缓冲区

/* 主/备用SF灵敏度 (dBm, 典型值) */
#define LORA_SF_SENSITIVITY_7   (-123)
#define LORA_SF_SENSITIVITY_8   (-126)
#define LORA_SF_SENSITIVITY_9   (-129)
#define LORA_SF_SENSITIVITY_10  (-132)
#define LORA_SF_SENSITIVITY_11  (-134)
#define LORA_SF_SENSITIVITY_12  (-137)

/*============================================================================
 *                              接口函数
 *===========================================================================*/

/**
 * @brief  初始化LoRa Radio硬件
 * @retval true: 成功
 */
bool app_lora_init(void);

/**
 * @brief  配置Radio参数 (SF和功率)
 * @param  sf: 扩频因子 7-12
 * @param  power_dbm: 发射功率 (dBm)
 */
void app_lora_configure(uint8_t sf, int8_t power_dbm);

/**
 * @brief  发送数据帧 (非阻塞)
 * @param  buf: 帧数据 (不含CRC，硬件自动添加)
 * @param  len: 帧长度
 * @retval true: 已提交发送
 */
bool app_lora_send(const uint8_t *buf, uint16_t len);

/**
 * @brief  检查发送是否完成
 * @retval true: 发送完成 (成功或超时)
 */
bool app_lora_tx_done(void);

/**
 * @brief  检查发送是否成功
 * @retval true: TxDone, false: TxTimeout
 */
bool app_lora_tx_success(void);

/**
 * @brief  复位发送状态为RUNNING (用于忽略虚假超时)
 */
void app_lora_reset_tx(void);

/**
 * @brief  开启接收窗口 (非阻塞)
 * @param  timeout_ms: 接收超时(ms), 0=连续接收
 * @retval true: 已开启接收
 */
bool app_lora_start_rx(uint32_t timeout_ms);

/**
 * @brief  检查是否收到数据
 * @retval true: 有数据可读
 */
bool app_lora_rx_available(void);

/**
 * @brief  获取接收数据
 * @param  buf: 输出缓冲区
 * @param  buf_size: 缓冲区大小
 * @param  rssi: 输出RSSI (dBm)
 * @param  snr: 输出SNR (0.25dB步进)
 * @retval 接收数据长度, 0=无数据
 */
uint16_t app_lora_get_rx_data(uint8_t *buf, uint16_t buf_size,
                               int16_t *rssi, int8_t *snr);

/**
 * @brief  检查接收窗口是否已结束
 * @retval true: 已结束 (超时、完成或错误)
 */
bool app_lora_rx_finished(void);

/**
 * @brief  开启扫描接收窗口 (连续模式, rxContinuous=true)
 * @param  sf: 扩频因子 7-12
 * @retval true: 已开启连续接收
 */
bool app_lora_start_scan_rx(uint8_t sf);

/**
 * @brief  停止接收, Radio回Standby
 */
void app_lora_stop_rx(void);

/**
 * @brief  获取LoRa模块是否就绪
 */
bool app_lora_is_ready(void);

/**
 * @brief  获取当前SF灵敏度
 * @param  sf: 扩频因子
 * @retval 灵敏度 (dBm)
 */
int8_t app_lora_get_sensitivity(uint8_t sf);

#endif /* APP_LORA_H_ */
