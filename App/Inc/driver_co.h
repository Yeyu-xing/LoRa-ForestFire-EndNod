/**
 ******************************************************************************
 * @file    driver_co.h
 * @brief   PS-CO-5000 一氧化碳传感器驱动
 * @note    UART 9600 8N1, 9字节帧, 主动上报模式
 *          USART2 与调试串口共用，通过 DMA Circular 接收
 ******************************************************************************
 */

#ifndef DRIVER_CO_H_
#define DRIVER_CO_H_

#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 *                              配置参数
 *===========================================================================*/
#define CO_UART                 huart2
#define CO_BAUDRATE             9600

/* DMA接收缓冲区 (9字节/帧, 64B足够缓冲多帧) */
#define CO_RX_BUF_SIZE          64

/* 传感器预热时间 (ms) */
#define CO_WARMUP_MS            30000

/* 帧格式常量 */
#define CO_FRAME_LEN            9
#define CO_FRAME_HEADER         0xFF
#define CO_GAS_TYPE_CO          0x19
#define CO_UNIT_PPM             0x02

/* 数据超时 (ms) — 超过此时间无有效帧则标记数据无效 */
#define CO_DATA_TIMEOUT_MS      5000

/*============================================================================
 *                              类型定义
 *===========================================================================*/
typedef struct {
    uint16_t concentration;     /* CO浓度 (ppm) */
    uint16_t full_range;        /* 满量程 (ppm) */
    bool     valid;             /* 数据有效 */
    bool     warming_up;        /* 预热中 */
    uint32_t last_update_tick;  /* 最后收到有效帧的时间 */
} co_data_t;

typedef enum {
    CO_STATE_OFF = 0,
    CO_STATE_WARMUP,            /* 传感器预热中 */
    CO_STATE_ACTIVE,            /* 正常工作 */
    CO_STATE_ERROR,
} co_state_t;

/*============================================================================
 *                              接口函数
 *===========================================================================*/
void co_init(void);
bool co_start(void);
void co_stop(void);
void co_process(void);

co_state_t co_get_state(void);
const co_data_t *co_get_data(void);
bool co_is_present(void);

/**
 * @brief  暂停/恢复传感器DMA接收 (调试输出时使用)
 * @note   co_suspend_rx 停止DMA, co_resume_rx 重新启动
 *         配对调用, 不可嵌套
 */
void co_suspend_rx(void);
void co_resume_rx(void);

/**
 * @brief  USART2空闲中断回调 (在stm32wlxx_it.c中调用)
 */
void co_uart_idle_irq(void);

#endif /* DRIVER_CO_H_ */
