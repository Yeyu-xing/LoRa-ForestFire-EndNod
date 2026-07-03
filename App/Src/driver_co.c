/**
 ******************************************************************************
 * @file    driver_co.c
 * @brief   PS-CO-5000 一氧化碳传感器驱动实现
 * @note    UART 9600 8N1, 9字节帧, DMA Circular 接收
 *          主动上报模式, 协议见规格书第4-5页
 ******************************************************************************
 */

#include "driver_co.h"
#include "board_io.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/*============================================================================
 *                              调试开关
 *===========================================================================*/
#define CO_DEBUG    0

#if CO_DEBUG
static void co_debug(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0) {
        if ((uint32_t)len >= sizeof(buf)) len = sizeof(buf) - 1;
        /* 直接用阻塞发送, 调试信息量小不会显著影响传感器接收 */
        HAL_UART_Transmit(&CO_UART, (uint8_t *)buf, len, 100);
    }
}
#else
#define co_debug(fmt, ...) do {} while(0)
#endif

/*============================================================================
 *                              私有变量
 *===========================================================================*/
static co_state_t co_state = CO_STATE_OFF;
static co_data_t co_data = {0};

/* DMA接收缓冲区 (Circular模式) */
static uint8_t co_rx_buf[CO_RX_BUF_SIZE];
static volatile uint16_t co_dma_last_pos = 0;
static volatile bool co_data_pending = false;

/* 帧组装缓冲 — 从DMA字节流中提取9字节帧 */
static uint8_t frame_buf[CO_FRAME_LEN];
static uint8_t frame_pos = 0;
static bool frame_synced = false;

/* 状态跟踪 */
static uint32_t co_start_tick = 0;
static bool co_ever_received = false;

/* DMA暂停标志 (调试输出时使用) */
static volatile bool co_rx_suspended = false;

/*============================================================================
 *                              私有函数 - 校验和
 *===========================================================================*/

/**
 * @brief  计算PS-CO-5000校验和
 * @note   对buf[1]~buf[7]求和, 取反加1
 *         对应规格书中的 FucCheckSum 函数
 */
static uint8_t co_calc_checksum(const uint8_t *frame)
{
    uint16_t sum = 0;
    for (uint8_t i = 1; i <= 7; i++) {
        sum += frame[i];
    }
    return (uint8_t)((~sum) + 1);
}

/*============================================================================
 *                              私有函数 - 帧解析
 *===========================================================================*/

/**
 * @brief  解析一条完整帧 (9字节, 已通过帧头同步和校验)
 * @note   区分主动上报帧、查询应答帧、校准应答帧
 */
static void co_parse_frame(const uint8_t *frame)
{
    uint8_t cmd = frame[1];

    /* ─── 主动上报: FF 19 02 00 HH LL FB FR CS ─── */
    if (cmd == CO_GAS_TYPE_CO && frame[2] == CO_UNIT_PPM) {
        /* HH=frame[4], LL=frame[5], FullRange=frame[6]*256+frame[7] */
        uint16_t ppm = ((uint16_t)frame[4] << 8) | frame[5];
        uint16_t range = ((uint16_t)frame[6] << 8) | frame[7];

        co_data.concentration  = ppm;
        co_data.full_range     = range;
        co_data.valid          = true;
        co_data.last_update_tick = HAL_GetTick();

        co_debug("[CO] %u ppm (range=%u)\r\n", ppm, range);
        return;
    }

    /* ─── 查询应答: FF 86 00 00 00 00 HH LL CS ─── */
    if (cmd == 0x86) {
        uint16_t ppm = ((uint16_t)frame[6] << 8) | frame[7];
        co_data.concentration  = ppm;
        co_data.valid          = true;
        co_data.last_update_tick = HAL_GetTick();

        co_debug("[CO] Query resp: %u ppm\r\n", ppm);
        return;
    }

    /* 其他帧 (校准应答等) 暂不处理 */
    co_debug("[CO] Unknown frame: cmd=0x%02X\r\n", cmd);
}

/**
 * @brief  从DMA缓冲区中提取并解析9字节帧
 * @note   使用状态机搜索0xFF帧头, 收集9字节后校验
 */
static void co_process_bytes(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint8_t byte = data[i];

        if (!frame_synced) {
            /* 搜索帧头 0xFF */
            if (byte == CO_FRAME_HEADER) {
                frame_buf[0] = byte;
                frame_pos = 1;
                frame_synced = true;
            }
            continue;
        }

        /* 已同步, 收集后续字节 */
        frame_buf[frame_pos] = byte;
        frame_pos++;

        if (frame_pos >= CO_FRAME_LEN) {
            /* 收满9字节, 校验 */
            frame_synced = false;
            frame_pos = 0;
            co_ever_received = true;

            if (co_calc_checksum(frame_buf) == frame_buf[8]) {
                co_parse_frame(frame_buf);
            } else {
                co_debug("[CO] Checksum FAIL:"
                         " %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                         frame_buf[0], frame_buf[1], frame_buf[2],
                         frame_buf[3], frame_buf[4], frame_buf[5],
                         frame_buf[6], frame_buf[7], frame_buf[8]);
            }
        }
    }
}

/*============================================================================
 *                              公开接口
 *===========================================================================*/

void co_init(void)
{
    co_state = CO_STATE_OFF;
    memset(&co_data, 0, sizeof(co_data));
    co_dma_last_pos = 0;
    co_data_pending = false;
    co_ever_received = false;
    co_rx_suspended = false;
    frame_pos = 0;
    frame_synced = false;
}

bool co_start(void)
{
    if (co_state != CO_STATE_OFF) return false;

    co_debug("[CO] Starting... (baud=%d, warmup=%ds)\r\n",
             CO_BAUDRATE, CO_WARMUP_MS / 1000);

    /* 清空数据 */
    memset(&co_data, 0, sizeof(co_data));
    co_data.warming_up = true;
    co_ever_received = false;
    frame_pos = 0;
    frame_synced = false;

    /* 使能5V电源 */
    board_pwr_set(PWR_5V_EN, true);

    /* 等待模块上电稳定 */
    HAL_Delay(500);

    /* 确保USART2可用 */
    if (CO_UART.RxState != HAL_UART_STATE_READY) {
        co_debug("[CO] USART2 RxState=%d, aborting...\r\n", CO_UART.RxState);
        HAL_UART_AbortReceive(&CO_UART);
    }
    if (CO_UART.gState != HAL_UART_STATE_READY) {
        co_debug("[CO] USART2 GState=%d, aborting...\r\n", CO_UART.gState);
        HAL_UART_Abort(&CO_UART);
    }

    /* 重新初始化USART2 (可能之前被其他波特率使用) */
    HAL_UART_DeInit(&CO_UART);
    if (HAL_UART_Init(&CO_UART) != HAL_OK) {
        co_debug("[CO] USART2 re-init FAILED\r\n");
        board_pwr_set(PWR_5V_EN, false);
        co_state = CO_STATE_ERROR;
        return false;
    }

    /* 确保DMA为Circular模式 */
    if (CO_UART.hdmarx != NULL && CO_UART.hdmarx->State != HAL_DMA_STATE_READY) {
        co_debug("[CO] DMA RX State=%d, resetting...\r\n", CO_UART.hdmarx->State);
        HAL_DMA_Abort(CO_UART.hdmarx);
    }

    __disable_irq();
    memset(co_rx_buf, 0, CO_RX_BUF_SIZE);

    if (CO_UART.hdmarx != NULL) {
        CO_UART.hdmarx->Init.Mode = DMA_CIRCULAR;
        HAL_DMA_Init(CO_UART.hdmarx);
    }

    HAL_StatusTypeDef dma_status = HAL_UART_Receive_DMA(&CO_UART, co_rx_buf, CO_RX_BUF_SIZE);

    if (dma_status == HAL_OK) {
        __HAL_UART_ENABLE_IT(&CO_UART, UART_IT_IDLE);
        co_dma_last_pos = CO_RX_BUF_SIZE;
    }

    __enable_irq();

    if (dma_status != HAL_OK) {
        board_pwr_set(PWR_5V_EN, false);
        co_state = CO_STATE_ERROR;
        co_debug("[CO] DMA start FAILED: status=%d\r\n", dma_status);
        return false;
    }

    co_start_tick = HAL_GetTick();
    co_state = CO_STATE_WARMUP;
    co_rx_suspended = false;
    co_debug("[CO] DMA Circular RX started\r\n");

    return true;
}

void co_stop(void)
{
    co_debug("[CO] Stopping...\r\n");

    HAL_UART_AbortReceive(&CO_UART);

    if (CO_UART.hdmarx != NULL) {
        CO_UART.hdmarx->Init.Mode = DMA_NORMAL;
        HAL_DMA_Init(CO_UART.hdmarx);
    }

    __HAL_UART_DISABLE_IT(&CO_UART, UART_IT_IDLE);

    board_pwr_set(PWR_5V_EN, false);
    co_debug("[CO] Power off\r\n");

    co_state = CO_STATE_OFF;
    co_rx_suspended = false;
}

void co_process(void)
{
    uint32_t now = HAL_GetTick();

    if (co_rx_suspended) return;

    /* 从Circular DMA缓冲区读取新数据 */
    if (CO_UART.hdmarx != NULL && co_state != CO_STATE_OFF) {
        uint16_t dma_pos = CO_RX_BUF_SIZE -
            (uint16_t)CO_UART.hdmarx->Instance->CNDTR;
        uint16_t old_pos = co_dma_last_pos;

        if (dma_pos != old_pos) {
            if (dma_pos > old_pos) {
                co_process_bytes(&co_rx_buf[old_pos], dma_pos - old_pos);
            } else {
                co_process_bytes(&co_rx_buf[old_pos], CO_RX_BUF_SIZE - old_pos);
                co_process_bytes(&co_rx_buf[0], dma_pos);
            }
            co_dma_last_pos = dma_pos;
        }
    }

    co_data_pending = false;

    /* 预热超时检查 */
    if (co_state == CO_STATE_WARMUP) {
        if ((now - co_start_tick) >= CO_WARMUP_MS) {
            co_state = CO_STATE_ACTIVE;
            co_data.warming_up = false;
            co_debug("[CO] Warmup complete → ACTIVE\r\n");
        }
    }

    /* 数据超时检查 (仅ACTIVE状态) */
    if (co_state == CO_STATE_ACTIVE && co_data.valid) {
        if ((now - co_data.last_update_tick) >= CO_DATA_TIMEOUT_MS) {
            co_data.valid = false;
            co_debug("[CO] Data timeout (%ds)\r\n", CO_DATA_TIMEOUT_MS / 1000);
        }
    }
}

co_state_t co_get_state(void)
{
    return co_state;
}

const co_data_t *co_get_data(void)
{
    return &co_data;
}

bool co_is_present(void)
{
    return co_ever_received;
}

/*============================================================================
 *                              调试输出分时复用
 *===========================================================================*/

void co_suspend_rx(void)
{
    if (co_state == CO_STATE_OFF || co_state == CO_STATE_ERROR) return;
    if (co_rx_suspended) return;

    __disable_irq();
    HAL_UART_AbortReceive(&CO_UART);
    co_rx_suspended = true;
    __enable_irq();
}

void co_resume_rx(void)
{
    if (!co_rx_suspended) return;
    if (co_state == CO_STATE_OFF || co_state == CO_STATE_ERROR) return;

    __disable_irq();

    /* 记录当前DMA位置 (abort后CNDTR被重置为BUF_SIZE) */
    co_dma_last_pos = CO_RX_BUF_SIZE;

    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(&CO_UART, co_rx_buf, CO_RX_BUF_SIZE);
    if (status == HAL_OK) {
        __HAL_UART_ENABLE_IT(&CO_UART, UART_IT_IDLE);
    }

    co_rx_suspended = false;
    __enable_irq();
}

/*============================================================================
 *                              中断回调
 *===========================================================================*/

void co_uart_idle_irq(void)
{
    if (co_rx_suspended) return;
    if (__HAL_UART_GET_FLAG(&CO_UART, UART_FLAG_IDLE) == RESET) return;

    __HAL_UART_CLEAR_IDLEFLAG(&CO_UART);
    co_data_pending = true;
}
