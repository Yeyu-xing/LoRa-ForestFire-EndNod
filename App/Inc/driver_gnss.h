/**
  ******************************************************************************
  * @file    driver_gnss.h
  * @brief   GNSS定位驱动 - ATGM336H NMEA解析与UART接收
  * @note    ATGM336H-F8N76 默认9600bps, 输出NMEA-0183语句
  *          本驱动解析GGA/RMC获取经纬度，使用DMA+空闲中断接收
  ******************************************************************************
  */

#ifndef DRIVER_GNSS_H_
#define DRIVER_GNSS_H_

#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 *                              配置参数
 *===========================================================================*/
/* GNSS串口 (USART1, PA9/PA10, 115200bps) */
#define GNSS_UART               huart1
#define GNSS_BAUDRATE           115200

/* NMEA接收缓冲区 (DMA Circular模式, 需容纳1Hz NMEA输出~1000字节) */
#define GNSS_RX_BUF_SIZE        512

/* GNSS定位超时 (ms) - 冷启动可能需要30s以上 */
#define GNSS_FIX_TIMEOUT_MS     120000      // 2分钟

/* GNSS模块启动延时 (ms) - 等待模块上电稳定 */
#define GNSS_BOOT_DELAY_MS      500

/*============================================================================
 *                          定位稳定性过滤参数
 *===========================================================================*/
/* 最低卫星数 — 至少6颗才有基本可靠性 */
#define GNSS_MIN_SATELLITES     6

/* HDOP上限 — 水平精度因子 < 2.0 (即 ×10 < 20) */
#define GNSS_MAX_HDOP_X10       20

/* 连续稳定次数 — 连续N次定位结果偏差 < 10m 才标记FIXED (约N秒) */
#define GNSS_STABLE_COUNT       5

/* 位置漂移阈值 (分小数单位, 1单位 ≈ 0.185m, 55 ≈ 10m) */
#define GNSS_POS_DRIFT_THRESHOLD 55

/*============================================================================
 *                              数据结构
 *===========================================================================*/
/* 位置数据 (经纬度用整数存储，避免浮点精度问题)
 * 例如: 纬度 3113.2456N -> lat_deg=31, lat_min=13, lat_frac=2456
 *       经度 12127.8912E -> lon_deg=121, lon_min=27, lon_frac=8912
 */
typedef struct {
    int8_t   lat_dir;           // 纬度方向: 'N' 或 'S'
    uint8_t  lat_deg;           // 纬度度 (0~90)
    uint8_t  lat_min;           // 纬度分 (0~59)
    uint16_t lat_frac;          // 纬度分小数 (0~9999)

    int8_t   lon_dir;           // 经度方向: 'E' 或 'W'
    uint16_t lon_deg;           // 经度度 (0~180)
    uint8_t  lon_min;           // 经度分 (0~59)
    uint16_t lon_frac;          // 经度分小数 (0~9999)

    uint8_t  satellites;        // 参与定位的卫星数
    uint8_t  fix_quality;       // 定位质量: 0=无效, 1=GPS, 2=DGPS
    uint16_t hdop_x10;          // HDOP * 10 (水平精度因子)
    uint16_t altitude_x10;      // 海拔高度 * 10 (米)

    bool     valid;             // 位置数据是否有效
} gnss_location_t;

/* GNSS驱动状态 */
typedef enum {
    GNSS_STATE_OFF = 0,     // 关闭
    GNSS_STATE_BOOTING,     // 启动中(等待模块稳定)
    GNSS_STATE_SEARCHING,   // 搜星中(等待定位)
    GNSS_STATE_FIXED,       // 已定位
    GNSS_STATE_TIMEOUT,     // 定位超时
    GNSS_STATE_ERROR,       // 错误
} gnss_state_t;

/*============================================================================
 *                              接口函数
 *===========================================================================*/
/**
  * @brief  初始化GNSS驱动 (不启动模块)
  */
void gnss_init(void);

/**
  * @brief  启动GNSS模块 (使能电源, 开始DMA接收)
  * @retval true: 启动成功
  */
bool gnss_start(void);

/**
  * @brief  停止GNSS模块 (关闭电源, 停止DMA接收)
  */
void gnss_stop(void);

/**
  * @brief  GNSS周期处理 (在主循环中调用)
  * @note   处理DMA空闲中断接收的数据，解析NMEA
  */
void gnss_process(void);

/**
  * @brief  获取当前GNSS状态
  */
gnss_state_t gnss_get_state(void);

/**
  * @brief  获取定位结果 (只读)
  * @retval 位置数据指针, valid字段指示是否有效
  */
const gnss_location_t *gnss_get_location(void);

/**
  * @brief  检查GNSS模块是否在线 (是否收到NMEA数据)
  * @retval true: 收到过数据
  */
bool gnss_is_present(void);

/**
  * @brief  获取GNSS启动后的经过时间 (ms)
  * @retval 经过时间, 0表示未启动
  */
uint32_t gnss_get_elapsed_ms(void);

/**
  * @brief  USART1空闲中断回调 (在stm32wlxx_it.c中调用)
  * @note   处理DMA接收完成
  */
void gnss_uart_idle_irq(void);

#endif /* DRIVER_GNSS_H_ */
