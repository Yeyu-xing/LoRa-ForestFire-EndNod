/**
  ******************************************************************************
  * @file    driver_gnss.c
  * @brief   GNSS定位驱动实现 - ATGM336H NMEA解析与UART接收
  * @note    使用USART1 DMA接收 + 空闲中断实现非阻塞NMEA数据接收
  *          解析GGA和RMC语句获取经纬度信息
  ******************************************************************************
  */

#include "driver_gnss.h"
#include "board_io.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

/*============================================================================
 *                              调试串口输出
 *===========================================================================*/
/* GNSS调试开关:
 * GNSS_DEBUG_RAW=1: 输出每条NMEA原始语句 (数据量大, 仅深度调试时开启)
 * GNSS_DEBUG=1:     输出解析结果、状态变化 (推荐保持开启)
 */
#define GNSS_DEBUG_RAW  0
#define GNSS_DEBUG      1

#if GNSS_DEBUG
static void gnss_debug(const char *fmt, ...)
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
#else
#define gnss_debug(fmt, ...) do {} while(0)
#endif

#if GNSS_DEBUG_RAW
static void gnss_debug_raw(const char *fmt, ...)
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
#else
#define gnss_debug_raw(fmt, ...) do {} while(0)
#endif

/*============================================================================
 *                              私有变量
 *===========================================================================*/
static gnss_state_t gnss_state = GNSS_STATE_OFF;
static gnss_location_t gnss_location = {0};

/* DMA接收缓冲区 (Circular模式, 不需要重启) */
static uint8_t gnss_rx_buf[GNSS_RX_BUF_SIZE];
static volatile uint16_t gnss_dma_last_pos = 0;  // 上次处理到的DMA位置
static volatile bool gnss_data_pending = false;   // ISR通知有新数据

/* 状态跟踪 */
static uint32_t gnss_start_tick = 0;
static bool gnss_ever_received = false;     // 是否收到过NMEA数据
static uint32_t gnss_last_data_tick = 0;    // 最后收到数据的时间

/* 行缓冲 (NMEA语句以\r\n结尾，可能跨DMA接收) */
#define NMEA_LINE_BUF_SIZE  128
static char nmea_line[NMEA_LINE_BUF_SIZE];
static uint16_t nmea_line_len = 0;

/* 定位稳定性跟踪 */
static uint8_t  stable_count = 0;           /* 连续稳定的次数 */
static int8_t   last_stable_lat_dir = 0;     /* 上次稳定位置的纬度方向 */
static uint8_t  last_stable_lat_deg = 0;
static uint8_t  last_stable_lat_min = 0;
static uint16_t last_stable_lat_frac = 0;
static int8_t   last_stable_lon_dir = 0;     /* 上次稳定位置的经度方向 */
static uint16_t last_stable_lon_deg = 0;
static uint8_t  last_stable_lon_min = 0;
static uint16_t last_stable_lon_frac = 0;

/*============================================================================
 *                              私有函数 - NMEA解析工具
 *===========================================================================*/
/**
  * @brief  从NMEA字段字符串中提取下一个字段
  * @param  str: 当前位置指针
  * @param  field: 输出字段缓冲区
  * @param  field_size: 缓冲区大小
  * @retval 指向下一个字段的指针, NULL表示结束
  */
static const char *nmea_next_field(const char *str, char *field, uint16_t field_size)
{
    if (str == NULL || *str == '\0') return NULL;

    uint16_t i = 0;
    while (*str != ',' && *str != '*' && *str != '\0' && *str != '\r' && *str != '\n') {
        if (i < field_size - 1) {
            field[i++] = *str;
        }
        str++;
    }
    field[i] = '\0';

    if (*str == ',') {
        return str + 1;    // 跳过逗号，指向下一字段
    }

    return NULL;    // 没有更多字段
}

/**
  * @brief  计算NMEA校验和 (XOR of all chars between $ and *)
  * @param  sentence: NMEA语句 (以$开头)
  * @retval 校验和 (0x00~0xFF)
  */
static uint8_t nmea_checksum(const char *sentence)
{
    uint8_t cksum = 0;

    // 跳过$符号
    const char *p = sentence;
    if (*p == '$') p++;

    while (*p != '*' && *p != '\0' && *p != '\r' && *p != '\n') {
        cksum ^= (uint8_t)*p;
        p++;
    }

    return cksum;
}

/**
  * @brief  验证NMEA语句校验和
  * @retval true: 校验通过
  */
static bool nmea_verify_checksum(const char *sentence)
{
    const char *star = strchr(sentence, '*');
    if (star == NULL) return false;

    uint8_t calc = nmea_checksum(sentence);
    uint8_t recv = (uint8_t)strtol(star + 1, NULL, 16);

    return (calc == recv);
}

/**
  * @brief  解析NMEA纬度字段 "ddmm.mmmm" + 方向 "N/S"
  * @param  lat_str: 纬度字符串, 如 "3113.2456"
  * @param  dir_str: 方向字符串, 如 "N"
  * @param  loc: 位置结构指针
  */
static void parse_latitude(const char *lat_str, const char *dir_str, gnss_location_t *loc)
{
    if (lat_str[0] == '\0') return;

    // NMEA格式: ddmm.mmmm
    // 度: 前两位(或三位，取决于经度/纬度)
    // 找到小数点的位置来确定度分界线
    char buf[16];

    // 提取度 (小数点前的前2位)
    int dot_pos = -1;
    for (int i = 0; lat_str[i] != '\0'; i++) {
        if (lat_str[i] == '.') { dot_pos = i; break; }
    }
    if (dot_pos < 3) return;   // 至少需要 "ddm"

    // 度: 0 ~ dot_pos-2
    int deg_len = dot_pos - 2;
    if (deg_len > 2) deg_len = 2;
    memcpy(buf, lat_str, deg_len);
    buf[deg_len] = '\0';
    loc->lat_deg = (uint8_t)atoi(buf);

    // 分: dot_pos-2 ~ dot_pos (2位整数)
    memcpy(buf, &lat_str[deg_len], 2);
    buf[2] = '\0';
    loc->lat_min = (uint8_t)atoi(buf);

    // 分小数: dot_pos+1 ~ (4位)
    if (dot_pos >= 0 && lat_str[dot_pos + 1] != '\0') {
        const char *frac = &lat_str[dot_pos + 1];
        // 取4位，不足补0
        memset(buf, '0', 4);
        buf[4] = '\0';
        for (int i = 0; i < 4 && frac[i] != '\0'; i++) {
            buf[i] = frac[i];
        }
        loc->lat_frac = (uint16_t)atoi(buf);
    }

    // 方向
    loc->lat_dir = (dir_str[0] == 'S') ? 'S' : 'N';
}

/**
  * @brief  解析NMEA经度字段 "dddmm.mmmm" + 方向 "E/W"
  */
static void parse_longitude(const char *lon_str, const char *dir_str, gnss_location_t *loc)
{
    if (lon_str[0] == '\0') return;

    char buf[16];

    int dot_pos = -1;
    for (int i = 0; lon_str[i] != '\0'; i++) {
        if (lon_str[i] == '.') { dot_pos = i; break; }
    }
    if (dot_pos < 3) return;

    // 经度度: 前dot_pos-2位 (最多3位)
    int deg_len = dot_pos - 2;
    if (deg_len > 3) deg_len = 3;
    memcpy(buf, lon_str, deg_len);
    buf[deg_len] = '\0';
    loc->lon_deg = (uint16_t)atoi(buf);

    // 分
    memcpy(buf, &lon_str[deg_len], 2);
    buf[2] = '\0';
    loc->lon_min = (uint8_t)atoi(buf);

    // 分小数
    if (dot_pos >= 0 && lon_str[dot_pos + 1] != '\0') {
        const char *frac = &lon_str[dot_pos + 1];
        memset(buf, '0', 4);
        buf[4] = '\0';
        for (int i = 0; i < 4 && frac[i] != '\0'; i++) {
            buf[i] = frac[i];
        }
        loc->lon_frac = (uint16_t)atoi(buf);
    }

    loc->lon_dir = (dir_str[0] == 'W') ? 'W' : 'E';
}

/*============================================================================
 *                              私有函数 - NMEA语句处理
 *===========================================================================*/
/**
  * @brief  解析GGA语句
  * @note   $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
  *         字段: 0=GGA, 1=UTC时间, 2=纬度, 3=纬度方向, 4=经度, 5=经度方向,
  *               6=定位质量, 7=卫星数, 8=HDOP, 9=海拔, 10=海拔单位,
  *               11=大地水准面差距, 12=单位, 13=差分站ID
  */
static void parse_gga(const char *sentence)
{
    char field[20];
    const char *p = sentence;

    // 跳过 $GPGGA,
    p = strchr(sentence, ',');
    if (p == NULL) return;
    p++;    // 跳过逗号

    // 字段1: UTC时间 (跳过)
    p = nmea_next_field(p, field, sizeof(field));

    // 字段2: 纬度
    char lat_str[16] = "";
    p = nmea_next_field(p, lat_str, sizeof(lat_str));

    // 字段3: 纬度方向
    char lat_dir[4] = "";
    p = nmea_next_field(p, lat_dir, sizeof(lat_dir));

    // 字段4: 经度
    char lon_str[16] = "";
    p = nmea_next_field(p, lon_str, sizeof(lon_str));

    // 字段5: 经度方向
    char lon_dir[4] = "";
    p = nmea_next_field(p, lon_dir, sizeof(lon_dir));

    // 字段6: 定位质量
    char quality_str[4] = "0";
    p = nmea_next_field(p, quality_str, sizeof(quality_str));
    uint8_t quality = (uint8_t)atoi(quality_str);

    // 字段7: 卫星数
    char sat_str[4] = "0";
    p = nmea_next_field(p, sat_str, sizeof(sat_str));
    uint8_t sats = (uint8_t)atoi(sat_str);

    // 字段8: HDOP (避免使用atof, 手动解析 "x.y" -> x*10+y)
    char hdop_str[8] = "0";
    p = nmea_next_field(p, hdop_str, sizeof(hdop_str));
    uint16_t hdop_x10 = 0;
    {
        int int_part = 0, frac_part = 0;
        char *dot = strchr(hdop_str, '.');
        if (dot != NULL) {
            *dot = '\0';
            int_part = atoi(hdop_str);
            if (dot[1] >= '0' && dot[1] <= '9') frac_part = dot[1] - '0';
        } else {
            int_part = atoi(hdop_str);
        }
        hdop_x10 = (uint16_t)(int_part * 10 + frac_part);
    }

    // 字段9: 海拔 (同样避免atof)
    char alt_str[12] = "0";
    p = nmea_next_field(p, alt_str, sizeof(alt_str));
    uint16_t alt_x10 = 0;
    {
        int int_part = 0, frac_part = 0;
        char *dot = strchr(alt_str, '.');
        if (dot != NULL) {
            *dot = '\0';
            int_part = atoi(alt_str);
            if (dot[1] >= '0' && dot[1] <= '9') frac_part = dot[1] - '0';
        } else {
            int_part = atoi(alt_str);
        }
        alt_x10 = (uint16_t)(int_part * 10 + frac_part);
    }

    // 更新位置数据
    gnss_location.fix_quality = quality;
    gnss_location.satellites = sats;
    gnss_location.hdop_x10 = hdop_x10;
    gnss_location.altitude_x10 = alt_x10;

    if (quality > 0 && lat_str[0] != '\0' && lon_str[0] != '\0') {
        parse_latitude(lat_str, lat_dir, &gnss_location);
        parse_longitude(lon_str, lon_dir, &gnss_location);
        gnss_location.valid = true;
    }

    // 输出GGA解析结果
    gnss_debug("[GNSS] GGA: q=%d sat=%d hdop=%d.%d alt=%d.%d %s %s %s %s\r\n",
               quality, sats, hdop_x10 / 10, hdop_x10 % 10,
               alt_x10 / 10, alt_x10 % 10,
               lat_str, lat_dir, lon_str, lon_dir);
}

/**
  * @brief  解析RMC语句 (补充有效性判断)
  * @note   $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
  *         字段2: A=有效, V=无效
  */
static void parse_rmc(const char *sentence)
{
    char field[20];
    const char *p = sentence;

    p = strchr(sentence, ',');
    if (p == NULL) return;
    p++;

    // 字段1: UTC时间
    p = nmea_next_field(p, field, sizeof(field));

    // 字段2: 状态 A/V
    char status[4] = "V";
    p = nmea_next_field(p, status, sizeof(status));

    if (status[0] != 'A') {
        gnss_location.valid = false;
        gnss_location.fix_quality = 0;
        gnss_debug("[GNSS] RMC: status=%c (invalid)\r\n", status[0]);
    } else {
        gnss_debug("[GNSS] RMC: status=A (valid)\r\n");
    }

    // RMC也包含经纬度，但GGA已经解析，此处仅用状态位
}

/**
  * @brief  检查当前位置是否与上次稳定位置足够接近
  * @note   比较度+分+分小数, 允许的漂移阈值内视为"同一位置"
  * @retval true: 位置稳定 (偏差 < 阈值), false: 漂移过大或方向变化
  */
static bool is_position_stable(void)
{
    /* 必须是有效的方向且度数相同 */
    if (gnss_location.lat_dir != last_stable_lat_dir ||
        gnss_location.lon_dir != last_stable_lon_dir)
        return false;

    if (gnss_location.lat_deg != last_stable_lat_deg ||
        gnss_location.lon_deg != last_stable_lon_deg)
        return false;

    /* 允许分钟差 ±1, 通过分小数补偿计算总漂移 */
    int32_t lat_diff = ((int32_t)gnss_location.lat_min - (int32_t)last_stable_lat_min) * 10000
                     + ((int32_t)gnss_location.lat_frac - (int32_t)last_stable_lat_frac);
    int32_t lon_diff = ((int32_t)gnss_location.lon_min - (int32_t)last_stable_lon_min) * 10000
                     + ((int32_t)gnss_location.lon_frac - (int32_t)last_stable_lon_frac);

    if (lat_diff < 0) lat_diff = -lat_diff;
    if (lon_diff < 0) lon_diff = -lon_diff;

    return (lat_diff <= GNSS_POS_DRIFT_THRESHOLD &&
            lon_diff <= GNSS_POS_DRIFT_THRESHOLD);
}

/**
  * @brief  检查当前位置是否满足基本质量要求 (卫星数 + HDOP)
  * @retval true: 质量合格
  */
static bool is_position_quality_ok(void)
{
    return (gnss_location.satellites >= GNSS_MIN_SATELLITES &&
            gnss_location.hdop_x10 < GNSS_MAX_HDOP_X10);
}

/**
  * @brief  处理一行完整的NMEA语句
  */
static void process_nmea_line(const char *line)
{
    // 验证基本格式
    if (line[0] != '$') return;

    // 输出接收到的原始NMEA语句 (仅GNSS_DEBUG_RAW=1时输出)
    gnss_debug_raw("[GNSS] RX: %s\r\n", line);

    // 校验和验证
    if (!nmea_verify_checksum(line)) {
        gnss_debug("[GNSS] Checksum FAIL: %s\r\n", line);
        return;
    }

    // 根据语句类型分发
    if (strstr(line, "$GNGGA") != NULL || strstr(line, "$GPGGA") != NULL) {
        parse_gga(line);
    } else if (strstr(line, "$GNRMC") != NULL || strstr(line, "$GPRMC") != NULL) {
        parse_rmc(line);
    }
    // 其他语句 (GSA, GSV, VTG等) 暂不处理
}

/**
  * @brief  从DMA接收缓冲区中提取完整的NMEA行
  */
static void process_rx_data(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        char c = (char)data[i];

        if (c == '$') {
            // NMEA语句开始，重置行缓冲
            nmea_line_len = 0;
            nmea_line[nmea_line_len++] = c;
        } else if (c == '\n' || c == '\r') {
            // 行结束，处理完整语句
            if (nmea_line_len > 0 && nmea_line[0] == '$') {
                nmea_line[nmea_line_len] = '\0';
                process_nmea_line(nmea_line);
            }
            nmea_line_len = 0;
        } else {
            // 普通字符，追加到行缓冲
            if (nmea_line_len < NMEA_LINE_BUF_SIZE - 1) {
                nmea_line[nmea_line_len++] = c;
            }
        }
    }
}

/*============================================================================
 *                              公开接口实现
 *===========================================================================*/
void gnss_init(void)
{
    gnss_state = GNSS_STATE_OFF;
    memset(&gnss_location, 0, sizeof(gnss_location));
    gnss_dma_last_pos = 0;
    gnss_data_pending = false;
    gnss_ever_received = false;
    nmea_line_len = 0;
}

bool gnss_start(void)
{
    if (gnss_state != GNSS_STATE_OFF) return false;

    gnss_debug("[GNSS] Starting... (baud=%d, timeout=%ds)\r\n",
               GNSS_BAUDRATE, GNSS_FIX_TIMEOUT_MS / 1000);

    // 清空位置数据
    memset(&gnss_location, 0, sizeof(gnss_location));

    // 重置稳定性跟踪
    stable_count = 0;
    last_stable_lat_dir  = 0;
    last_stable_lat_deg  = 0;
    last_stable_lat_min  = 0;
    last_stable_lat_frac = 0;
    last_stable_lon_dir  = 0;
    last_stable_lon_deg  = 0;
    last_stable_lon_min  = 0;
    last_stable_lon_frac = 0;

    // 使能GNSS模块
    board_pwr_set(PWR_GNSS_EN, true);
    gnss_debug("[GNSS] Module enabled (GNSSEN=HIGH)\r\n");

    // 等待模块上电
    HAL_Delay(GNSS_BOOT_DELAY_MS);

    // 初始化接收状态
    gnss_dma_last_pos = 0;
    gnss_data_pending = false;
    gnss_ever_received = false;
    gnss_last_data_tick = HAL_GetTick();

    // 确保USART1处于可用状态
    if (GNSS_UART.RxState != HAL_UART_STATE_READY) {
        gnss_debug("[GNSS] RxState=%d (not READY), aborting previous RX...\r\n",
                   GNSS_UART.RxState);
        HAL_UART_AbortReceive(&GNSS_UART);
    }
    if (GNSS_UART.gState != HAL_UART_STATE_READY) {
        gnss_debug("[GNSS] GState=%d (not READY), aborting previous TX...\r\n",
                   GNSS_UART.gState);
        HAL_UART_Abort(&GNSS_UART);
    }

    // 重新初始化USART1以确保所有寄存器和状态正确
    gnss_debug("[GNSS] Re-initializing USART1...\r\n");
    HAL_UART_DeInit(&GNSS_UART);
    if (HAL_UART_Init(&GNSS_UART) != HAL_OK) {
        gnss_debug("[GNSS] USART1 re-init FAILED\r\n");
    } else {
        gnss_debug("[GNSS] USART1 re-init OK, RxState=%d, GState=%d\r\n",
                   GNSS_UART.RxState, GNSS_UART.gState);
    }

    // 确保DMA RX通道状态为READY
    if (GNSS_UART.hdmarx != NULL && GNSS_UART.hdmarx->State != HAL_DMA_STATE_READY) {
        gnss_debug("[GNSS] DMA RX State=%d, resetting...\r\n", GNSS_UART.hdmarx->State);
        HAL_DMA_Abort(GNSS_UART.hdmarx);
    }

    // 关闭中断,防止空闲中断在DMA启动前触发
    __disable_irq();

    // 清空缓冲区
    memset(gnss_rx_buf, 0, GNSS_RX_BUF_SIZE);

    // 启动DMA Circular模式接收 (不会自动停止, 不需要重启)
    // 手动设置DMA为Circular模式
    GNSS_UART.hdmarx->Init.Mode = DMA_CIRCULAR;
    HAL_DMA_Init(GNSS_UART.hdmarx);  // 重新初始化DMA通道

    HAL_StatusTypeDef dma_status = HAL_UART_Receive_DMA(&GNSS_UART, gnss_rx_buf, GNSS_RX_BUF_SIZE);

    // DMA启动成功后再使能空闲中断
    if (dma_status == HAL_OK) {
        __HAL_UART_ENABLE_IT(&GNSS_UART, UART_IT_IDLE);
        gnss_dma_last_pos = GNSS_RX_BUF_SIZE;  // CNDTR初始值=BUF_SIZE, pos=0
    }

    __enable_irq();

    if (dma_status != HAL_OK) {
        board_pwr_set(PWR_GNSS_EN, false);
        gnss_state = GNSS_STATE_ERROR;
        gnss_debug("[GNSS] DMA start FAILED: status=%d, RxState=%d, GState=%d, ErrorCode=%lu\r\n",
                   dma_status, GNSS_UART.RxState, GNSS_UART.gState,
                   (unsigned long)GNSS_UART.ErrorCode);
        return false;
    }

    gnss_start_tick = HAL_GetTick();
    gnss_state = GNSS_STATE_BOOTING;
    gnss_debug("[GNSS] DMA Circular RX started, waiting for data...\r\n");

    return true;
}

void gnss_stop(void)
{
    gnss_debug("[GNSS] Stopping module...\r\n");

    // 停止DMA接收
    HAL_UART_AbortReceive(&GNSS_UART);

    // 恢复DMA为Normal模式 (下次vcom_Init可能需要Normal模式)
    if (GNSS_UART.hdmarx != NULL) {
        GNSS_UART.hdmarx->Init.Mode = DMA_NORMAL;
        HAL_DMA_Init(GNSS_UART.hdmarx);
    }

    // 禁用空闲中断
    __HAL_UART_DISABLE_IT(&GNSS_UART, UART_IT_IDLE);

    // 关闭GNSS模块使能
    board_pwr_set(PWR_GNSS_EN, false);
    gnss_debug("[GNSS] Module disabled (GNSSEN=LOW)\r\n");

    gnss_state = GNSS_STATE_OFF;
}

void gnss_process(void)
{
    uint32_t now = HAL_GetTick();

    // 从Circular DMA缓冲区读取新数据
    // CNDTR = DMA通道剩余传输计数 (递减, 0=满, BUF_SIZE=空)
    if (GNSS_UART.hdmarx != NULL) {
        uint16_t dma_pos = GNSS_RX_BUF_SIZE - (uint16_t)GNSS_UART.hdmarx->Instance->CNDTR;
        uint16_t old_pos = gnss_dma_last_pos;

        if (dma_pos != old_pos) {
            gnss_ever_received = true;
            gnss_last_data_tick = now;

            // 从old_pos到dma_pos读取新数据 (可能回绕)
            if (dma_pos > old_pos) {
                // 线性区间: old_pos -> dma_pos
                gnss_debug_raw("[GNSS] RX %d bytes\r\n", dma_pos - old_pos);
                process_rx_data(&gnss_rx_buf[old_pos], dma_pos - old_pos);
            } else {
                // 回绕: old_pos -> end + start -> dma_pos
                gnss_debug_raw("[GNSS] RX %d bytes (wrap)\r\n",
                               (GNSS_RX_BUF_SIZE - old_pos) + dma_pos);
                process_rx_data(&gnss_rx_buf[old_pos], GNSS_RX_BUF_SIZE - old_pos);
                process_rx_data(&gnss_rx_buf[0], dma_pos);
            }

            gnss_dma_last_pos = dma_pos;

            // 收到数据后从BOOTING转为SEARCHING
            if (gnss_state == GNSS_STATE_BOOTING) {
                gnss_state = GNSS_STATE_SEARCHING;
                gnss_debug("[GNSS] State: BOOTING -> SEARCHING (first data received)\r\n");
            }
        }
    }

    // ISR通知有新数据到达 (空闲中断触发, 主循环主动读取即可)
    gnss_data_pending = false;  // 清除标志, 实际处理已在上面完成

    // 状态更新
    switch (gnss_state) {
        case GNSS_STATE_BOOTING:
            // 等待模块发送数据
            if ((now - gnss_start_tick) > 10000 && !gnss_ever_received) {
                gnss_state = GNSS_STATE_SEARCHING;
                gnss_debug("[GNSS] State: BOOTING -> SEARCHING (10s no data, cold start?)\r\n");
            }
            break;

        case GNSS_STATE_SEARCHING:
            // 质量不够, 重置稳定性计数
            if (!is_position_quality_ok()) {
                if (stable_count > 0) {
                    gnss_debug("[GNSS] Quality degraded, reset stable count\r\n");
                    stable_count = 0;
                }
            }
            // 质量合格且位置有效, 检查稳定性
            else if (gnss_location.valid) {
                if (stable_count == 0) {
                    // 首次合格位置, 记录作为参考
                    last_stable_lat_dir  = gnss_location.lat_dir;
                    last_stable_lat_deg  = gnss_location.lat_deg;
                    last_stable_lat_min  = gnss_location.lat_min;
                    last_stable_lat_frac = gnss_location.lat_frac;
                    last_stable_lon_dir  = gnss_location.lon_dir;
                    last_stable_lon_deg  = gnss_location.lon_deg;
                    last_stable_lon_min  = gnss_location.lon_min;
                    last_stable_lon_frac = gnss_location.lon_frac;
                    stable_count = 1;
                    gnss_debug("[GNSS] Stability 1/%d (sat=%d hdop=%d.%d)\r\n",
                               GNSS_STABLE_COUNT,
                               gnss_location.satellites,
                               gnss_location.hdop_x10 / 10,
                               gnss_location.hdop_x10 % 10);
                } else if (is_position_stable()) {
                    stable_count++;
                    gnss_debug("[GNSS] Stability %d/%d (sat=%d hdop=%d.%d)\r\n",
                               stable_count, GNSS_STABLE_COUNT,
                               gnss_location.satellites,
                               gnss_location.hdop_x10 / 10,
                               gnss_location.hdop_x10 % 10);
                    if (stable_count >= GNSS_STABLE_COUNT) {
                        gnss_state = GNSS_STATE_FIXED;
                        gnss_debug("[GNSS] State: SEARCHING -> FIXED\r\n");
                    }
                } else {
                    // 位置漂移, 重置并更新参考点
                    gnss_debug("[GNSS] Position drifted, reset stable count\r\n");
                    stable_count = 0;  // 下一轮重新记录
                }
            }
            // 检查定位超时
            if (gnss_state == GNSS_STATE_SEARCHING &&
                (now - gnss_start_tick) >= GNSS_FIX_TIMEOUT_MS) {
                gnss_state = GNSS_STATE_TIMEOUT;
                gnss_debug("[GNSS] State: SEARCHING -> TIMEOUT (%ds, stable=%d/%d)\r\n",
                           GNSS_FIX_TIMEOUT_MS / 1000,
                           stable_count, GNSS_STABLE_COUNT);
            }
            break;

        case GNSS_STATE_FIXED:
            // 已定位，无需额外处理
            break;

        case GNSS_STATE_TIMEOUT:
        case GNSS_STATE_ERROR:
            // 错误状态，等待外部处理
            break;

        default:
            break;
    }
}

gnss_state_t gnss_get_state(void)
{
    return gnss_state;
}

const gnss_location_t *gnss_get_location(void)
{
    return &gnss_location;
}

bool gnss_is_present(void)
{
    return gnss_ever_received;
}

uint32_t gnss_get_elapsed_ms(void)
{
    if (gnss_state == GNSS_STATE_OFF) return 0;
    return HAL_GetTick() - gnss_start_tick;
}

void gnss_uart_idle_irq(void)
{
    // 检查是否为USART1空闲中断
    if (__HAL_UART_GET_FLAG(&GNSS_UART, UART_FLAG_IDLE) == RESET) return;

    // 清除空闲中断标志
    __HAL_UART_CLEAR_IDLEFLAG(&GNSS_UART);

    // Circular DMA模式: 不需要重启DMA, 只通知主循环有新数据
    // 主循环通过比较CNDTR差值读取新数据
    gnss_data_pending = true;
}
