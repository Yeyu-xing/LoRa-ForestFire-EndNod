/**
 ******************************************************************************
 * @file    lora_protocol.c
 * @brief   LoRa私有通信协议模块实现
 * @note    纯协议逻辑, 操作内存缓冲区, 不依赖硬件
 ******************************************************************************
 */

#include "lora_protocol.h"
#include <string.h>

/*============================================================================
 *                              重传队列 (内存FIFO)
 *===========================================================================*/
static proto_retransmit_entry_t retransmit_queue[PROTO_RETRANSMIT_QUEUE_SIZE];
static uint16_t retransmit_head = 0;           /* 队头 (最早写入) */
static uint16_t retransmit_tail = 0;           /* 队尾 (下次写入位置) */
static uint16_t retransmit_count = 0;          /* 当前条目数 */

/*============================================================================
 *                              Beacon 候选列表
 *===========================================================================*/
static proto_beacon_candidate_t candidates[PROTO_MAX_CANDIDATES];
static uint8_t candidate_count = 0;
static bool candidate_excluded[PROTO_MAX_CANDIDATES];  /* 注册失败时标记排除 */

/*============================================================================
 *                              高层接口状态
 *===========================================================================*/
static proto_ack_info_t last_ack;              /* 最新ACK信息缓存 */
static uint16_t assigned_short_addr = PROTO_ADDR_BROADCAST;  /* 注册分配的短地址 */

/*============================================================================
 *                              FrameControl 编解码
 *===========================================================================*/

/* 组合FrameControl字节 */
uint8_t proto_make_fc(uint8_t frame_type, uint8_t ack_type,
                      uint8_t sf_flag, uint8_t power_level)
{
    return (frame_type & PROTO_FRAME_TYPE_MASK) |
           (ack_type & PROTO_ACK_TYPE_MASK) |
           (sf_flag & PROTO_SF_FLAG_MASK) |
           (power_level & PROTO_POWER_LEVEL_MASK);
}

/* 解析FrameControl字节 */
void proto_parse_fc(uint8_t fc, uint8_t *frame_type, uint8_t *ack_type,
                    uint8_t *sf_flag, uint8_t *power_level)
{
    if (frame_type)  *frame_type  = fc & PROTO_FRAME_TYPE_MASK;
    if (ack_type)    *ack_type    = fc & PROTO_ACK_TYPE_MASK;
    if (sf_flag)     *sf_flag     = fc & PROTO_SF_FLAG_MASK;
    if (power_level) *power_level = fc & PROTO_POWER_LEVEL_MASK;
}

/*============================================================================
 *                              帧构建与解析
 *===========================================================================*/

/* 构建完整发送帧: [FC(1)][Dest(2)][Sender(2)][Seq(2)][TLV...], CRC由硬件追加 */
uint16_t proto_build_frame(uint8_t *buf, uint16_t buf_size,
                           uint8_t fc, uint16_t dest, uint16_t sender, uint16_t seq,
                           const uint8_t *tlv_data, uint8_t tlv_len)
{
    uint16_t frame_len = PROTO_HEADER_LEN + tlv_len;
    if (buf_size < frame_len) return 0;

    /* 逐字节写入 (小端序) */
    buf[0] = fc;
    buf[1] = (uint8_t)(dest & 0xFF);
    buf[2] = (uint8_t)((dest >> 8) & 0xFF);
    buf[3] = (uint8_t)(sender & 0xFF);
    buf[4] = (uint8_t)((sender >> 8) & 0xFF);
    buf[5] = (uint8_t)(seq & 0xFF);
    buf[6] = (uint8_t)((seq >> 8) & 0xFF);

    if (tlv_data != NULL && tlv_len > 0) {
        memcpy(&buf[PROTO_HEADER_LEN], tlv_data, tlv_len);
    }
    return frame_len;
}

/* 解析接收帧: 原始字节 → proto_frame_t */
bool proto_parse_frame(const uint8_t *buf, uint16_t len, proto_frame_t *frame)
{
    if (buf == NULL || frame == NULL) return false;
    if (len < PROTO_HEADER_LEN) return false;

    uint8_t fc = buf[0];
    proto_parse_fc(fc, &frame->frame_type, &frame->ack_type,
                   &frame->sf_flag, &frame->power_level);

    /* 小端序解码 */
    frame->dest_addr   = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
    frame->sender_addr = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
    frame->msg_seq     = (uint16_t)buf[5] | ((uint16_t)buf[6] << 8);

    frame->payload     = &buf[PROTO_HEADER_LEN];
    frame->payload_len = len - PROTO_HEADER_LEN;
    return true;
}

/*============================================================================
 *                              TLV 通用操作
 *===========================================================================*/

/* 编码单个TLV: [Type(1)][Length(1)][Value(Length)], 返回TLV总长度 */
uint8_t proto_tlv_encode(uint8_t *buf, uint16_t buf_size,
                         uint8_t type, const uint8_t *value, uint8_t value_len)
{
    if (buf_size < (uint16_t)(2 + value_len)) return 0;
    buf[0] = type;
    buf[1] = value_len;
    if (value != NULL && value_len > 0) {
        memcpy(&buf[2], value, value_len);
    }
    return 2 + value_len;
}

/* 在TLV数据段中查找指定Type, 返回Value指针和长度 */
bool proto_tlv_find(const uint8_t *tlv_data, uint16_t tlv_len,
                    uint8_t type, const uint8_t **value, uint8_t *value_len)
{
    uint16_t offset = 0;
    /* 遍历TLV序列: 每个单元 = Type(1) + Length(1) + Value(Length) */
    while (offset + 2 <= tlv_len) {
        uint8_t t = tlv_data[offset];
        uint8_t l = tlv_data[offset + 1];
        if (offset + 2 + l > tlv_len) return false;  /* TLV被截断 */
        if (t == type) {
            if (value)     *value     = &tlv_data[offset + 2];
            if (value_len) *value_len = l;
            return true;
        }
        offset += 2 + l;  /* 跳到下一个TLV */
    }
    return false;
}

/*============================================================================
 *                              专用TLV编码
 *===========================================================================*/

/* TLV 0x01 实时传感器数据: temp(2B,LE) + hum(2B) + CO(2B) + batt(2B) + alarm(1B) = 9B */
uint8_t proto_tlv_sensor_realtime(uint8_t *buf, uint16_t buf_size,
                                  int16_t temp_c, uint16_t hum_rh,
                                  uint16_t co_ppm, uint16_t batt_mv, uint8_t alarm)
{
    uint8_t value[9];
    value[0]  = (uint8_t)(temp_c & 0xFF);
    value[1]  = (uint8_t)((temp_c >> 8) & 0xFF);
    value[2]  = (uint8_t)(hum_rh & 0xFF);
    value[3]  = (uint8_t)((hum_rh >> 8) & 0xFF);
    value[4]  = (uint8_t)(co_ppm & 0xFF);
    value[5]  = (uint8_t)((co_ppm >> 8) & 0xFF);
    value[6]  = (uint8_t)(batt_mv & 0xFF);
    value[7]  = (uint8_t)((batt_mv >> 8) & 0xFF);
    value[8]  = alarm;
    return proto_tlv_encode(buf, buf_size, TLV_SENSOR_REALTIME, value, 9);
}

/* TLV 0x10 历史数据块: hist_seq(2B) + timestamp(2B) + 传感器数据(9B) = 13B */
uint8_t proto_tlv_history_block(uint8_t *buf, uint16_t buf_size,
                                uint16_t hist_seq, uint16_t timestamp,
                                int16_t temp_c, uint16_t hum_rh,
                                uint16_t co_ppm, uint16_t batt_mv, uint8_t alarm)
{
    uint8_t value[13];
    value[0]  = (uint8_t)(hist_seq & 0xFF);
    value[1]  = (uint8_t)((hist_seq >> 8) & 0xFF);
    value[2]  = (uint8_t)(timestamp & 0xFF);
    value[3]  = (uint8_t)((timestamp >> 8) & 0xFF);
    value[4]  = (uint8_t)(temp_c & 0xFF);
    value[5]  = (uint8_t)((temp_c >> 8) & 0xFF);
    value[6]  = (uint8_t)(hum_rh & 0xFF);
    value[7]  = (uint8_t)((hum_rh >> 8) & 0xFF);
    value[8]  = (uint8_t)(co_ppm & 0xFF);
    value[9]  = (uint8_t)((co_ppm >> 8) & 0xFF);
    value[10] = (uint8_t)(batt_mv & 0xFF);
    value[11] = (uint8_t)((batt_mv >> 8) & 0xFF);
    value[12] = alarm;
    return proto_tlv_encode(buf, buf_size, TLV_HISTORY_BLOCK, value, 13);
}

/* TLV 0xAC ACK信息: last_seq(2B) + rssi(1B) + snr(1B) + gw_power(1B) [+ suggest(1B)] */
uint8_t proto_tlv_ack_info(uint8_t *buf, uint16_t buf_size,
                           uint16_t last_in_seq, int8_t rssi, int8_t snr,
                           int8_t gw_tx_power,
                           bool has_suggest, uint8_t suggest_power)
{
    uint8_t value[5];
    value[0] = (uint8_t)(last_in_seq & 0xFF);
    value[1] = (uint8_t)((last_in_seq >> 8) & 0xFF);
    value[2] = (uint8_t)rssi;
    value[3] = (uint8_t)snr;
    value[4] = (uint8_t)gw_tx_power;

    /* 含功率建议时Value=6字节 */
    if (has_suggest) {
        if (buf_size < 8) return 0;
        buf[0] = TLV_ACK_INFO; buf[1] = 6;
        memcpy(&buf[2], value, 5); buf[7] = suggest_power;
        return 8;
    }
    return proto_tlv_encode(buf, buf_size, TLV_ACK_INFO, value, 5);
}

/* 解析TLV 0xAC */
bool proto_parse_tlv_ack(const uint8_t *value, uint8_t len, proto_ack_info_t *ack)
{
    if (len < 5) return false;
    ack->last_in_seq  = (uint16_t)value[0] | ((uint16_t)value[1] << 8);
    ack->rssi         = (int8_t)value[2];
    ack->snr          = (int8_t)value[3];
    ack->gw_tx_power  = (int8_t)value[4];
    ack->has_suggest  = (len >= 6);           /* Length>=6表示包含功率建议 */
    ack->suggest_power = ack->has_suggest ? value[5] : 0;
    return true;
}

/* TLV 0x40 Beacon: type(1) + power(1) + gw_id(2) + hops(1) + flags(1) [+ load(1)] */
uint8_t proto_tlv_beacon(uint8_t *buf, uint16_t buf_size,
                         uint8_t beacon_type, int8_t tx_power,
                         uint16_t gateway_id, uint8_t hops, uint8_t flags,
                         bool has_load, uint8_t load_factor)
{
    uint8_t value[6];
    value[0] = beacon_type;
    value[1] = (uint8_t)tx_power;
    value[2] = (uint8_t)(gateway_id & 0xFF);
    value[3] = (uint8_t)((gateway_id >> 8) & 0xFF);
    value[4] = hops;
    value[5] = flags;

    /* 可选负载因子 (中继/网关可省略) */
    if (has_load) {
        if (buf_size < 9) return 0;
        buf[0] = TLV_BEACON_INFO; buf[1] = 6;
        memcpy(&buf[2], value, 6); buf[8] = load_factor;
        return 9;
    }
    return proto_tlv_encode(buf, buf_size, TLV_BEACON_INFO, value, 6);
}

/* TLV 0x35 节点角色: 1字节 */
uint8_t proto_tlv_node_role(uint8_t *buf, uint16_t buf_size, uint8_t role)
{
    return proto_tlv_encode(buf, buf_size, TLV_NODE_ROLE, &role, 1);
}

/* TLV 0x36 链路探测: 1字节 (请求/应答) */
uint8_t proto_tlv_link_probe(uint8_t *buf, uint16_t buf_size, uint8_t probe_type)
{
    return proto_tlv_encode(buf, buf_size, TLV_LINK_PROBE, &probe_type, 1);
}

/* TLV 0x34 GNSS位置: lat(4B) + lon(7B) = 11B */
uint8_t proto_tlv_gnss_location(uint8_t *buf, uint16_t buf_size,
                                int8_t lat_dir, uint8_t lat_deg, uint8_t lat_min, uint16_t lat_frac,
                                int8_t lon_dir, uint16_t lon_deg, uint8_t lon_min, uint16_t lon_frac)
{
    uint8_t value[11];
    value[0]  = (uint8_t)lat_dir;
    value[1]  = lat_deg;
    value[2]  = lat_min;
    value[3]  = (uint8_t)(lat_frac & 0xFF);
    value[4]  = (uint8_t)((lat_frac >> 8) & 0xFF);
    value[5]  = (uint8_t)lon_dir;
    value[6]  = (uint8_t)(lon_deg & 0xFF);
    value[7]  = (uint8_t)((lon_deg >> 8) & 0xFF);
    value[8]  = lon_min;
    value[9]  = (uint8_t)(lon_frac & 0xFF);
    value[10] = (uint8_t)((lon_frac >> 8) & 0xFF);

    return proto_tlv_encode(buf, buf_size, TLV_GNSS_LOCATION, value, 11);
}

/* TLV 0x32 分配短地址: 2字节 (小端序) */
uint8_t proto_tlv_assign_short_addr(uint8_t *buf, uint16_t buf_size, uint16_t addr)
{
    uint8_t v[2];
    v[0] = (uint8_t)(addr & 0xFF);
    v[1] = (uint8_t)((addr >> 8) & 0xFF);
    return proto_tlv_encode(buf, buf_size, TLV_ASSIGN_SHORT_ADDR, v, 2);
}

/* TLV 0xFF 指令执行状态: 1字节 */
uint8_t proto_tlv_cmd_status(uint8_t *buf, uint16_t buf_size, uint8_t status_code)
{
    return proto_tlv_encode(buf, buf_size, TLV_CMD_STATUS, &status_code, 1);
}

/* ─── 下行指令专用编码 ─── */

/* TLV 0x20 设置采集间隔: 常规(2B) + 预警(2B) + 火警(2B) = 6B */
uint8_t proto_tlv_set_measure_interval(uint8_t *buf, uint16_t buf_size,
    uint16_t normal_s, uint16_t warning_s, uint16_t alarm_s)
{
    uint8_t v[6];
    v[0] = (uint8_t)(normal_s & 0xFF);  v[1] = (uint8_t)((normal_s >> 8) & 0xFF);
    v[2] = (uint8_t)(warning_s & 0xFF); v[3] = (uint8_t)((warning_s >> 8) & 0xFF);
    v[4] = (uint8_t)(alarm_s & 0xFF);   v[5] = (uint8_t)((alarm_s >> 8) & 0xFF);
    return proto_tlv_encode(buf, buf_size, TLV_SET_INTERVAL, v, 6);
}

/* TLV 0x21 设置发送间隔 */
uint8_t proto_tlv_set_tx_interval(uint8_t *buf, uint16_t buf_size,
    uint16_t normal_s, uint16_t warning_s, uint16_t alarm_s)
{
    uint8_t v[6];
    v[0] = (uint8_t)(normal_s & 0xFF);  v[1] = (uint8_t)((normal_s >> 8) & 0xFF);
    v[2] = (uint8_t)(warning_s & 0xFF); v[3] = (uint8_t)((warning_s >> 8) & 0xFF);
    v[4] = (uint8_t)(alarm_s & 0xFF);   v[5] = (uint8_t)((alarm_s >> 8) & 0xFF);
    return proto_tlv_encode(buf, buf_size, TLV_SET_TX_INTERVAL, v, 6);
}

/* TLV 0x22 设置温度阈值: 预警+预警恢复+报警+报警恢复, 各2B ×0.1°C = 8B */
uint8_t proto_tlv_set_temp(uint8_t *buf, uint16_t buf_size,
    int16_t warn, int16_t warn_rec, int16_t alarm, int16_t alarm_rec)
{
    uint8_t v[8];
    v[0] = (uint8_t)(warn & 0xFF);      v[1] = (uint8_t)((warn >> 8) & 0xFF);
    v[2] = (uint8_t)(warn_rec & 0xFF);  v[3] = (uint8_t)((warn_rec >> 8) & 0xFF);
    v[4] = (uint8_t)(alarm & 0xFF);     v[5] = (uint8_t)((alarm >> 8) & 0xFF);
    v[6] = (uint8_t)(alarm_rec & 0xFF); v[7] = (uint8_t)((alarm_rec >> 8) & 0xFF);
    return proto_tlv_encode(buf, buf_size, TLV_SET_TEMP, v, 8);
}

/* TLV 0x23 设置CO阈值: 预警值(2B) + 恢复值(2B) = 4B */
uint8_t proto_tlv_set_co(uint8_t *buf, uint16_t buf_size,
    uint16_t warn_ppm, uint16_t recovery_ppm)
{
    uint8_t v[4];
    v[0] = (uint8_t)(warn_ppm & 0xFF);      v[1] = (uint8_t)((warn_ppm >> 8) & 0xFF);
    v[2] = (uint8_t)(recovery_ppm & 0xFF);  v[3] = (uint8_t)((recovery_ppm >> 8) & 0xFF);
    return proto_tlv_encode(buf, buf_size, TLV_SET_CO, v, 4);
}

/* TLV 0x24 设置湿度阈值: 预警值(2B ×0.1%RH) + 恢复值(2B ×0.1%RH) = 4B */
uint8_t proto_tlv_set_humidity(uint8_t *buf, uint16_t buf_size,
    uint16_t warn_rh, uint16_t recovery_rh)
{
    uint8_t v[4];
    v[0] = (uint8_t)(warn_rh & 0xFF);      v[1] = (uint8_t)((warn_rh >> 8) & 0xFF);
    v[2] = (uint8_t)(recovery_rh & 0xFF);  v[3] = (uint8_t)((recovery_rh >> 8) & 0xFF);
    return proto_tlv_encode(buf, buf_size, TLV_SET_HUMIDITY, v, 4);
}

/* TLV 0x25 设置RF参数: 主SF(1B) + 备用SF(1B) + 默认功率档位(1B) = 3B */
uint8_t proto_tlv_set_rf(uint8_t *buf, uint16_t buf_size,
    uint8_t sf_primary, uint8_t sf_backup, uint8_t default_power)
{
    uint8_t v[3] = { sf_primary, sf_backup, default_power };
    return proto_tlv_encode(buf, buf_size, TLV_SET_RF, v, 3);
}

/* ─── 上行配置上报 ─── */

/* TLV 0x0F 全配置上报: 14个uint16字段 = 28B value, 2B头 = 30B总长
 *
 * Value布局 (28B, 小端序):
 *   [ 0- 1] measure_interval_normal   (s)
 *   [ 2- 3] measure_interval_warning  (s)
 *   [ 4- 5] measure_interval_alarm    (s)
 *   [ 6- 7] transmit_interval_normal  (s)
 *   [ 8- 9] transmit_interval_warning (s)
 *   [10-11] transmit_interval_alarm   (s)
 *   [12-13] temp_warning_threshold    (×0.1°C, signed)
 *   [14-15] temp_warning_recovery     (×0.1°C, signed)
 *   [16-17] temp_alarm_threshold      (×0.1°C, signed)
 *   [18-19] temp_alarm_recovery       (×0.1°C, signed)
 *   [20-21] co_warning_threshold      (ppm)
 *   [22-23] co_warning_recovery       (ppm)
 *   [24-25] hum_warning_threshold     (×0.1%RH)
 *   [26-27] hum_warning_recovery      (×0.1%RH)
 */
uint8_t proto_tlv_config_report(uint8_t *buf, uint16_t buf_size,
    uint16_t meas_norm, uint16_t meas_warn, uint16_t meas_alarm,
    uint16_t tx_norm, uint16_t tx_warn, uint16_t tx_alarm,
    int16_t temp_warn, int16_t temp_warn_rec,
    int16_t temp_alarm, int16_t temp_recovery,
    uint16_t co_warn, uint16_t co_recovery,
    uint16_t hum_warn, uint16_t hum_recovery)
{
    uint8_t v[28];
    v[0]  = (uint8_t)(meas_norm & 0xFF);        v[1]  = (uint8_t)((meas_norm >> 8) & 0xFF);
    v[2]  = (uint8_t)(meas_warn & 0xFF);        v[3]  = (uint8_t)((meas_warn >> 8) & 0xFF);
    v[4]  = (uint8_t)(meas_alarm & 0xFF);       v[5]  = (uint8_t)((meas_alarm >> 8) & 0xFF);
    v[6]  = (uint8_t)(tx_norm & 0xFF);          v[7]  = (uint8_t)((tx_norm >> 8) & 0xFF);
    v[8]  = (uint8_t)(tx_warn & 0xFF);          v[9]  = (uint8_t)((tx_warn >> 8) & 0xFF);
    v[10] = (uint8_t)(tx_alarm & 0xFF);         v[11] = (uint8_t)((tx_alarm >> 8) & 0xFF);
    v[12] = (uint8_t)(temp_warn & 0xFF);        v[13] = (uint8_t)((temp_warn >> 8) & 0xFF);
    v[14] = (uint8_t)(temp_warn_rec & 0xFF);    v[15] = (uint8_t)((temp_warn_rec >> 8) & 0xFF);
    v[16] = (uint8_t)(temp_alarm & 0xFF);       v[17] = (uint8_t)((temp_alarm >> 8) & 0xFF);
    v[18] = (uint8_t)(temp_recovery & 0xFF);    v[19] = (uint8_t)((temp_recovery >> 8) & 0xFF);
    v[20] = (uint8_t)(co_warn & 0xFF);          v[21] = (uint8_t)((co_warn >> 8) & 0xFF);
    v[22] = (uint8_t)(co_recovery & 0xFF);      v[23] = (uint8_t)((co_recovery >> 8) & 0xFF);
    v[24] = (uint8_t)(hum_warn & 0xFF);         v[25] = (uint8_t)((hum_warn >> 8) & 0xFF);
    v[26] = (uint8_t)(hum_recovery & 0xFF);     v[27] = (uint8_t)((hum_recovery >> 8) & 0xFF);

    return proto_tlv_encode(buf, buf_size, TLV_CONFIG_REPORT, v, 28);
}

/*============================================================================
 *                          下行指令解析
 *===========================================================================*/

/**
 * @brief  从帧payload中提取所有指令TLV (类型 0x20-0x2A)
 * @param  out_cmds: 输出数组, 按出现顺序填充
 * @param  max_cmds: 数组最大容量
 * @retval 提取到的指令条数
 */
uint8_t proto_parse_commands(const uint8_t *payload, uint16_t payload_len,
    proto_cmd_t *out_cmds, uint8_t max_cmds)
{
    uint8_t count = 0;
    uint16_t offset = 0;

    while (offset + 2 <= payload_len && count < max_cmds) {
        uint8_t type = payload[offset];
        uint8_t len  = payload[offset + 1];
        if (offset + 2 + len > payload_len) break;   /* TLV截断 */

        /* 只收集下行指令类型 (0x20-0x2A) */
        if (type >= 0x20 && type <= 0x2A) {
            out_cmds[count].type  = type;
            out_cmds[count].value = &payload[offset + 2];
            out_cmds[count].len   = len;
            count++;
        }
        offset += 2 + len;
    }
    return count;
}

/*============================================================================
 *                              功率等级映射
 *===========================================================================*/
static const int8_t power_table[] = PROTO_POWER_LEVELS;    /* {14,16,18,20,22} */
#define POWER_TABLE_COUNT 5

/* 档位 → dBm */
int8_t proto_power_level_to_dbm(uint8_t level)
{
    if (level >= POWER_TABLE_COUNT) return -1;
    return power_table[level];
}

/* dBm → 最接近的不超过该值的档位 */
uint8_t proto_dbm_to_power_level(int8_t dbm)
{
    uint8_t best = 0;
    for (uint8_t i = 0; i < POWER_TABLE_COUNT; i++) {
        if (power_table[i] <= dbm) best = i;
    }
    return best;
}

/*============================================================================
 *                              重传队列
 *===========================================================================*/

/* 初始化 */
void proto_retransmit_init(void)
{
    retransmit_head = 0; retransmit_tail = 0; retransmit_count = 0;
}

/* 推入条目, 队列满时丢弃最旧 */
bool proto_retransmit_push(const proto_retransmit_entry_t *entry)
{
    if (retransmit_count >= PROTO_RETRANSMIT_QUEUE_SIZE) {
        retransmit_head = (retransmit_head + 1) % PROTO_RETRANSMIT_QUEUE_SIZE;
        retransmit_count--;
    }
    retransmit_queue[retransmit_tail] = *entry;
    retransmit_tail = (retransmit_tail + 1) % PROTO_RETRANSMIT_QUEUE_SIZE;
    retransmit_count++;
    return true;
}

/* ACK确认: 移除所有 msg_seq ≤ last_in_seq 的条目 */
void proto_retransmit_ack(uint16_t last_in_seq)
{
    while (retransmit_count > 0) {
        if (retransmit_queue[retransmit_head].msg_seq <= last_in_seq) {
            retransmit_head = (retransmit_head + 1) % PROTO_RETRANSMIT_QUEUE_SIZE;
            retransmit_count--;
        } else {
            break;
        }
    }
}

/* 获取最早未确认条目 */
bool proto_retransmit_oldest(proto_retransmit_entry_t *entry)
{
    if (retransmit_count == 0) return false;
    *entry = retransmit_queue[retransmit_head];
    return true;
}

/* 按索引获取条目 (0=最旧) */
bool proto_retransmit_get(uint16_t index, proto_retransmit_entry_t *entry)
{
    if (index >= retransmit_count || entry == NULL) return false;
    uint16_t pos = (retransmit_head + index) % PROTO_RETRANSMIT_QUEUE_SIZE;
    *entry = retransmit_queue[pos];
    return true;
}

/* 队列条目数 */
uint16_t proto_retransmit_count(void)
{
    return retransmit_count;
}

/*============================================================================
 *                              Beacon 候选管理
 *===========================================================================*/

/* 清空候选列表和排除标记 */
void proto_candidates_clear(void)
{
    candidate_count = 0;
    for (uint8_t i = 0; i < PROTO_MAX_CANDIDATES; i++) candidate_excluded[i] = false;
}

/* 添加候选 */
bool proto_candidates_add(uint16_t sender_addr, uint8_t beacon_type,
                          int8_t tx_power, uint16_t gateway_id,
                          uint8_t hop_count, bool is_valid,
                          uint8_t load_factor, int16_t rssi, int8_t snr)
{
    if (candidate_count >= PROTO_MAX_CANDIDATES) return false;
    proto_beacon_candidate_t *c = &candidates[candidate_count];
    c->sender_addr = sender_addr;
    c->beacon_type = beacon_type;
    c->tx_power    = tx_power;
    c->gateway_id  = gateway_id;
    c->hop_count   = hop_count;
    c->is_valid    = is_valid;
    c->load_factor = load_factor;
    c->rssi        = rssi;
    c->snr         = snr;
    candidate_count++;
    return true;
}

/**
 * @brief  从候选列表中选择最佳父节点
 * @note   过滤条件: require_valid过滤, hops避免环路
 *         评分策略: IsValid(+0x4000) > 网关优先(+0x2000) > 短路经 > 低负载 > 强RSSI
 */
bool proto_candidates_select(int8_t my_max_power, int8_t sf_sensitivity,
                             int8_t margin, bool require_valid,
                             uint8_t current_hops, uint16_t *out_addr)
{
    if (candidate_count == 0 || out_addr == NULL) return false;

    int16_t best_score = -32768;
    uint8_t best_idx = 0;
    bool found = false;

    for (uint8_t i = 0; i < candidate_count; i++) {
        /* 跳过已排除的候选 (注册失败标记) */
        if (candidate_excluded[i]) continue;

        const proto_beacon_candidate_t *c = &candidates[i];

        /* IsValid过滤: 只选中继有效候选 */
        if (require_valid && !c->is_valid) continue;

        /* 环路避免: 只能选跳数比当前小的父节点 */
        if (current_hops != 255 && c->hop_count >= current_hops) continue;

        /* 预估上行RSSI = 自身最大功率 - (Beacon发射功率 - 下行接收RSSI) */
        int16_t est_up = my_max_power - (c->tx_power - c->rssi);
        if (est_up < sf_sensitivity - margin) continue;  /* 链路不可达 */

        /* 评分: 综合考量信号强度、路径长度、负载 */
        int16_t score = 0;
        if (c->is_valid)      score += 0x4000;           /* 中继有效 */
        if (c->beacon_type == 0) score += 0x2000;        /* 直连网关优先 */
        score -= (int16_t)c->hop_count * 500;             /* 路径越短越好 */
        score -= (int16_t)c->load_factor * 100;           /* 负载越低越好 */
        score += est_up;                                  /* 信号越强越好 */

        if (!found || score > best_score) {
            best_score = score;
            best_idx = i;
            found = true;
        }
    }

    if (!found) return false;
    *out_addr = candidates[best_idx].sender_addr;
    return true;
}

/* 候选总数 */
uint8_t proto_candidates_count(void)
{
    return candidate_count;
}

/* 按索引获取 */
const proto_beacon_candidate_t *proto_candidates_get(uint8_t index)
{
    if (index >= candidate_count) return NULL;
    return &candidates[index];
}

/* 排除指定地址 */
void proto_candidates_exclude(uint16_t addr)
{
    for (uint8_t i = 0; i < candidate_count; i++) {
        if (candidates[i].sender_addr == addr) {
            candidate_excluded[i] = true;
            return;
        }
    }
}

/* 重置排除 */
void proto_candidates_reset_exclusions(void)
{
    for (uint8_t i = 0; i < PROTO_MAX_CANDIDATES; i++) candidate_excluded[i] = false;
}

/*============================================================================
 *                          高层帧构建
 *===========================================================================*/

/**
 * @brief  构建完整注册帧
 * @note   组装 TLV 0x30(UID 12B) + 0x31(Cap 3B) + 0x35(Role 1B) + 可选 0x34(Location 11B)
 *         FrameControl: FrameType=REG, AckType=0, SF=主, Power=当前档位
 *         SenderAddr: 0xFFFF (未注册)
 */
uint16_t proto_build_register_frame(uint8_t *buf, uint16_t buf_size,
    const uint8_t uid[12], const proto_gnss_location_t *location,
    uint16_t dest_addr, uint8_t power_level)
{
    uint8_t tlv[64];    /* 4个TLV ≈ 40B, 64B足够 */
    uint8_t pos = 0;
    uint8_t ret;

    /* TLV 0x30: 长ID (12字节) */
    ret = proto_tlv_encode(tlv + pos, sizeof(tlv) - pos,
                           TLV_REGISTER_LONG_ID, uid, 12);
    if (ret == 0) return 0; pos += ret;

    /* TLV 0x31: 终端能力 (maxSF, maxPower, sensorMask) */
    uint8_t cap[3] = { PROTO_SF_BACKUP, PROTO_POWER_LEVEL_MAX, 0x07 };
    ret = proto_tlv_encode(tlv + pos, sizeof(tlv) - pos,
                           TLV_TERMINAL_CAPABILITY, cap, 3);
    if (ret == 0) return 0; pos += ret;

    /* TLV 0x35: 节点角色 = 终端 */
    ret = proto_tlv_encode(tlv + pos, sizeof(tlv) - pos,
                           TLV_NODE_ROLE,
                           (const uint8_t[]){NODE_ROLE_TERMINAL}, 1);
    if (ret == 0) return 0; pos += ret;

    /* TLV 0x34: GNSS位置 (可选, 仅当location非NULL时附带) */
    if (location != NULL) {
        ret = proto_tlv_gnss_location(tlv + pos, sizeof(tlv) - pos,
                location->lat_dir, location->lat_deg,
                location->lat_min, location->lat_frac,
                location->lon_dir, location->lon_deg,
                location->lon_min, location->lon_frac);
        if (ret == 0) return 0; pos += ret;
    }

    uint8_t fc = proto_make_fc(PROTO_FRAME_TYPE_REG, 0,
                               PROTO_SF_FLAG_PRIMARY, power_level);

    return proto_build_frame(buf, buf_size,
                             fc, dest_addr, PROTO_ADDR_BROADCAST, 0,
                             tlv, pos);
}

/**
 * @brief  构建完整传感器数据帧
 * @note   顺序: 0x01实时(必选) → extra(必选) → 0x10历史×N(用剩余空间填充)
 *         FrameControl: FrameType=DATA, AckType=0
 */
uint16_t proto_build_data_frame(uint8_t *buf, uint16_t buf_size,
    const proto_sensor_data_t *sensor,
    uint16_t dest_addr, uint16_t sender_addr,
    uint16_t seq, uint8_t sf_flag, uint8_t power_level,
    const uint8_t *extra_tlv, uint8_t extra_len)
{
    /* 直接在输出缓冲区payload区域构建TLV序列 */
    uint8_t *payload = buf + PROTO_HEADER_LEN;
    uint16_t max_payload = buf_size - PROTO_HEADER_LEN;
    if (max_payload > PROTO_MAX_PAYLOAD_LEN) max_payload = PROTO_MAX_PAYLOAD_LEN;
    uint16_t pos = 0;
    uint8_t ret;

    /* TLV 0x01: 实时传感器数据 (11B = 2+9), 必须携带 */
    ret = proto_tlv_sensor_realtime(payload + pos, max_payload - pos,
            sensor->temperature, sensor->humidity,
            sensor->co_concentration, sensor->battery_mv,
            sensor->alarm_flag);
    if (ret == 0) return 0; pos += ret;

    /* 附加TLV (配置上报 / CMD状态): 优先于历史块, 必须携带 */
    if (extra_tlv != NULL && extra_len > 0) {
        if (pos + extra_len > max_payload) return 0;  /* 放不下则整帧失败 */
        memcpy(payload + pos, extra_tlv, extra_len);
        pos += extra_len;
    }

    /* TLV 0x10: 历史数据补发 — 用剩余空间尽量填充, 每个15B */
    for (uint16_t i = 0; i < proto_retransmit_count(); i++) {
        if (max_payload - pos < 15) break;
        proto_retransmit_entry_t hist;
        proto_retransmit_get(i, &hist);
        ret = proto_tlv_history_block(payload + pos, max_payload - pos,
                hist.msg_seq, (uint16_t)(hist.timestamp / 1000),
                hist.temperature, hist.humidity,
                hist.co_concentration, hist.battery_mv, hist.alarm_flag);
        if (ret == 0) break;
        pos += ret;
    }

    uint8_t fc = proto_make_fc(PROTO_FRAME_TYPE_DATA, 0,
                               sf_flag, power_level);

    return proto_build_frame(buf, buf_size,
                             fc, dest_addr, sender_addr, seq,
                             payload, pos);
}

/*============================================================================
 *                          高层 RX 分发
 *===========================================================================*/

/**
 * @brief  统一处理接收到的帧, 返回动作码
 * @note   内部自动完成:
 *         - Beacon: 提取TLV 0x40 → proto_candidates_add()
 *         - ACK帧: 提取TLV 0xAC → 更新重传队列 + 缓存ack信息
 *         - 注册完成: 提取TLV 0x32 → 存 assigned_short_addr
 *         调用者根据返回的proto_rx_action_t决定状态跳转
 */
proto_rx_action_t proto_dispatch_rx(const uint8_t *buf, uint16_t len,
    int16_t rssi, int8_t snr)
{
    proto_frame_t frame;

    /* 帧过短, 无法解析 */
    if (len < PROTO_HEADER_LEN) return PROTO_RX_PARSE_ERROR;

    if (!proto_parse_frame(buf, len, &frame))
        return PROTO_RX_PARSE_ERROR;

    /* ─── Beacon (FrameType=CMD + TLV 0x40) ─── */
    if (frame.frame_type == PROTO_FRAME_TYPE_CMD) {
        const uint8_t *v; uint8_t vl;
        if (proto_tlv_find(frame.payload, frame.payload_len,
                           TLV_BEACON_INFO, &v, &vl) && vl >= 6) {
            uint8_t btype = v[0];                           /* 0=网关 1=中继 */
            int8_t  bpow  = (int8_t)v[1];                   /* 发射功率 */
            uint16_t gwid = (uint16_t)v[2] | ((uint16_t)v[3] << 8);  /* 网关ID */
            uint8_t hops  = v[4];                           /* HopsToGateway */
            bool    valid = (v[5] & BEACON_FLAG_IS_VALID) != 0; /* IsValid */
            uint8_t load  = (vl >= 7) ? v[6] : 0;           /* 负载(可选) */

            proto_candidates_add(frame.sender_addr, btype,
                                bpow, gwid, hops, valid, load, rssi, snr);
            return PROTO_RX_BEACON;
        }
    }

    /* ─── ACK帧 ─── */
    if (frame.frame_type == PROTO_FRAME_TYPE_ACK) {
        const uint8_t *v; uint8_t vl;

        /* 优先检查注册完成 (TLV 0x32: 分配短地址) */
        if (proto_tlv_find(frame.payload, frame.payload_len,
                           TLV_ASSIGN_SHORT_ADDR, &v, &vl) && vl >= 2) {
            assigned_short_addr = (uint16_t)v[0] | ((uint16_t)v[1] << 8);
            return PROTO_RX_REGISTERED;
        }

        /* 代理ACK (中继转发): 需扩展接收窗口等最终确认 */
        if (frame.ack_type == PROTO_ACK_TYPE_PROXY)
            return PROTO_RX_PROXY_ACK;

        /* 端到端/普通ACK: 解析TLV 0xAC, 更新重传队列 */
        if (proto_tlv_find(frame.payload, frame.payload_len,
                           TLV_ACK_INFO, &v, &vl)) {
            if (proto_parse_tlv_ack(v, vl, &last_ack)) {
                proto_retransmit_ack(last_ack.last_in_seq);
                return PROTO_RX_ACK_CONFIRMED;
            }
        }
    }

    return PROTO_RX_NOTHING;
}

/* 获取最新ACK信息 */
const proto_ack_info_t *proto_get_last_ack(void)
{
    return &last_ack;
}

/* 获取注册分配的短地址 */
uint16_t proto_get_assigned_addr(void)
{
    return assigned_short_addr;
}
