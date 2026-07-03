/**
 ******************************************************************************
 * @file    lora_protocol.h
 * @brief   LoRa私有通信协议模块 — 帧结构/TLV/终端状态机
 * @note    纯协议层, 不依赖硬件/Radio/HAL, 独立于平台
 *          帧格式: [FC(1)] [DestAddr(2)] [SenderAddr(2)] [MsgSeq(2)] [TLV Payload]
 *          硬件CRC自动追加/校验 (crcOn=true)
 ******************************************************************************
 */

#ifndef LORA_PROTOCOL_H_
#define LORA_PROTOCOL_H_

#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 *                              通用常量
 *===========================================================================*/

/* 保留短地址 */
#define PROTO_ADDR_GATEWAY        0x0000      /* 网关固定地址 */
#define PROTO_ADDR_BROADCAST      0xFFFF      /* 广播/未注册临时地址 */

/* 主/备用扩频因子 — 终端默认主SF, 恶化时切备用 */
#define PROTO_SF_PRIMARY          7           /* 主SF (高速,低功耗) */
#define PROTO_SF_BACKUP           11          /* 备用SF (高灵敏度) */

/* 协议时序常量 (单位: ms) — 全网统一, app_main直接引用 */

/* Class A RX窗口 (数据帧发送后使用) */
#define PROTO_RX_DELAY_MS         50          /* 发送完成后→RX窗口延迟 */
#define PROTO_RX_WINDOW_MS        2000        /* 单个RX窗口持续时长 */
#define PROTO_RX_GAP_MS           50          /* RX1结束→RX2开启间隔 */
/* 计算值 (RX2开启时刻) */
#define PROTO_RX2_DELAY_MS        (PROTO_RX_DELAY_MS + PROTO_RX_WINDOW_MS + PROTO_RX_GAP_MS)

/* 注册流程专用 */
#define PROTO_REG_RX_DELAY_MS     10          /* 注册帧→RX窗口延迟 */
#define PROTO_REG_RX15_MS         15000       /* 注册15秒RX窗口 (网关直接) */
#define PROTO_REG_RX_EXTEND_MS    180000      /* 中继扩展RX窗口 3分钟 (收到代理ACK后) */

/* 其他时序 */
#define PROTO_TX_TIMEOUT          5000        /* 发送超时 */
#define PROTO_SCAN_DURATION_MS    45000       /* Beacon扫描持续时间 (>2个Beacon周期20s) */
#define PROTO_REG_RETRY_INTERVAL  3000        /* 注册重试间隔 */
#define PROTO_REG_COMPLETE_TIMEOUT 2000       /* 等待注册完成超时 */
#define PROTO_LOSS_THRESHOLD      5           /* 连续丢帧阈值→切备用SF */
#define PROTO_RECOVER_THRESHOLD   10          /* 备用SF恢复阈值→切回主SF */
#define PROTO_MAX_CONSECUTIVE_LOSS 10         /* 最大连续丢失→父节点失效 */
#define PROTO_ISOLATED_RETRY_1    60000       /* 隔离态重试: 1分钟 */
#define PROTO_ISOLATED_RETRY_5    300000      /* 隔离态重试: 5分钟 */
#define PROTO_ISOLATED_RETRY_30   1800000     /* 隔离态重试: 30分钟 */
#define PROTO_ISOLATED_RETRY_MAX  3600000     /* 隔离态重试上限: 60分钟 */

#define PROTO_MAX_HOPS            3           /* 网络最大跳数 */

/*============================================================================
 *                              FrameControl 位定义
 *                              (1字节, MSB=bit7, LSB=bit0)
 *
 *  | bit 7-6 | bit 5-4 | bit 3   | bit 2-0  |
 *  |---------|---------|---------|----------|
 *  |帧类型   | ACK类型 | SF指示  | 功率档位 |
 *===========================================================================*/

/* 帧类型 (bit 7-6) */
#define PROTO_FRAME_TYPE_DATA       0x00      /* 数据帧 */
#define PROTO_FRAME_TYPE_ACK        0x40      /* ACK确认帧 */
#define PROTO_FRAME_TYPE_CMD        0x80      /* 命令帧/Beacon */
#define PROTO_FRAME_TYPE_REG        0xC0      /* 注册帧 */
#define PROTO_FRAME_TYPE_MASK       0xC0      /* 帧类型掩码 */

/* ACK类型 (bit 5-4) — 仅 FrameType=ACK 时有效 */
#define PROTO_ACK_TYPE_NONE         0x00      /* 无ACK (普通数据帧) */
#define PROTO_ACK_TYPE_PROXY        0x10      /* 代理ACK (中继发出) */
#define PROTO_ACK_TYPE_END2END      0x20      /* 端到端ACK (网关直接) */
#define PROTO_ACK_TYPE_RSVD         0x30      /* 保留 */
#define PROTO_ACK_TYPE_MASK         0x30

/* SF指示 (bit 3) */
#define PROTO_SF_FLAG_PRIMARY       0x00      /* 主SF */
#define PROTO_SF_FLAG_BACKUP        0x08      /* 备用SF */
#define PROTO_SF_FLAG_MASK          0x08

/* 功率档位 (bit 2-0) */
#define PROTO_POWER_LEVEL_MASK      0x07

/* 帧头固定长度 (FC + DestAddr + SenderAddr + MsgSeq = 7字节) */
#define PROTO_HEADER_LEN            7

/* TLV载荷最大长度 (Payload上限) */
#define PROTO_MAX_PAYLOAD_LEN       120

/*============================================================================
 *                              TLV 类型定义
 *                              每个TLV: Type(1B) + Length(1B) + Value(Length)
 *===========================================================================*/

// ─── 传感器数据 ───
#define TLV_SENSOR_REALTIME         0x01      /* 实时传感器数据 (温度/湿度/CO/电池/报警) */

// ─── 历史数据 ───
#define TLV_HISTORY_BLOCK           0x10      /* 历史数据块 (原序列号+时间戳+传感器数据) */

// ─── ACK信息 ───
#define TLV_ACK_INFO                0xAC      /* ACK确认信息 (LastInSeq/RSSI/SNR/TxPower) */

// ─── 下行指令 (范围 0x20-0x2A) ───
#define TLV_SET_INTERVAL            0x20      /* 设置采集间隔 6B: 常规(2B)+预警(2B)+火警(2B) 秒 */
#define TLV_SET_TX_INTERVAL         0x21      /* 设置发送间隔 6B: 常规(2B)+预警(2B)+火警(2B) 秒 */
#define TLV_SET_TEMP                0x22      /* 设置温度阈值 8B: 预警+预警恢复+报警+报警恢复, ×0.1°C */
#define TLV_SET_CO                  0x23      /* 设置CO阈值   4B: 预警值(2B ppm)+恢复值(2B ppm) */
#define TLV_SET_HUMIDITY            0x24      /* 设置湿度阈值 4B: 预警值(2B ×0.1%RH)+恢复值(2B ×0.1%RH) */
#define TLV_SET_RF                  0x25      /* 设置RF参数   3B: 主SF(1B)+备用SF(1B)+默认功率档位(1B) */
#define TLV_IMMEDIATE_REPORT        0x26      /* 立即上报请求 0B */
#define TLV_REQUEST_RETRANSMIT      0x27      /* 请求补发     2B: 目标序列号 */
#define TLV_UPDATE_PARENT           0x28      /* 更新父节点   2B: 新短地址 */
#define TLV_QUERY_CONFIG            0x29      /* 查询全配置   0B → 终端下次上行附带配置TLV */
#define TLV_REMOTE_RESET            0x2A      /* 远程复位     0B → 延迟5s复位 */

// ─── 上行配置上报 ───
#define TLV_CONFIG_REPORT           0x0F      /* 全配置上报 (24B, 回应0x29) */

// ─── 注册/身份 ───
#define TLV_REGISTER_LONG_ID        0x30      /* 注册长ID (12B 唯一标识) */
#define TLV_TERMINAL_CAPABILITY     0x31      /* 终端能力 (最大SF/功率档/传感器掩码) */
#define TLV_ASSIGN_SHORT_ADDR       0x32      /* 分配短地址 (2B, 注册完成时返回) */
#define TLV_TX_POWER_DBM            0x33      /* 发射功率 (1B dBm) */
#define TLV_GNSS_LOCATION           0x34      /* GNSS位置信息 (11B, 注册帧附带) */
#define TLV_NODE_ROLE               0x35      /* 节点角色声明 (1B: 0终端/1中继) */
#define TLV_LINK_PROBE              0x36      /* 链路探测 (1B: 0请求/1应答) */

// ─── Beacon/状态 ───
#define TLV_BEACON_INFO             0x40      /* Beacon信息 (type/power/gw_id/hops/flags/load) */
#define TLV_CMD_STATUS              0xFF      /* 指令执行状态 (每项2B: type+status, 支持多条) */

/*============================================================================
 *                              节点角色码 (TLV 0x35 Value 单字节)
 *===========================================================================*/
#define NODE_ROLE_TERMINAL          0         /* 终端节点 */
#define NODE_ROLE_RELAY             1         /* 中继节点 */

/*============================================================================
 *                              链路探测类型 (TLV 0x36 Value[0])
 *===========================================================================*/
#define LINK_PROBE_REQUEST          0         /* 探测请求 */
#define LINK_PROBE_RESPONSE         1         /* 探测应答 */

/*============================================================================
 *                              Beacon Flags (TLV 0x40 的 Flags 字节)
 *===========================================================================*/
#define BEACON_FLAG_IS_VALID        0x01      /* bit0: 中继已确认连通网关, 可为终端服务 */

/*============================================================================
 *                              功率档位映射
 *                              档位0~4对应dBm: 14/16/18/20/22
 *===========================================================================*/
#define PROTO_POWER_LEVELS          { 14, 16, 18, 20, 22 }
#define PROTO_POWER_LEVEL_MAX       4         /* 最大档位 */
#define PROTO_POWER_LEVEL_DEFAULT   0         /* 默认档位=14dBm */

/*============================================================================
 *                              终端协议状态机
 *                              SCAN → REGISTERING → CONNECTED ⇄ ISOLATED
 *===========================================================================*/
typedef enum {
    PROTO_STATE_SCAN = 0,                      /* 扫描Beacon, 选择父节点 */
    PROTO_STATE_REGISTERING,                   /* 向父节点发送注册请求 */
    PROTO_STATE_CONNECTED,                     /* 连接态: 周期上报+处理ACK */
    PROTO_STATE_ISOLATED,                      /* 失联态: 退避休眠后重试 */
} proto_terminal_state_t;

/*============================================================================
 *                              中继协议状态机 (预留, 终端暂不使用)
 *                              SCAN → REGISTERING → ACTIVE → LOST → SCAN
 *===========================================================================*/
typedef enum {
    RELAY_STATE_SCAN = 0,
    RELAY_STATE_REGISTERING,
    RELAY_STATE_ACTIVE,                        /* 已激活, IsValid=1, 广播Beacon */
    RELAY_STATE_LOST,                          /* 与父节点失联, IsValid=0 */
} proto_relay_state_t;

/* 协议错误码 (TLV 0xFF 的 Value) */
#define PROTO_ERR_SUCCESS        0             /* 指令执行成功 */
#define PROTO_ERR_PARAM          1             /* 参数错误 */
#define PROTO_ERR_EXEC           2             /* 执行失败 */

/*============================================================================
 *                              帧结构 (proto_parse_frame 解析结果)
 *===========================================================================*/
typedef struct {
    uint8_t  frame_type;                       /* PROTO_FRAME_TYPE_xxx */
    uint8_t  ack_type;                         /* PROTO_ACK_TYPE_xxx (ACK帧时有效) */
    uint8_t  sf_flag;                          /* 主/备用SF标志 */
    uint8_t  power_level;                      /* 发送方功率档位 */
    uint16_t dest_addr;                        /* 目标短地址 (小端序) */
    uint16_t sender_addr;                      /* 发送方短地址 (小端序) */
    uint16_t msg_seq;                          /* 消息序列号 (小端序) */
    const uint8_t *payload;                    /* TLV载荷数据指针 */
    uint16_t payload_len;                      /* TLV载荷长度 */
} proto_frame_t;

/*============================================================================
 *                              重传队列
 *                              内存FIFO, 最大16条
 *===========================================================================*/
#define PROTO_RETRANSMIT_QUEUE_SIZE  16

typedef struct {
    uint32_t timestamp;                        /* 采集时刻 (HAL_GetTick) */
    int16_t  temperature;                      /* 温度 (×10, 即0.1°C) */
    uint16_t humidity;                         /* 湿度 (×10, 即0.1%RH) */
    uint16_t co_concentration;                 /* CO浓度 (ppm) */
    uint16_t battery_mv;                       /* 电池电压 (mV) */
    uint8_t  alarm_flag;                       /* 报警标志 */
    uint16_t msg_seq;                          /* 对应消息序列号 */
} proto_retransmit_entry_t;

/*============================================================================
 *                              Beacon 候选列表
 *                              扫描期间收集, 最多8个候选
 *===========================================================================*/
#define PROTO_MAX_CANDIDATES    8

typedef struct {
    uint16_t sender_addr;                      /* 发送方短地址 */
    uint8_t  beacon_type;                      /* 0=网关, 1=中继 */
    int8_t   tx_power;                         /* 发送功率 (dBm) */
    uint16_t gateway_id;                       /* 可达网关短地址 */
    uint8_t  hop_count;                        /* 到网关跳数 (HopsToGateway) */
    bool     is_valid;                         /* Flags.IsValid 位 */
    uint8_t  load_factor;                      /* 接入终端数/负载 */
    int16_t  rssi;                             /* 接收RSSI (dBm) */
    int8_t   snr;                              /* 接收SNR (0.25dB步进) */
} proto_beacon_candidate_t;

/*============================================================================
 *                              ACK 信息 (解析结果)
 *===========================================================================*/
typedef struct {
    uint16_t last_in_seq;                      /* 累计连续收到最大序列号 */
    int8_t   rssi;                             /* 上行RSSI (dBm, 有符号) */
    int8_t   snr;                              /* 上行SNR (0.25dB步进) */
    int8_t   gw_tx_power;                      /* 网关/中继发射功率 (dBm) */
    bool     has_suggest;                      /* 是否包含功率建议 */
    uint8_t  suggest_power;                    /* 建议功率档位 */
} proto_ack_info_t;

/*============================================================================
 *                              帧操作 (底层)
 *===========================================================================*/

/* 构造FrameControl字节 (按位或组合) */
uint8_t proto_make_fc(uint8_t frame_type, uint8_t ack_type,
                      uint8_t sf_flag, uint8_t power_level);

/* 从FrameControl字节提取各字段 */
void proto_parse_fc(uint8_t fc, uint8_t *frame_type, uint8_t *ack_type,
                    uint8_t *sf_flag, uint8_t *power_level);

/* 构建完整发送帧 (header + TLV payload), 返回帧长 */
uint16_t proto_build_frame(uint8_t *buf, uint16_t buf_size,
                           uint8_t fc, uint16_t dest, uint16_t sender, uint16_t seq,
                           const uint8_t *tlv_data, uint8_t tlv_len);

/* 解析接收帧 (buf→proto_frame_t), 不校验CRC (硬件已校验) */
bool proto_parse_frame(const uint8_t *buf, uint16_t len, proto_frame_t *frame);

/*============================================================================
 *                              TLV 通用操作 (底层)
 *===========================================================================*/

/* 编码单个TLV单元: Type(1B) + Length(1B) + Value */
uint8_t proto_tlv_encode(uint8_t *buf, uint16_t buf_size,
                         uint8_t type, const uint8_t *value, uint8_t value_len);

/* 在TLV数据段中查找指定Type, 返回Value指针和长度 */
bool proto_tlv_find(const uint8_t *tlv_data, uint16_t tlv_len,
                    uint8_t type, const uint8_t **value, uint8_t *value_len);

/*============================================================================
 *                              专用TLV编码 (中层)
 *===========================================================================*/

/* 编码 0x01 实时传感器数据 */
uint8_t proto_tlv_sensor_realtime(uint8_t *buf, uint16_t buf_size,
                                  int16_t temp_c, uint16_t hum_rh,
                                  uint16_t co_ppm, uint16_t batt_mv, uint8_t alarm);

/* 编码 0x10 历史数据块 */
uint8_t proto_tlv_history_block(uint8_t *buf, uint16_t buf_size,
                                uint16_t hist_seq, uint16_t timestamp,
                                int16_t temp_c, uint16_t hum_rh,
                                uint16_t co_ppm, uint16_t batt_mv, uint8_t alarm);

/* 编码 0xAC ACK信息 (可选SuggestPower) */
uint8_t proto_tlv_ack_info(uint8_t *buf, uint16_t buf_size,
                           uint16_t last_in_seq, int8_t rssi, int8_t snr,
                           int8_t gw_tx_power,
                           bool has_suggest, uint8_t suggest_power);

/* 解析 0xAC ACK信息 */
bool proto_parse_tlv_ack(const uint8_t *value, uint8_t len, proto_ack_info_t *ack);

/* 编码 0x40 Beacon信息 (type + power + gw_id + hops + flags + 可选load) */
uint8_t proto_tlv_beacon(uint8_t *buf, uint16_t buf_size,
                         uint8_t beacon_type, int8_t tx_power,
                         uint16_t gateway_id, uint8_t hops, uint8_t flags,
                         bool has_load, uint8_t load_factor);

/* 编码 0x35 节点角色声明 */
uint8_t proto_tlv_node_role(uint8_t *buf, uint16_t buf_size, uint8_t role);

/* 编码 0x36 链路探测 */
uint8_t proto_tlv_link_probe(uint8_t *buf, uint16_t buf_size, uint8_t probe_type);

/* 编码 0x34 GNSS位置 (注册帧附带, 11字节) */
uint8_t proto_tlv_gnss_location(uint8_t *buf, uint16_t buf_size,
                                int8_t lat_dir, uint8_t lat_deg, uint8_t lat_min, uint16_t lat_frac,
                                int8_t lon_dir, uint16_t lon_deg, uint8_t lon_min, uint16_t lon_frac);

/* 编码 0x32 分配短地址 */
uint8_t proto_tlv_assign_short_addr(uint8_t *buf, uint16_t buf_size, uint16_t addr);

/* 编码 0xFF 指令执行状态 */
uint8_t proto_tlv_cmd_status(uint8_t *buf, uint16_t buf_size, uint8_t status_code);

/* ─── 下行指令专用编码 ─── */
uint8_t proto_tlv_set_measure_interval(uint8_t *buf, uint16_t buf_size,
    uint16_t normal_s, uint16_t warning_s, uint16_t alarm_s);
uint8_t proto_tlv_set_tx_interval(uint8_t *buf, uint16_t buf_size,
    uint16_t normal_s, uint16_t warning_s, uint16_t alarm_s);
uint8_t proto_tlv_set_temp(uint8_t *buf, uint16_t buf_size,
    int16_t warn, int16_t warn_rec, int16_t alarm, int16_t alarm_rec);
uint8_t proto_tlv_set_co(uint8_t *buf, uint16_t buf_size,
    uint16_t warn_ppm, uint16_t recovery_ppm);
uint8_t proto_tlv_set_humidity(uint8_t *buf, uint16_t buf_size,
    uint16_t warn_rh, uint16_t recovery_rh);
uint8_t proto_tlv_set_rf(uint8_t *buf, uint16_t buf_size,
    uint8_t sf_primary, uint8_t sf_backup, uint8_t default_power);

/* ─── 上行配置上报 ─── */

/** 编码 TLV 0x0F 全配置上报 (28B value, 2B头 = 30B 总长) */
uint8_t proto_tlv_config_report(uint8_t *buf, uint16_t buf_size,
    uint16_t meas_norm, uint16_t meas_warn, uint16_t meas_alarm,
    uint16_t tx_norm, uint16_t tx_warn, uint16_t tx_alarm,
    int16_t temp_warn, int16_t temp_warn_rec,
    int16_t temp_alarm, int16_t temp_recovery,
    uint16_t co_warn, uint16_t co_recovery,
    uint16_t hum_warn, uint16_t hum_recovery);

/*============================================================================
 *                          下行指令解析 (终端收到ACK帧后调用)
 *===========================================================================*/

/** 单条下行指令的解析结果 */
typedef struct {
    uint8_t type;                               /* TLV类型 */
    const uint8_t *value;                       /* Value指针 (指向帧内) */
    uint8_t len;                                /* Value长度 */
} proto_cmd_t;

/** 从帧payload中提取所有指令TLV (0x20-0x2A), 返回指令条数 */
uint8_t proto_parse_commands(const uint8_t *payload, uint16_t payload_len,
    proto_cmd_t *out_cmds, uint8_t max_cmds);

/*============================================================================
 *                              功率等级转换
 *===========================================================================*/

/* 功率档位(0~4) → dBm */
int8_t proto_power_level_to_dbm(uint8_t level);

/* dBm → 最接近的不超过该值的功率档位 */
uint8_t proto_dbm_to_power_level(int8_t dbm);

/*============================================================================
 *                              重传队列接口
 *===========================================================================*/

/* 初始化队列 (清空) */
void proto_retransmit_init(void);

/* 推入一条待确认记录 (发送时调用) */
bool proto_retransmit_push(const proto_retransmit_entry_t *entry);

/* 根据LastInSeq移除已确认的条目 */
void proto_retransmit_ack(uint16_t last_in_seq);

/* 获取最早未确认的条目 */
bool proto_retransmit_oldest(proto_retransmit_entry_t *entry);

/* 按索引获取条目 (0=最旧, count-1=最新), 用于遍历队列 */
bool proto_retransmit_get(uint16_t index, proto_retransmit_entry_t *entry);

/* 队列中待确认条目数 */
uint16_t proto_retransmit_count(void);

/*============================================================================
 *                              Beacon 候选管理
 *===========================================================================*/

/* 清空候选列表 */
void proto_candidates_clear(void);

/* 添加一个Beacon候选 (扫描时每收到一个Beacon调用一次) */
bool proto_candidates_add(uint16_t sender_addr, uint8_t beacon_type,
                          int8_t tx_power, uint16_t gateway_id,
                          uint8_t hop_count, bool is_valid,
                          uint8_t load_factor, int16_t rssi, int8_t snr);

/**
 * @brief  从候选列表中选择最佳父节点
 * @note   选择策略: IsValid → 网关优先 → 短路经 → 低负载 → 强RSSI
 * @param  require_valid: 是否只选 IsValid=true 的候选 (终端应传true)
 * @param  current_hops: 当前节点跳数, 未注册传255
 * @param  out_addr: 输出选中的父节点地址
 */
bool proto_candidates_select(int8_t my_max_power, int8_t sf_sensitivity,
                             int8_t margin, bool require_valid,
                             uint8_t current_hops, uint16_t *out_addr);

/* 候选列表条目数 */
uint8_t proto_candidates_count(void);

/* 按索引获取候选条目 */
const proto_beacon_candidate_t *proto_candidates_get(uint8_t index);

/* 排除指定地址的候选 (注册失败后调用, 下次select跳过此候选) */
void proto_candidates_exclude(uint16_t addr);

/* 重置所有排除标记 (新一轮扫描开始时调用) */
void proto_candidates_reset_exclusions(void);

/*============================================================================
 *                          高层接口 — 供 app_main 直接调用
 *===========================================================================*/

/** 传感器数据输入 (由app_main采集后传入) */
typedef struct {
    int16_t  temperature;       /* 温度 (×10, 即0.1°C) */
    uint16_t humidity;          /* 湿度 (×10, 即0.1%RH) */
    uint16_t co_concentration;  /* CO浓度 (ppm) */
    uint16_t battery_mv;        /* 电池电压 (mV) */
    uint8_t  alarm_flag;        /* 报警标志位 */
} proto_sensor_data_t;

/** GNSS位置输入 (由app_main从EEPROM加载后传入) */
typedef struct {
    int8_t   lat_dir;           /* 纬度方向 'N'/'S' */
    uint8_t  lat_deg, lat_min;  /* 纬度 度/分 */
    uint16_t lat_frac;          /* 纬度 分小数 */
    int8_t   lon_dir;           /* 经度方向 'E'/'W' */
    uint16_t lon_deg;           /* 经度 度 */
    uint8_t  lon_min;           /* 经度 分 */
    uint16_t lon_frac;          /* 经度 分小数 */
} proto_gnss_location_t;

/** RX分发结果码 — app_main根据此枚举决定状态跳转 */
typedef enum {
    PROTO_RX_NOTHING = 0,       /* 无有效处理 (非协议相关帧) */
    PROTO_RX_ACK_CONFIRMED,     /* ACK确认 (重传队列已自动更新) */
    PROTO_RX_PROXY_ACK,         /* 代理ACK (中继发), 需扩展RX窗口等待最终确认 */
    PROTO_RX_REGISTERED,        /* 注册成功 (短地址已存, 取 proto_get_assigned_addr()) */
    PROTO_RX_BEACON,            /* Beacon帧 (已自动加入候选列表) */
    PROTO_RX_PARSE_ERROR,       /* 帧解析失败 */
} proto_rx_action_t;

/*============================================================================
 *                          高层帧构建 (app_main发送前调用)
 *===========================================================================*/

/**
 * @brief  构建完整注册帧
 * @note   内部组装 TLV 0x30(UID) + 0x31(能力) + 0x35(角色) + 可选0x34(位置)
 * @param  location: NULL则不加位置TLV
 * @retval 帧长度 (header+payload), 0=缓冲区不足
 */
uint16_t proto_build_register_frame(uint8_t *buf, uint16_t buf_size,
    const uint8_t uid[12], const proto_gnss_location_t *location,
    uint16_t dest_addr, uint8_t power_level);

/**
 * @brief  构建完整传感器数据帧
 * @note   内部组装 TLV 0x01(实时数据) + 多个 TLV 0x10(历史补发, 遍历重传队列填满) + 可选的 extra 数据
 * @param  extra_tlv/extra_len: 调用者预构建的附加TLV (配置上报/CMD状态等), 传NULL表示无
 * @param  seq: 消息序列号 (调用者需提前调用 app_config_next_msg_seq)
 * @retval 帧长度, 0=失败
 */
uint16_t proto_build_data_frame(uint8_t *buf, uint16_t buf_size,
    const proto_sensor_data_t *sensor,
    uint16_t dest_addr, uint16_t sender_addr,
    uint16_t seq, uint8_t sf_flag, uint8_t power_level,
    const uint8_t *extra_tlv, uint8_t extra_len);

/*============================================================================
 *                          高层RX分发 (app_main收到帧后调用)
 *===========================================================================*/

/**
 * @brief  统一处理接收到的帧
 * @note   内部自动完成: Beacon候选收集 / ACK重传队列更新 / 注册短地址提取
 * @param  rssi/snr: 接收信号质量
 * @retval proto_rx_action_t 动作码, app_main据此决定下一步
 */
proto_rx_action_t proto_dispatch_rx(const uint8_t *buf, uint16_t len,
    int16_t rssi, int8_t snr);

/* 获取上次ACK的详细信息 (仅PROTO_RX_ACK_CONFIRMED后有效) */
const proto_ack_info_t *proto_get_last_ack(void);

/* 获取注册分配的短地址 (仅PROTO_RX_REGISTERED后有效) */
uint16_t proto_get_assigned_addr(void);

/*============================================================================
 *                              配置持久化 (示例结构, 由app_config调用)
 *===========================================================================*/
typedef struct {
    uint16_t short_addr;                       /* 本机短地址 */
    uint16_t parent_addr;                      /* 父节点短地址 */
    uint8_t  sf_primary;                       /* 主SF */
    uint8_t  sf_backup;                        /* 备用SF */
    uint8_t  sf_current;                       /* 当前使用SF */
    uint8_t  power_level;                      /* 当前功率档位 */
    uint16_t msg_seq;                          /* 消息序列号 */
    uint8_t  consecutive_loss;                 /* 连续丢失计数 */
    bool     has_short_addr;                   /* 是否有有效短地址 */
} proto_persistent_t;

#endif /* LORA_PROTOCOL_H_ */
