/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   应用参数存储模块
  * @note    基于BL24C256A EEPROM实现参数持久化
  ******************************************************************************
  */

#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 *                              配置参数
 *===========================================================================*/
/* EEPROM存储地址分配 (BL24C256A = 32KB)
 *
 * 0x0000 - 0x00FF: 参数区 (256B)
 * 0x0100 - 0x011F: 位置数据区 (32B)
 * 0x0120 - 0x012F: 缓存管理区 (16B)
 * 0x0130 - 0x7FFF: 缓存数据区 (~32KB)
 */
#define CONFIG_EEPROM_BASE      0x0000      // 参数区起始地址
#define CONFIG_EEPROM_SIZE      256         // 参数区大小

#define LOCATION_EEPROM_BASE    0x0100      // 位置数据区起始地址
#define LOCATION_EEPROM_SIZE    32          // 位置数据区大小

#define CACHE_MGMT_BASE         0x0120      // 缓存管理区起始地址
#define CACHE_MGMT_SIZE         16          // 缓存管理区大小

#define CACHE_DATA_BASE         0x0130      // 缓存数据区起始地址
#define CACHE_DATA_SIZE         0x7ED0      // 缓存数据区大小 (~32KB)

/* 参数校验 */
#define CONFIG_MAGIC            0xAA55
#define CONFIG_VERSION          3           // V3: 添加协议相关字段

/* 默认值 */
#define DEFAULT_SHORT_ID                0x0000      // 默认短ID (从UID生成)
#define DEFAULT_MEASURE_INTERVAL_NORMAL  60         // 常规采集间隔(s)
#define DEFAULT_MEASURE_INTERVAL_WARNING 30         // 预警采集间隔(s)
#define DEFAULT_MEASURE_INTERVAL_ALARM   5          // 火警采集间隔(s)
#define DEFAULT_TRANSMIT_INTERVAL_NORMAL 20         // 常规发送间隔(s)
#define DEFAULT_TRANSMIT_INTERVAL_WARNING 30        // 预警发送间隔(s)
#define DEFAULT_TRANSMIT_INTERVAL_ALARM  5          // 火警发送间隔(s)
#define DEFAULT_TEMP_WARNING_THRESHOLD   45.0f      // 温度预警阈值(°C)
#define DEFAULT_TEMP_WARNING_RECOVERY    40.0f      // 温度预警恢复(°C)
#define DEFAULT_TEMP_ALARM_THRESHOLD     60.0f      // 温度火警阈值(°C)
#define DEFAULT_TEMP_ALARM_RECOVERY      50.0f      // 温度火警恢复(°C)
#define DEFAULT_CO_WARNING_THRESHOLD     50.0f      // CO预警阈值(ppm)
#define DEFAULT_CO_WARNING_RECOVERY      30.0f      // CO预警恢复(ppm)
#define DEFAULT_HUM_WARNING_THRESHOLD    20.0f      // 低湿预警阈值(%RH)
#define DEFAULT_HUM_WARNING_RECOVERY     30.0f      // 低湿预警恢复(%RH)
#define DEFAULT_LORA_TX_POWER            14          // 发射功率(dBm)
#define DEFAULT_LORA_SF                  7           // 扩频因子
#define DEFAULT_GNSS_EN_ACTIVE_LOW       0           // GNSS使能高电平有效

/* 协议相关默认值 */
#define DEFAULT_PROTO_PARENT_ADDR       0xFFFF      // 父节点地址(未知)
#define DEFAULT_PROTO_POWER_LEVEL       0           // 功率档位0=14dBm
#define DEFAULT_PROTO_SF_PRIMARY        7           // 主SF
#define DEFAULT_PROTO_SF_BACKUP         11          // 备用SF
#define DEFAULT_PROTO_MSG_SEQ           0           // 消息序列号初始值

/*============================================================================
 *                              运行模式
 *===========================================================================*/
typedef enum {
    RUN_MODE_NORMAL = 0,        // 常规监测
    RUN_MODE_EARLY_WARNING,     // 预警模式 (CO超标/湿度过低)
    RUN_MODE_FIRE_ALARM,        // 火警模式 (温度超高)
} run_mode_t;

/*============================================================================
 *                              参数结构
 *===========================================================================*/
typedef struct {
    uint16_t magic;                         // 校验魔数
    uint16_t version;                       // 参数版本号

    /* 设备标识 */
    uint32_t uid_hash;                      // UID的CRC32 (只读)
    uint16_t short_id;                      // 短ID (可配置)

    /* 运行参数 (单位: 秒) */
    uint16_t measure_interval_normal;       // 常规采集间隔
    uint16_t measure_interval_warning;      // 预警采集间隔
    uint16_t measure_interval_alarm;        // 火警采集间隔
    uint16_t transmit_interval_normal;      // 常规发送间隔
    uint16_t transmit_interval_warning;     // 预警发送间隔
    uint16_t transmit_interval_alarm;       // 火警发送间隔

    /* 报警阈值 (含迟滞) */
    float temp_warning_threshold;           // 温度预警阈值(°C)
    float temp_warning_recovery;            // 温度预警恢复(°C)
    float temp_alarm_threshold;             // 温度火警阈值(°C)
    float temp_alarm_recovery;              // 温度火警恢复(°C)
    float co_warning_threshold;             // CO预警阈值(ppm)
    float co_warning_recovery;              // CO预警恢复(ppm)
    float hum_warning_threshold;            // 低湿预警阈值(%RH)
    float hum_warning_recovery;             // 低湿预警恢复(%RH)

    /* LoRa参数 */
    uint8_t lora_tx_power;                  // 发射功率(dBm)
    uint8_t lora_sf;                        // 扩频因子

    /* GNSS参数 */
    uint8_t gnss_en_active_low;             // GNSS使能极性: 0=高电平有效, 1=低电平有效

    /* 协议参数 (V3) */
    uint16_t proto_short_addr;              // 本机短地址 (网关0x0000, 未注册0xFFFF)
    uint16_t proto_parent_addr;             // 父节点短地址
    uint8_t  proto_power_level;             // 功率档位 0-4
    uint8_t  proto_sf_primary;              // 主SF
    uint8_t  proto_sf_backup;               // 备用SF
    uint8_t  proto_sf_current;              // 当前SF
    uint16_t proto_msg_seq;                 // 消息序列号
    uint8_t  proto_has_short_addr;          // 是否有有效短地址 (0/1)
    uint8_t  _reserved[6];                  // 保留, 填充到整字节

    /* 校验 */
    uint16_t checksum;                      // 校验和
} app_config_t;

/*============================================================================
 *                              缓存记录结构
 *===========================================================================*/
typedef struct __attribute__((packed)) {
    uint32_t timestamp;                     // 时间戳(RTC)
    float temperature;                      // 温度(°C)
    float humidity;                         // 湿度(%RH)
    float co_concentration;                 // CO浓度(ppm)
    uint16_t battery_mv;                    // 电池电压(mV)
    uint8_t alarm_flag;                     // 报警标志 (run_mode_t)
    uint8_t retry_count;                    // 重试次数
} cache_record_t;                           // 约20字节

/* 缓存管理结构 */
typedef struct __attribute__((packed)) {
    uint16_t head;                          // 队列头索引
    uint16_t tail;                          // 队列尾索引
    uint16_t count;                         // 当前记录数
    uint16_t max_records;                   // 最大记录数
} cache_mgmt_t;

/*============================================================================
 *                              报警标志位
 *===========================================================================*/
#define ALARM_FLAG_NONE         0x00
#define ALARM_FLAG_CO           0x01        // CO预警
#define ALARM_FLAG_HUMIDITY     0x02        // 低湿预警
#define ALARM_FLAG_TEMPERATURE  0x04        // 温度火警

/*============================================================================
 *                              接口函数
 *===========================================================================*/
/**
  * @brief  初始化配置模块
  * @note   从EEPROM加载参数，若无效则使用默认值
  * @retval true: 成功, false: EEPROM不可用
  */
bool app_config_init(void);

/**
  * @brief  获取当前配置 (只读)
  * @retval 配置结构指针
  */
const app_config_t *app_config_get(void);

/**
  * @brief  修改并保存配置
  * @param  new_config: 新配置
  * @retval true: 成功, false: 失败
  */
bool app_config_set(const app_config_t *new_config);

/**
  * @brief  恢复默认配置并保存
  * @retval true: 成功
  */
bool app_config_reset(void);

/**
  * @brief  根据运行模式获取采集间隔
  * @param  mode: 运行模式
  * @retval 间隔(秒)
  */
uint16_t app_config_get_measure_interval(run_mode_t mode);

/**
  * @brief  根据运行模式获取发送间隔
  * @param  mode: 运行模式
  * @retval 间隔(秒)
  */
uint16_t app_config_get_transmit_interval(run_mode_t mode);

/*============================================================================
 *                              缓存队列接口
 *===========================================================================*/
/**
  * @brief  初始化缓存队列
  * @retval true: 成功
  */
bool app_cache_init(void);

/**
  * @brief  推入一条缓存记录
  * @param  record: 记录指针
  * @retval true: 成功, false: 队列满
  */
bool app_cache_push(const cache_record_t *record);

/**
  * @brief  弹出一条缓存记录 (最早入队的)
  * @param  record: 输出记录指针
  * @retval true: 成功, false: 队列空
  */
bool app_cache_pop(cache_record_t *record);

/**
  * @brief  获取缓存队列中的记录数
  * @retval 记录数
  */
uint16_t app_cache_count(void);

/**
  * @brief  清空缓存队列
  * @retval true: 成功
  */
bool app_cache_clear(void);

/*============================================================================
 *                              位置数据EEPROM存储
 *===========================================================================*/
/* 位置数据结构 (存储在EEPROM中，部署后持久化) */
typedef struct __attribute__((packed)) {
    uint16_t magic;                         // 校验魔数
    int8_t   lat_dir;                       // 纬度方向: 'N'/'S'
    uint8_t  lat_deg;                       // 纬度度
    uint8_t  lat_min;                       // 纬度分
    uint16_t lat_frac;                      // 纬度分小数
    int8_t   lon_dir;                       // 经度方向: 'E'/'W'
    uint16_t lon_deg;                       // 经度度
    uint8_t  lon_min;                       // 经度分
    uint16_t lon_frac;                      // 经度分小数
    uint16_t checksum;                      // 校验和
} gnss_stored_location_t;

#define LOCATION_MAGIC          0x55AA

/*============================================================================
 *                              位置数据接口
 *===========================================================================*/
/**
  * @brief  保存位置数据到EEPROM
  * @param  loc: 位置数据指针
  * @retval true: 成功
  */
bool app_config_save_location(const gnss_stored_location_t *loc);

/**
  * @brief  从EEPROM加载位置数据
  * @param  loc: 输出位置数据指针
  * @retval true: 数据有效
  */
bool app_config_load_location(gnss_stored_location_t *loc);

/**
  * @brief  检查EEPROM中是否有有效的位置数据
  * @retval true: 有有效位置
  */
bool app_config_has_location(void);

/**
 * @brief  清除EEPROM中的位置数据 (重新部署时调用)
 * @retval true: 成功
 */
bool app_config_clear_location(void);

/*============================================================================
 *                              协议参数接口
 *===========================================================================*/
/**
 * @brief  获取协议持久化数据 (只读)
 */
const app_config_t *app_config_get_proto(void);

/**
 * @brief  更新协议持久化数据 (短地址/父节点/SF/功率/序列号等)
 * @param  config: 新配置 (仅proto_*字段会被参考, 其他保持)
 * @retval true: 成功
 */
bool app_config_set_proto(const app_config_t *proto_cfg);

/**
 * @brief  分配下一个消息序列号 (递增并保存)
 * @retval 序列号
 */
uint16_t app_config_next_msg_seq(void);

/**
 * @brief  设置短地址 (注册成功后调用, 标记has_short_addr)
 * @param  addr: 分配的短地址
 * @retval true: 成功
 */
bool app_config_set_short_addr(uint16_t addr);

/**
 * @brief  是否有有效短地址
 */
bool app_config_has_short_addr(void);

/**
 * @brief  清除短地址 (重新部署时)
 */
bool app_config_clear_short_addr(void);

/**
 * @brief  设置短地址仅到RAM, 不写入EEPROM (调试阶段使用)
 */
void app_config_set_short_addr_ram(uint16_t addr);

#endif /* APP_CONFIG_H_ */
