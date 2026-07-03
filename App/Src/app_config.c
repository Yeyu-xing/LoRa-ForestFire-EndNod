/**
  ******************************************************************************
  * @file    app_config.c
  * @brief   应用参数存储模块实现
  ******************************************************************************
  */

#include "app_config.h"

#include <string.h>
#include <stddef.h>

#include "driver_eeprom.h"
#include "board_io.h"
#include "lora_protocol.h"

/*============================================================================
 *                              私有变量
 *===========================================================================*/
static app_config_t current_config;

/* 单条缓存记录大小 */
#define CACHE_RECORD_SIZE   sizeof(cache_record_t)

/* 缓存区最大记录数 */
#define CACHE_MAX_RECORDS   (CACHE_DATA_SIZE / CACHE_RECORD_SIZE)

/*============================================================================
 *                              私有函数 - CRC32
 *===========================================================================*/
/**
  * @brief  计算CRC32 (用于UID哈希)
  * @param  data: 数据指针
  * @param  len: 数据长度
  * @retval CRC32值
  */
static uint32_t crc32_calc(const uint8_t *data, uint16_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return ~crc;
}

/*============================================================================
 *                              私有函数 - 校验和
 *===========================================================================*/
/**
  * @brief  计算参数结构校验和 (排除checksum字段自身)
  * @param  config: 配置结构指针
  * @retval 校验和
  */
static uint16_t config_calc_checksum(const app_config_t *config)
{
    uint16_t sum = 0;
    const uint8_t *p = (const uint8_t *)config;
    uint16_t len = offsetof(app_config_t, checksum);
    
    for (uint16_t i = 0; i < len; i++) {
        sum += p[i];
    }
    
    return sum;
}

/*============================================================================
 *                              私有函数 - 生成UID哈希
 *===========================================================================*/
static uint32_t config_generate_uid_hash(void)
{
    uint8_t uid[12];
    board_get_uid(uid);
    return crc32_calc(uid, 12);
}

/*============================================================================
 *                              私有函数 - 填充默认配置
 *===========================================================================*/
static void config_fill_defaults(app_config_t *config)
{
    config->magic = CONFIG_MAGIC;
    config->version = CONFIG_VERSION;
    
    config->uid_hash = config_generate_uid_hash();
    config->short_id = (uint16_t)(config->uid_hash & 0xFFFF);
    
    config->measure_interval_normal = DEFAULT_MEASURE_INTERVAL_NORMAL;
    config->measure_interval_warning = DEFAULT_MEASURE_INTERVAL_WARNING;
    config->measure_interval_alarm = DEFAULT_MEASURE_INTERVAL_ALARM;
    config->transmit_interval_normal = DEFAULT_TRANSMIT_INTERVAL_NORMAL;
    config->transmit_interval_warning = DEFAULT_TRANSMIT_INTERVAL_WARNING;
    config->transmit_interval_alarm = DEFAULT_TRANSMIT_INTERVAL_ALARM;
    
    config->temp_warning_threshold = DEFAULT_TEMP_WARNING_THRESHOLD;
    config->temp_warning_recovery = DEFAULT_TEMP_WARNING_RECOVERY;
    config->temp_alarm_threshold = DEFAULT_TEMP_ALARM_THRESHOLD;
    config->temp_alarm_recovery = DEFAULT_TEMP_ALARM_RECOVERY;
    config->co_warning_threshold = DEFAULT_CO_WARNING_THRESHOLD;
    config->co_warning_recovery = DEFAULT_CO_WARNING_RECOVERY;
    config->hum_warning_threshold = DEFAULT_HUM_WARNING_THRESHOLD;
    config->hum_warning_recovery = DEFAULT_HUM_WARNING_RECOVERY;
    
    config->lora_tx_power = DEFAULT_LORA_TX_POWER;
    config->lora_sf = DEFAULT_LORA_SF;

    config->gnss_en_active_low = DEFAULT_GNSS_EN_ACTIVE_LOW;

    /* 协议参数 (V3) */
    config->proto_short_addr      = DEFAULT_PROTO_PARENT_ADDR;
    config->proto_parent_addr     = DEFAULT_PROTO_PARENT_ADDR;
    config->proto_power_level     = DEFAULT_PROTO_POWER_LEVEL;
    config->proto_sf_primary      = DEFAULT_PROTO_SF_PRIMARY;
    config->proto_sf_backup       = DEFAULT_PROTO_SF_BACKUP;
    config->proto_sf_current      = DEFAULT_PROTO_SF_PRIMARY;
    config->proto_msg_seq         = DEFAULT_PROTO_MSG_SEQ;
    config->proto_has_short_addr  = 0;
    memset(config->_reserved, 0, sizeof(config->_reserved));

    config->checksum = config_calc_checksum(config);
}

/*============================================================================
 *                              参数接口实现
 *===========================================================================*/
bool app_config_init(void)
{
    // 检查EEPROM
    if (!eeprom_is_present()) {
        // EEPROM不可用，使用默认值
        config_fill_defaults(&current_config);
        return false;
    }
    
    // 从EEPROM读取参数
    if (!eeprom_read(CONFIG_EEPROM_BASE, (uint8_t *)&current_config, sizeof(app_config_t))) {
        config_fill_defaults(&current_config);
        return false;
    }
    
    // 校验魔数和版本
    if (current_config.magic != CONFIG_MAGIC || current_config.version != CONFIG_VERSION) {
        config_fill_defaults(&current_config);
        app_config_set(&current_config);
        return true;
    }
    
    // 校验和验证
    if (config_calc_checksum(&current_config) != current_config.checksum) {
        config_fill_defaults(&current_config);
        app_config_set(&current_config);
        return true;
    }
    
    // UID哈希不可修改，始终从硬件重新生成
    current_config.uid_hash = config_generate_uid_hash();
    
    return true;
}

const app_config_t *app_config_get(void)
{
    return &current_config;
}

bool app_config_set(const app_config_t *new_config)
{
    if (new_config == NULL) return false;
    
    // 复制配置
    app_config_t tmp = *new_config;
    
    // 强制保持UID哈希
    tmp.uid_hash = config_generate_uid_hash();
    
    // 更新校验和
    tmp.checksum = config_calc_checksum(&tmp);
    
    // 写入EEPROM
    if (!eeprom_write(CONFIG_EEPROM_BASE, (const uint8_t *)&tmp, sizeof(app_config_t))) {
        return false;
    }
    
    // 更新内存副本
    current_config = tmp;
    
    return true;
}

bool app_config_reset(void)
{
    config_fill_defaults(&current_config);
    return app_config_set(&current_config);
}

uint16_t app_config_get_measure_interval(run_mode_t mode)
{
    switch (mode) {
        case RUN_MODE_FIRE_ALARM:    return current_config.measure_interval_alarm;
        case RUN_MODE_EARLY_WARNING: return current_config.measure_interval_warning;
        default:                     return current_config.measure_interval_normal;
    }
}

uint16_t app_config_get_transmit_interval(run_mode_t mode)
{
    switch (mode) {
        case RUN_MODE_FIRE_ALARM:    return current_config.transmit_interval_alarm;
        case RUN_MODE_EARLY_WARNING: return current_config.transmit_interval_warning;
        default:                     return current_config.transmit_interval_normal;
    }
}

/*============================================================================
 *                              缓存队列实现
 *===========================================================================*/
bool app_cache_init(void)
{
    cache_mgmt_t mgmt;
    
    if (!eeprom_is_present()) return false;
    
    // 读取管理信息
    if (!eeprom_read(CACHE_MGMT_BASE, (uint8_t *)&mgmt, sizeof(mgmt))) {
        return false;
    }
    
    // 如果管理信息无效，初始化
    if (mgmt.max_records != CACHE_MAX_RECORDS || mgmt.count > CACHE_MAX_RECORDS) {
        mgmt.head = 0;
        mgmt.tail = 0;
        mgmt.count = 0;
        mgmt.max_records = CACHE_MAX_RECORDS;
        
        if (!eeprom_write(CACHE_MGMT_BASE, (const uint8_t *)&mgmt, sizeof(mgmt))) {
            return false;
        }
    }
    
    return true;
}

bool app_cache_push(const cache_record_t *record)
{
    cache_mgmt_t mgmt;
    
    if (record == NULL || !eeprom_is_present()) return false;
    
    // 读取管理信息
    if (!eeprom_read(CACHE_MGMT_BASE, (uint8_t *)&mgmt, sizeof(mgmt))) {
        return false;
    }
    
    // 队列满，丢弃最旧的记录
    if (mgmt.count >= mgmt.max_records) {
        mgmt.head = (mgmt.head + 1) % mgmt.max_records;
        mgmt.count--;
    }
    
    // 计算写入地址
    uint16_t addr = CACHE_DATA_BASE + mgmt.tail * CACHE_RECORD_SIZE;
    
    // 写入记录
    if (!eeprom_write(addr, (const uint8_t *)record, CACHE_RECORD_SIZE)) {
        return false;
    }
    
    // 更新管理信息
    mgmt.tail = (mgmt.tail + 1) % mgmt.max_records;
    mgmt.count++;
    
    return eeprom_write(CACHE_MGMT_BASE, (const uint8_t *)&mgmt, sizeof(mgmt));
}

bool app_cache_pop(cache_record_t *record)
{
    cache_mgmt_t mgmt;
    
    if (record == NULL || !eeprom_is_present()) return false;
    
    // 读取管理信息
    if (!eeprom_read(CACHE_MGMT_BASE, (uint8_t *)&mgmt, sizeof(mgmt))) {
        return false;
    }
    
    // 队列空
    if (mgmt.count == 0) return false;
    
    // 计算读取地址
    uint16_t addr = CACHE_DATA_BASE + mgmt.head * CACHE_RECORD_SIZE;
    
    // 读取记录
    if (!eeprom_read(addr, (uint8_t *)record, CACHE_RECORD_SIZE)) {
        return false;
    }
    
    // 更新管理信息
    mgmt.head = (mgmt.head + 1) % mgmt.max_records;
    mgmt.count--;
    
    return eeprom_write(CACHE_MGMT_BASE, (const uint8_t *)&mgmt, sizeof(mgmt));
}

uint16_t app_cache_count(void)
{
    cache_mgmt_t mgmt;
    
    if (!eeprom_is_present()) return 0;
    
    if (!eeprom_read(CACHE_MGMT_BASE, (uint8_t *)&mgmt, sizeof(mgmt))) {
        return 0;
    }
    
    return mgmt.count;
}

bool app_cache_clear(void)
{
    cache_mgmt_t mgmt = {
        .head = 0,
        .tail = 0,
        .count = 0,
        .max_records = CACHE_MAX_RECORDS,
    };
    
    return eeprom_write(CACHE_MGMT_BASE, (const uint8_t *)&mgmt, sizeof(mgmt));
}

/*============================================================================
 *                              位置数据接口实现
 *===========================================================================*/
static uint16_t location_calc_checksum(const gnss_stored_location_t *loc)
{
    uint16_t sum = 0;
    const uint8_t *p = (const uint8_t *)loc;
    uint16_t len = offsetof(gnss_stored_location_t, checksum);

    for (uint16_t i = 0; i < len; i++) {
        sum += p[i];
    }

    return sum;
}

bool app_config_save_location(const gnss_stored_location_t *loc)
{
    if (loc == NULL || !eeprom_is_present()) return false;

    gnss_stored_location_t tmp = *loc;
    tmp.magic = LOCATION_MAGIC;
    tmp.checksum = location_calc_checksum(&tmp);

    return eeprom_write(LOCATION_EEPROM_BASE, (const uint8_t *)&tmp, sizeof(gnss_stored_location_t));
}

bool app_config_load_location(gnss_stored_location_t *loc)
{
    if (loc == NULL || !eeprom_is_present()) return false;

    if (!eeprom_read(LOCATION_EEPROM_BASE, (uint8_t *)loc, sizeof(gnss_stored_location_t))) {
        return false;
    }

    if (loc->magic != LOCATION_MAGIC) return false;
    if (location_calc_checksum(loc) != loc->checksum) return false;

    return true;
}

bool app_config_has_location(void)
{
    gnss_stored_location_t loc;
    return app_config_load_location(&loc);
}

bool app_config_clear_location(void)
{
    if (!eeprom_is_present()) return false;

    // 写入无效magic来清除
    uint16_t invalid = 0;
    return eeprom_write(LOCATION_EEPROM_BASE, (const uint8_t *)&invalid, sizeof(invalid));
}

/*============================================================================
 *                              协议参数接口实现
 *===========================================================================*/
const app_config_t *app_config_get_proto(void)
{
    return &current_config;
}

bool app_config_set_proto(const app_config_t *proto_cfg)
{
    if (proto_cfg == NULL) return false;

    // 更新所有proto_*字段, 保持其他字段不变
    current_config.proto_short_addr     = proto_cfg->proto_short_addr;
    current_config.proto_parent_addr    = proto_cfg->proto_parent_addr;
    current_config.proto_power_level    = proto_cfg->proto_power_level;
    current_config.proto_sf_primary     = proto_cfg->proto_sf_primary;
    current_config.proto_sf_backup      = proto_cfg->proto_sf_backup;
    current_config.proto_sf_current     = proto_cfg->proto_sf_current;
    current_config.proto_msg_seq        = proto_cfg->proto_msg_seq;
    current_config.proto_has_short_addr = proto_cfg->proto_has_short_addr;

    return app_config_set(&current_config);
}

uint16_t app_config_next_msg_seq(void)
{
    current_config.proto_msg_seq++;
    // 保存到EEPROM (使用app_config_set保存完整配置)
    app_config_set(&current_config);
    return current_config.proto_msg_seq;
}

bool app_config_set_short_addr(uint16_t addr)
{
    current_config.proto_short_addr     = addr;
    current_config.proto_has_short_addr = 1;
    return app_config_set(&current_config);
}

bool app_config_has_short_addr(void)
{
    return (current_config.proto_has_short_addr != 0);
}

bool app_config_clear_short_addr(void)
{
    current_config.proto_short_addr     = PROTO_ADDR_BROADCAST;
    current_config.proto_has_short_addr = 0;
    current_config.proto_parent_addr    = PROTO_ADDR_BROADCAST;
    return app_config_set(&current_config);
}

void app_config_set_short_addr_ram(uint16_t addr)
{
    current_config.proto_short_addr     = addr;
    current_config.proto_has_short_addr = 1;
    /* 不调用 app_config_set(), 不写入 EEPROM */
}
