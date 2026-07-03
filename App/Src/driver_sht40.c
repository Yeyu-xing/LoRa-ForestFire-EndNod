/**
  ******************************************************************************
  * @file    driver_sht40.c
  * @brief   SHT40温湿度传感器驱动实现
  ******************************************************************************
  */

#include "driver_sht40.h"
#include "i2c.h"

/*============================================================================
 *                              私有常量
 *===========================================================================*/
/* 测量等待时间表 */
static const uint16_t measure_time_ms[] = {
    SHT40_MEASURE_TIME_HIGH_MS,
    SHT40_MEASURE_TIME_MED_MS,
    SHT40_MEASURE_TIME_LOW_MS
};

/* 测量命令表 */
static const uint8_t measure_cmd[] = {
    SHT40_CMD_HIGH_PRECISION,
    SHT40_CMD_MED_PRECISION,
    SHT40_CMD_LOW_PRECISION
};

/*============================================================================
 *                              私有函数
 *===========================================================================*/
/**
  * @brief  CRC-8校验计算
  * @param  data: 数据指针
  * @param  len: 数据长度
  * @retval CRC-8值
  * @note   多项式: 0x31, 初始值: 0xFF
  */
static uint8_t sht40_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = crc << 1;
            }
        }
    }
    
    return crc;
}

/**
  * @brief  发送命令
  * @param  cmd: 命令字节
  * @retval HAL状态
  */
static HAL_StatusTypeDef sht40_send_cmd(uint8_t cmd)
{
    return HAL_I2C_Master_Transmit(&hi2c1, SHT40_I2C_ADDR << 1, &cmd, 1, SHT40_I2C_TIMEOUT_MS);
}

/**
  * @brief  读取数据
  * @param  buf: 数据缓冲区
  * @param  len: 数据长度
  * @retval HAL状态
  */
static HAL_StatusTypeDef sht40_read_data(uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Master_Receive(&hi2c1, SHT40_I2C_ADDR << 1, buf, len, SHT40_I2C_TIMEOUT_MS);
}

/*============================================================================
 *                              公开函数
 *===========================================================================*/
bool sht40_init(void)
{
    // 通过软复位检测传感器是否存在
    return sht40_reset();
}

bool sht40_is_present(void)
{
    // 尝试向传感器发送命令，检查ACK响应
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, SHT40_I2C_ADDR << 1, 1, SHT40_I2C_TIMEOUT_MS);
    return (status == HAL_OK);
}

bool sht40_read(sht40_data_t *data, sht40_precision_t precision)
{
    uint8_t buf[6];
    uint16_t temp_raw, hum_raw;
    
    if (data == NULL || precision > SHT40_PRECISION_LOW) {
        return false;
    }
    
    // 发送测量命令
    if (sht40_send_cmd(measure_cmd[precision]) != HAL_OK) {
        data->valid = false;
        return false;
    }
    
    // 等待测量完成
    HAL_Delay(measure_time_ms[precision]);
    
    // 读取6字节数据: [T_MSB, T_LSB, CRC_T, RH_MSB, RH_LSB, CRC_RH]
    if (sht40_read_data(buf, 6) != HAL_OK) {
        data->valid = false;
        return false;
    }
    
    // CRC校验 - 温度数据
    if (sht40_crc8(buf, 2) != buf[2]) {
        data->valid = false;
        return false;
    }
    
    // CRC校验 - 湿度数据
    if (sht40_crc8(buf + 3, 2) != buf[5]) {
        data->valid = false;
        return false;
    }
    
    // 转换原始值
    temp_raw = ((uint16_t)buf[0] << 8) | buf[1];
    hum_raw = ((uint16_t)buf[3] << 8) | buf[4];
    
    // 计算温度: T = -45 + 175 * (S_T / 2^16-1)
    data->temperature = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);
    
    // 计算湿度: RH = -6 + 125 * (S_RH / 2^16-1)
    data->humidity = -6.0f + 125.0f * ((float)hum_raw / 65535.0f);
    
    // 限制湿度范围 [0, 100]
    if (data->humidity < 0.0f) data->humidity = 0.0f;
    if (data->humidity > 100.0f) data->humidity = 100.0f;
    
    data->valid = true;
    return true;
}

bool sht40_read_serial(uint32_t *serial)
{
    uint8_t buf[6];
    
    if (serial == NULL) {
        return false;
    }
    
    // 发送读取序列号命令
    if (sht40_send_cmd(SHT40_CMD_READ_SERIAL) != HAL_OK) {
        return false;
    }
    
    // 等待处理完成
    HAL_Delay(1);
    
    // 读取6字节: [SN_MSB, SN_LSB, CRC, SN_MSB, SN_LSB, CRC]
    if (sht40_read_data(buf, 6) != HAL_OK) {
        return false;
    }
    
    // CRC校验（可选）
    // 序列号由两个16位数据组成
    *serial = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
              ((uint32_t)buf[3] << 8) | buf[4];
    
    return true;
}

bool sht40_reset(void)
{
    if (sht40_send_cmd(SHT40_CMD_SOFT_RESET) != HAL_OK) {
        return false;
    }
    
    // 等待复位完成
    HAL_Delay(1);
    
    return true;
}
