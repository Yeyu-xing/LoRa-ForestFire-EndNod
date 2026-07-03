/**
  ******************************************************************************
  * @file    driver_eeprom.c
  * @brief   BL24C256A EEPROM驱动实现
  ******************************************************************************
  */

#include "driver_eeprom.h"
#include "i2c.h"

/*============================================================================
 *                              私有函数
 *===========================================================================*/
/**
  * @brief  发送16位地址
  * @param  addr: 地址
  * @retval HAL状态
  */
static HAL_StatusTypeDef eeprom_send_addr(uint16_t addr)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(addr >> 8);   // 高字节
    buf[1] = (uint8_t)(addr & 0xFF); // 低字节
    
    return HAL_I2C_Master_Transmit(&hi2c1, EEPROM_I2C_ADDR << 1, buf, 2, EEPROM_I2C_TIMEOUT_MS);
}

/*============================================================================
 *                              基础操作
 *===========================================================================*/
bool eeprom_is_present(void)
{
    // 检测设备是否响应ACK
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, EEPROM_I2C_ADDR << 1, 1, EEPROM_I2C_TIMEOUT_MS);
    return (status == HAL_OK);
}

bool eeprom_wait_ready(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    
    // ACK轮询：写入完成后EEPROM才会响应ACK
    while ((HAL_GetTick() - start) < timeout_ms) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, EEPROM_I2C_ADDR << 1, 1, 10) == HAL_OK) {
            return true;
        }
    }
    
    return false;
}

/*============================================================================
 *                              单字节操作
 *===========================================================================*/
bool eeprom_write_byte(uint16_t addr, uint8_t data)
{
    uint8_t buf[3];
    
    // 检查地址范围
    if (addr >= EEPROM_SIZE) {
        return false;
    }
    
    // 构造数据: [地址高字节, 地址低字节, 数据]
    buf[0] = (uint8_t)(addr >> 8);
    buf[1] = (uint8_t)(addr & 0xFF);
    buf[2] = data;
    
    // 发送写入命令
    if (HAL_I2C_Master_Transmit(&hi2c1, EEPROM_I2C_ADDR << 1, buf, 3, EEPROM_I2C_TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    
    // 等待写周期完成
    return eeprom_wait_ready(EEPROM_WRITE_CYCLE_MS + 10);
}

bool eeprom_read_byte(uint16_t addr, uint8_t *data)
{
    if (data == NULL || addr >= EEPROM_SIZE) {
        return false;
    }
    
    // 发送要读取的地址
    if (eeprom_send_addr(addr) != HAL_OK) {
        return false;
    }
    
    // 读取数据
    return (HAL_I2C_Master_Receive(&hi2c1, EEPROM_I2C_ADDR << 1, data, 1, EEPROM_I2C_TIMEOUT_MS) == HAL_OK);
}

/*============================================================================
 *                              多字节操作
 *===========================================================================*/
bool eeprom_write(uint16_t addr, const uint8_t *data, uint16_t len)
{
    uint16_t remaining = len;
    uint16_t current_addr = addr;
    const uint8_t *current_data = data;
    
    if (data == NULL || len == 0) {
        return false;
    }
    
    // 检查地址范围
    if ((uint32_t)addr + len > EEPROM_SIZE) {
        return false;
    }
    
    while (remaining > 0) {
        // 计算当前页剩余空间
        uint16_t page_offset = current_addr & EEPROM_PAGE_MASK;
        uint16_t page_remaining = EEPROM_PAGE_SIZE - page_offset;
        uint16_t write_len = (remaining < page_remaining) ? remaining : page_remaining;
        
        // 构造写入缓冲区: [地址高字节, 地址低字节, 数据...]
        uint8_t buf[2 + EEPROM_PAGE_SIZE];
        buf[0] = (uint8_t)(current_addr >> 8);
        buf[1] = (uint8_t)(current_addr & 0xFF);
        for (uint16_t i = 0; i < write_len; i++) {
            buf[2 + i] = current_data[i];
        }
        
        // 发送写入命令
        if (HAL_I2C_Master_Transmit(&hi2c1, EEPROM_I2C_ADDR << 1, buf, 2 + write_len, EEPROM_I2C_TIMEOUT_MS) != HAL_OK) {
            return false;
        }
        
        // 等待写周期完成
        if (!eeprom_wait_ready(EEPROM_WRITE_CYCLE_MS + 10)) {
            return false;
        }
        
        // 更新指针
        current_addr += write_len;
        current_data += write_len;
        remaining -= write_len;
    }
    
    return true;
}

bool eeprom_read(uint16_t addr, uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) {
        return false;
    }
    
    // 检查地址范围
    if ((uint32_t)addr + len > EEPROM_SIZE) {
        return false;
    }
    
    // 发送起始地址
    if (eeprom_send_addr(addr) != HAL_OK) {
        return false;
    }
    
    // 连续读取数据
    return (HAL_I2C_Master_Receive(&hi2c1, EEPROM_I2C_ADDR << 1, data, len, EEPROM_I2C_TIMEOUT_MS) == HAL_OK);
}

/*============================================================================
 *                              辅助函数
 *===========================================================================*/
bool eeprom_fill(uint16_t addr, uint8_t value, uint16_t len)
{
    // 优化：按页填充
    uint8_t fill_buf[EEPROM_PAGE_SIZE];
    
    if ((uint32_t)addr + len > EEPROM_SIZE) {
        return false;
    }
    
    // 预填充缓冲区
    for (int i = 0; i < EEPROM_PAGE_SIZE; i++) {
        fill_buf[i] = value;
    }
    
    uint16_t remaining = len;
    uint16_t current_addr = addr;
    
    while (remaining > 0) {
        // 计算本次写入长度
        uint16_t page_offset = current_addr & EEPROM_PAGE_MASK;
        uint16_t page_remaining = EEPROM_PAGE_SIZE - page_offset;
        uint16_t write_len = (remaining < page_remaining) ? remaining : page_remaining;
        
        // 写入数据
        if (!eeprom_write(current_addr, fill_buf, write_len)) {
            return false;
        }
        
        current_addr += write_len;
        remaining -= write_len;
    }
    
    return true;
}
