/**
  ******************************************************************************
  * @file    test_sht40.h
  * @brief   SHT40温湿度传感器测试
  ******************************************************************************
  */

#ifndef TEST_SHT40_H_
#define TEST_SHT40_H_

/**
  * @brief  启动SHT40测试
  * @note   通过UART2输出温湿度数据
  */
void test_sht40_start(void);

/**
  * @brief  周期性读取温湿度
  * @note   需在主循环或定时器中调用
  */
void test_sht40_process(void);

#endif /* TEST_SHT40_H_ */
