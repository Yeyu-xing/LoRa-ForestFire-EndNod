/**
  ******************************************************************************
  * @file    test_oled.h
  * @brief   OLED屏幕测试 - 排查显示不亮问题
  ******************************************************************************
  */

#ifndef TEST_OLED_H_
#define TEST_OLED_H_

/**
  * @brief  启动OLED测试
  * @note   通过I2C2扫描设备、初始化OLED、显示测试图案
  *         通过UART2输出调试信息
  */
void test_oled_start(void);

/**
  * @brief  OLED测试周期处理
  * @note   在主循环中调用，周期性刷新测试画面
  */
void test_oled_process(void);

#endif /* TEST_OLED_H_ */
