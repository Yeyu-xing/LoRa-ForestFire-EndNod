/**
  ******************************************************************************
  * @file    test_button.h
  * @brief   按键驱动单元测试 - 通过UART2输出事件信息
  ******************************************************************************
  */

#ifndef TEST_BUTTON_H_
#define TEST_BUTTON_H_

/**
  * @brief  启动按键测试
  * @note   初始化按键驱动，注册回调，启动TIM17
  *         按键事件会通过UART2发送提示信息
  */
void test_button_start(void);

#endif /* TEST_BUTTON_H_ */
