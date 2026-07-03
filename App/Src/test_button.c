/**
  ******************************************************************************
  * @file    test_button.c
  * @brief   按键驱动单元测试 - 通过UART2输出事件信息
  ******************************************************************************
  */

#include "test_button.h"

#include <stdio.h>

#include "driver_button.h"
#include "usart.h"
#include "tim.h"
#include <string.h>

/*============================================================================
 *                              私有函数
 *===========================================================================*/
/**
  * @brief  通过UART2发送字符串
  * @param  str: 要发送的字符串
  */
static void uart2_print(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
}

/**
  * @brief  按键事件回调函数
  * @param  btn: 按键ID
  * @param  event: 事件类型
  */
static void button_event_callback(board_btn_t btn, btn_event_t event)
{
    const char *btn_name;
    const char *event_name;
    
    // 获取按键名称
    switch (btn) {
        case BTN_UP:   btn_name = "UP";   break;
        case BTN_OK:   btn_name = "OK";   break;
        case BTN_DOWN: btn_name = "DOWN"; break;
        default:       btn_name = "?";    return;
    }
    
    // 获取事件名称
    switch (event) {
        case BTN_EVENT_PRESS:       event_name = "PRESS";       break;
        case BTN_EVENT_RELEASE:     event_name = "RELEASE";     break;
        case BTN_EVENT_SHORT_CLICK: event_name = "SHORT_CLICK"; break;
        case BTN_EVENT_LONG_PRESS:  event_name = "LONG_PRESS";  break;
        default:                    event_name = "?";           return;
    }
    
    // 格式化并发送消息
    char msg[64];
    snprintf(msg, sizeof(msg), "[BTN_%s] %s\r\n", btn_name, event_name);
    uart2_print(msg);
}

/*============================================================================
 *                              测试启动
 *===========================================================================*/
void test_button_start(void)
{
    // 发送测试开始提示
    uart2_print("\r\n=== Button Driver Test ===\r\n");
    uart2_print("Press buttons to see events...\r\n\r\n");
    
    // 初始化按键驱动
    btn_driver_init();
    
    // 注册回调函数
    btn_register_callback(button_event_callback);
    
    // 启动TIM17定时器（用于消抖和长按检测）
    HAL_TIM_Base_Start_IT(&htim17);
}
