/**
  ******************************************************************************
  * @file    driver_button.h
  * @brief   按键驱动 - 外部中断触发 + 定时消抖
  * @note    按键通过EXTI外部中断触发，TIM17用于消抖确认
  ******************************************************************************
  */

#ifndef DRIVER_BUTTON_H_
#define DRIVER_BUTTON_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "board_io.h"

/*============================================================================
 *                              配置参数
 *                         (根据需求修改以下宏定义)
 *===========================================================================*/
/* 消抖时间 (ms)，EXTI触发后等待GPIO稳定的时间 */
#define BTN_DEBOUNCE_TIME_MS    20

/* 长按判定时间 (ms) */
#define BTN_LONG_PRESS_TIME_MS  1000

/* TIM17中断周期 (ms) */
#define BTN_TIMER_TICK_MS       1

/*============================================================================
 *                              类型定义
 *===========================================================================*/
/* 按键事件类型 */
typedef enum {
    BTN_EVENT_PRESS,        // 按下（消抖后确认）
    BTN_EVENT_RELEASE,      // 释放（消抖后确认）
    BTN_EVENT_SHORT_CLICK,  // 短按（释放时判定，且未触发长按）
    BTN_EVENT_LONG_PRESS,   // 长按（按住超过阈值时触发，只触发一次）
} btn_event_t;

/* 按键事件回调函数类型
 * @param btn: 触发事件的按键
 * @param event: 事件类型
 */
typedef void (*btn_callback_t)(board_btn_t btn, btn_event_t event);

/*============================================================================
 *                              接口函数
 *===========================================================================*/
/**
  * @brief  初始化按键驱动
  * @note   应在GPIO/EXTI初始化后调用
  */
void btn_driver_init(void);

/**
  * @brief  外部中断回调（在HAL_GPIO_EXTI_Callback中调用）
  * @param  gpio_pin: 触发中断的GPIO引脚
  * @note   此函数会启动消抖定时器
  */
void btn_exti_handler(uint16_t gpio_pin);

/**
  * @brief  定时器处理（在TIM17中断中调用）
  * @note   处理消抖计数和长按检测
  */
void btn_timer_process(void);

/**
  * @brief  注册事件回调函数
  * @param  callback: 回调函数指针
  */
void btn_register_callback(btn_callback_t callback);

/**
  * @brief  检查按键是否正在被按住
  * @param  btn: 按键ID
  * @retval true: 正在按住, false: 未按住
  */
bool btn_is_pressed(board_btn_t btn);

#endif /* DRIVER_BUTTON_H_ */
