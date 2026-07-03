/**
  ******************************************************************************
  * @file    menu_adapter.h
  * @brief   菜单适配层 - 连接Easy_Menu和OLED驱动
  * @note    将Easy_Menu的UTF-8接口适配到OLED驱动库
  ******************************************************************************
  */

#ifndef __MENU_ADAPTER_H__
#define __MENU_ADAPTER_H__

#include <stdint.h>

/**
  * @brief  初始化菜单适配层
  * @note   调用此函数前，确保I2C和OLED已初始化
  */
void Menu_Adapter_Init(void);

/**
  * @brief  菜单显示刷新（供主循环调用）
  * @param  tick: 系统Tick（建议1ms自增）
  */
void Menu_Adapter_Display(uint32_t tick);

/**
  * @brief  菜单输入处理
  * @param  input: 输入类型（上/下/左/右）
  */
void Menu_Adapter_Input(uint8_t input);

#endif /* __MENU_ADAPTER_H__ */
