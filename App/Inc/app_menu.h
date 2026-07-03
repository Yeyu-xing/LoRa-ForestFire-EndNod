/**
  ******************************************************************************
  * @file    app_menu.h
  * @brief   应用菜单模块 - 菜单定义与按键映射
  * @note    基于Easy_Menu库实现，提供主页实时数据展示和设置菜单
  ******************************************************************************
  */

#ifndef APP_MENU_H_
#define APP_MENU_H_

#include <stdint.h>
#include <stdbool.h>
#include "app_main.h"
#include "board_io.h"
#include "driver_button.h"

/*============================================================================
 *                              接口函数
 *===========================================================================*/
/**
  * @brief  初始化菜单模块
  * @note   初始化OLED、Easy_Menu适配层和菜单结构
  */
void app_menu_init(void);

/**
  * @brief  菜单显示刷新
  * @note   在主循环中调用，刷新OLED显示
  */
void app_menu_display(void);

/**
  * @brief  按键输入处理
  * @param  btn: 按键ID
  * @param  event: 按键事件
  * @retval true: 菜单已处理该按键, false: 菜单未处理
  */
bool app_menu_handle_button(board_btn_t btn, btn_event_t event);

/**
  * @brief  进入设置菜单
  * @note   从主页跳转到设置菜单页
  */
void app_menu_enter_settings(void);

/**
  * @brief  退出设置菜单
  * @note   返回主页
  */
void app_menu_exit_settings(void);

/**
  * @brief  检查是否在主页
  * @retval true: 当前在主页
  */
bool app_menu_is_main_page(void);

/**
  * @brief  检查是否在设置菜单（包含子页面）
  * @retval true: 当前在设置相关页面
  */
bool app_menu_is_settings_page(void);

/**
 * @brief  检查屏幕是否亮起
 * @note   屏幕亮起时应阻止低功耗休眠
 * @retval true: 屏幕亮起
 */
bool app_menu_is_screen_on(void);

/**
 * @brief  请求重新部署 (清除位置数据，触发GNSS定位)
 * @note   在菜单中选择"重新部署"时调用
 */
void app_menu_request_redeploy(void);

/**
 * @brief  检查是否有重新部署请求
 * @retval true: 有待处理的重新部署请求
 */
bool app_menu_has_redeploy_request(void);

/**
 * @brief  程序化亮屏 (按键唤醒STOP2时使用)
 */
void app_menu_wake_screen(void);

#endif /* APP_MENU_H_ */
