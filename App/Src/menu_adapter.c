/**
  ******************************************************************************
  * @file    menu_adapter.c
  * @brief   菜单适配层 - 连接Easy_Menu和OLED驱动
  * @note    实现UTF-8字符到OLED显示的适配
  ******************************************************************************
  */

#include "menu_adapter.h"
#include "Easy_Menu.h"
#include "oled.h"
#include "font.h"

/* =============================================================== 私有函数 =============================================================== */

/**
  * @brief  ASCII字符显示函数（像素坐标）
  * @param  x: X轴坐标（像素）
  * @param  y: Y轴坐标（像素）
  * @param  ch: ASCII字符
  * @param  reverse_flag: 反色标志，0-正常，1-反色
  */
static void Display_Char_Callback(uint16_t x, uint16_t y, char ch, uint8_t reverse_flag)
{
    OLED_ColorMode color = reverse_flag ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL;
    OLED_PrintASCIIChar(x, y, ch, &afont8x6, color);
}

/**
  * @brief  ASCII字符显示函数（行坐标）
  * @param  x: X轴坐标（像素）
  * @param  line: 行号
  * @param  ch: ASCII字符
  * @param  reverse_flag: 反色标志，0-正常，1-反色
  */
static void Display_Char_Line_Callback(uint16_t x, uint8_t line, char ch, uint8_t reverse_flag)
{
    Display_Char_Callback(x, line * 16, ch, reverse_flag);
}

/**
  * @brief  UTF-8字符显示函数（像素坐标）
  * @param  x: X轴坐标（像素）
  * @param  y: Y轴坐标（像素）
  * @param  str: UTF-8字符串指针
  * @param  len: UTF-8字符长度（字节数）
  * @param  reverse_flag: 反色标志，0-正常，1-反色
  */
static void Display_UTF8_Char_Callback(uint16_t x, uint16_t y, char *str, uint8_t len, uint8_t reverse_flag)
{
    OLED_ColorMode color = reverse_flag ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL;
    
    // 使用font16x16显示UTF-8字符（包含中文字库）
    // 注意：font16x16需要在font.c中添加所需的汉字字模
    OLED_PrintString(x, y, str, &font16x16, color);
}

/**
  * @brief  UTF-8字符显示函数（行坐标）
  * @param  x: X轴坐标（像素）
  * @param  line: 行号
  * @param  str: UTF-8字符串指针
  * @param  len: UTF-8字符长度（字节数）
  * @param  reverse_flag: 反色标志，0-正常，1-反色
  */
static void Display_UTF8_Char_Line_Callback(uint16_t x, uint8_t line, char *str, uint8_t len, uint8_t reverse_flag)
{
    Display_UTF8_Char_Callback(x, line * 16, str, len, reverse_flag);
}

/* =============================================================== 公共函数 =============================================================== */

/**
  * @brief  初始化菜单适配层
  */
void Menu_Adapter_Init(void)
{
    // 初始化OLED
    OLED_Init();
    OLED_DisPlay_On();
    
    // 初始化菜单系统，绑定显示回调函数
    Easy_Menu_Init(
        Display_Char_Callback,           // ASCII字符显示（像素坐标）
        Display_Char_Line_Callback,      // ASCII字符显示（行坐标）
        Display_UTF8_Char_Callback,      // UTF-8字符显示（像素坐标）
        Display_UTF8_Char_Line_Callback  // UTF-8字符显示（行坐标）
    );
}

/**
  * @brief  菜单显示刷新
  * @param  tick: 系统Tick
  * @note   每次刷新都清空OLED显存，因此需要强制Easy_Menu全量重绘。
  *         Easy_Menu使用增量更新（脏标记），Ordinary_Page_Display()
  *         在无Show_Item刷新时不调用Easy_Menu_All_Update()，导致
  *         清空显存后内容丢失（屏幕熄灭）。解决方案：在Display前
  *         清除compare_buffer使脏标记失效，Display后强制全量更新。
  */
void Menu_Adapter_Display(uint32_t tick)
{
    // 开始新帧（清空OLED显存）
    OLED_NewFrame();
    
    // 强制Easy_Menu全量重绘：清除compare_buffer使所有行标记为"脏"
    Easy_Menu_All_Clear();
    
    // 调用菜单显示函数（处理输入+页面更新）
    Easy_Menu_Display(tick);
    
    // 强制全量更新：将buffer内容绘制到OLED显存
    // （Ordinary_Page_Display不会自动调用，Show_Page也不会）
    Easy_Menu_All_Update();
    
    // 显示帧内容
    OLED_ShowFrame();
}

/**
  * @brief  菜单输入处理
  * @param  input: 输入类型
  *         - 0: EASY_MENU_NONE
  *         - 1: EASY_MENU_UP
  *         - 2: EASY_MENU_DOWN
  *         - 3: EASY_MENU_LEFT
  *         - 4: EASY_MENU_RIGHT
  */
void Menu_Adapter_Input(uint8_t input)
{
    Easy_Menu_Input((Easy_Menu_Input_TYPE)input);
}
