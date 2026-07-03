#include "Easy_Menu.h"

Easy_Menu easy_menu;

/* =============================================================== 系统辅助函数 =============================================================== */
/**
  * @brief  获取UTF-8字符的字节长度
  * @param  str: UTF-8字符字符串指针
  * @retval UTF-8字符长度（1-4字节），0表示无效编码
  */
static unsigned char GetUTF8Len(char *str)
{
    if (str == NULL || str[0] == '\0') return 0;
    
    if ((str[0] & 0x80) == 0x00) {
        return 1;  // ASCII字符（0xxxxxxx）
    }
    else if ((str[0] & 0xE0) == 0xC0) {
        return 2;  // 2字节UTF-8字符（110xxxxx）
    }
    else if ((str[0] & 0xF0) == 0xE0) {
        return 3;  // 3字节UTF-8字符（1110xxxx），常用中文
    }
    else if ((str[0] & 0xF8) == 0xF0) {
        return 4;  // 4字节UTF-8字符（11110xxxx）
    }
    return 0;  // 无效的UTF-8编码
}

unsigned char Compare_String(char *str1, char *str2, unsigned char len)
{
    unsigned char result = 1;
    
    for(unsigned char i = 0; i < len; i++)
    {
        if(str1[i] != str2[i])
        {
            result = 0;
            break;
        }
    }
    
    return result;
}

/* =============================================================== 系统绘制函数 =============================================================== */
void Easy_Menu_All_Update(void)
{
    if(easy_menu.Display_Char_Line != NULL)
    {
        for(unsigned char line = 0; line < EASY_MENU_LINE_MAX_NUM; line++)
        {
            if(Compare_String(easy_menu.buffer[line], easy_menu.compare_buffer[line], EASY_MENU_COL_MAX_NUM) == 0)
            {
                memcpy(easy_menu.buffer[line], easy_menu.compare_buffer[line], EASY_MENU_COL_MAX_NUM);
                
                for(unsigned char col = 0; col < EASY_MENU_COL_MAX_NUM; )
                {
                    unsigned char utf8_len = GetUTF8Len(&easy_menu.buffer[line][col]);
                    
                    if(utf8_len == 1)
                    {
                        // ASCII字符
                        if(easy_menu.current_page->type == ORDINARY_PAGE)
                        {
                            easy_menu.Display_Char_Line(col * CHAR_WIDTH, line, easy_menu.buffer[line][col], (easy_menu.init_flag == 1 && easy_menu.current_line == line));
                        }
                        else
                        {
                            easy_menu.Display_Char_Line(col * CHAR_WIDTH, line, easy_menu.buffer[line][col], 0);
                        }
                        col++;
                    }
                    else if(utf8_len == 0 || col + utf8_len > EASY_MENU_COL_MAX_NUM)
                    {
                        // 无效UTF-8或超出边界，替换为空格
                        easy_menu.buffer[line][col] = ' ';
                        easy_menu.Display_Char_Line(col * CHAR_WIDTH, line, ' ', (easy_menu.init_flag == 1 && easy_menu.current_line == line));
                        col++;
                    }
                    else if(easy_menu.Display_UTF8_Char_Line != NULL)
                    {
                        // UTF-8多字节字符
                        easy_menu.Display_UTF8_Char_Line(col * CHAR_WIDTH, line, &easy_menu.buffer[line][col], utf8_len, (easy_menu.init_flag == 1 && easy_menu.current_line == line));
                        col += utf8_len;
                    }
                    else
                    {
                        // 没有UTF-8显示函数，跳过
                        col += utf8_len;
                    }
                }
            }
        }
            
    }
    else if(easy_menu.Display_Char != NULL)
    {
        for(unsigned char line = 0; line < EASY_MENU_LINE_MAX_NUM; line++)
        {
            if(Compare_String(easy_menu.buffer[line], easy_menu.compare_buffer[line], EASY_MENU_COL_MAX_NUM) == 0)
            {
                memcpy(easy_menu.buffer[line], easy_menu.compare_buffer[line], EASY_MENU_COL_MAX_NUM);
                
                for(unsigned char col = 0; col < EASY_MENU_COL_MAX_NUM; )
                {
                    unsigned char utf8_len = GetUTF8Len(&easy_menu.buffer[line][col]);
                    
                    if(utf8_len == 1)
                    {
                        // ASCII字符
                        if(easy_menu.current_page->type == ORDINARY_PAGE)
                        {
                            easy_menu.Display_Char(col * CHAR_WIDTH, line * CHAR_HEIGHT, easy_menu.buffer[line][col], (easy_menu.init_flag == 1 && easy_menu.current_line == line));
                        }
                        else
                        {
                            easy_menu.Display_Char(col * CHAR_WIDTH, line * CHAR_HEIGHT, easy_menu.buffer[line][col], 0);
                        }
                        col++;
                    }
                    else if(utf8_len == 0 || col + utf8_len > EASY_MENU_COL_MAX_NUM)
                    {
                        // 无效UTF-8或超出边界，替换为空格
                        easy_menu.buffer[line][col] = ' ';
                        easy_menu.Display_Char(col * CHAR_WIDTH, line * CHAR_HEIGHT, ' ', (easy_menu.init_flag == 1 && easy_menu.current_line == line));
                        col++;
                    }
                    else if(easy_menu.Display_UTF8_Char != NULL)
                    {
                        // UTF-8多字节字符
                        easy_menu.Display_UTF8_Char(col * CHAR_WIDTH, line * CHAR_HEIGHT, &easy_menu.buffer[line][col], utf8_len, (easy_menu.init_flag == 1 && easy_menu.current_line == line));
                        col += utf8_len;
                    }
                    else
                    {
                        // 没有UTF-8显示函数，跳过
                        col += utf8_len;
                    }
                }
            }
        }
    }
}

void Easy_Menu_Area_Update(unsigned char col_start, unsigned char col_end, unsigned char line_start, unsigned char line_end)
{
    if(easy_menu.Display_Char_Line != NULL)
    {
        for(unsigned char line = line_start; line <= line_end; line++)
        {
            if(line >= EASY_MENU_LINE_MAX_NUM)
                break;
            
            for(unsigned char col = col_start; col <= col_end; )
            {
                if(col >= EASY_MENU_COL_MAX_NUM) 
                    break;
                
                unsigned char utf8_len = GetUTF8Len(&easy_menu.buffer[line][col]);
                
                if(utf8_len == 1)
                {
                    easy_menu.Display_Char_Line(col * CHAR_WIDTH, line, easy_menu.buffer[line][col], (easy_menu.init_flag == 1 && easy_menu.current_line == line));
                    col++;
                }
                else if(utf8_len == 0 || col + utf8_len > EASY_MENU_COL_MAX_NUM)
                {
                    easy_menu.buffer[line][col] = ' ';
                    easy_menu.Display_Char_Line(col * CHAR_WIDTH, line, ' ', (easy_menu.init_flag == 1 && easy_menu.current_line == line));
                    col++;
                }
                else if(easy_menu.Display_UTF8_Char_Line != NULL)
                {
                    easy_menu.Display_UTF8_Char_Line(col * CHAR_WIDTH, line, &easy_menu.buffer[line][col], utf8_len, (easy_menu.init_flag == 1 && easy_menu.current_line == line));
                    col += utf8_len;
                }
                else
                {
                    col += utf8_len;
                }
            }
        }
    }
    else if(easy_menu.Display_Char != NULL)
    {
        for(unsigned char line = line_start; line <= line_end; line++)
        {
            if(line >= EASY_MENU_LINE_MAX_NUM)
                break;
            
            for(unsigned char col = col_start; col <= col_end; )
            {
                if(col >= EASY_MENU_COL_MAX_NUM) 
                    break;

                unsigned char utf8_len = GetUTF8Len(&easy_menu.buffer[line][col]);
                
                if(utf8_len == 1)
                {
                    easy_menu.Display_Char(col * CHAR_WIDTH, line * CHAR_HEIGHT, easy_menu.buffer[line][col], (easy_menu.init_flag == 1 && easy_menu.current_line == line));
                    col++;
                }
                else if(utf8_len == 0 || col + utf8_len > EASY_MENU_COL_MAX_NUM)
                {
                    easy_menu.buffer[line][col] = ' ';
                    easy_menu.Display_Char(col * CHAR_WIDTH, line * CHAR_HEIGHT, ' ', (easy_menu.init_flag == 1 && easy_menu.current_line == line));
                    col++;
                }
                else if(easy_menu.Display_UTF8_Char != NULL)
                {
                    easy_menu.Display_UTF8_Char(col * CHAR_WIDTH, line * CHAR_HEIGHT, &easy_menu.buffer[line][col], utf8_len, (easy_menu.init_flag == 1 && easy_menu.current_line == line));
                    col += utf8_len;
                }
                else
                {
                    col += utf8_len;
                }
            }
        }
    }
}

void Easy_Menu_All_Clear(void)
{
    for(unsigned char line = 0; line < EASY_MENU_LINE_MAX_NUM; line++)
    {
        for(unsigned char col = 0; col < EASY_MENU_COL_MAX_NUM; col++)
        {
            easy_menu.compare_buffer[line][col] = ' ';
        }
    }
}

void Easy_Menu_Area_Clear(unsigned char col_start, unsigned char col_end, unsigned char line_start, unsigned char line_end)
{
    for(unsigned char line = line_start; line <= line_end; line++)
    {
        if(line >= EASY_MENU_LINE_MAX_NUM)
            break;
        
        for(unsigned char col = col_start; col <= col_end; col++)
        {
            if(col >= EASY_MENU_COL_MAX_NUM) 
            break;
        
            easy_menu.compare_buffer[line][col] = ' ';
        }
    }
}

void Easy_Menu_Display_Char(unsigned char col, unsigned char line, char ch)
{
    if(col >= EASY_MENU_COL_MAX_NUM || line >= EASY_MENU_LINE_MAX_NUM) return;
    
    easy_menu.compare_buffer[line][col] = ch;
}

void Easy_Menu_Display_String(int col, unsigned char line, char* str)
{
    unsigned char len = strlen(str);
    for(unsigned char i = 0; str[i] != '\0'; i++)
    {
        if(col > 0)
        {
            Easy_Menu_Display_Char(col++, line, str[i]);
        }
        else
        {
            Easy_Menu_Display_Char(EASY_MENU_COL_MAX_NUM + col, line, str[len - i - 1]);
            col--;
        }
    }
}

void Easy_Menu_Printf(int col, unsigned char line, const char *format, ...)
{
	char temp_buffer[EASY_MENU_COL_MAX_NUM]; // 临时存储格式化后的字符串
	va_list arg;      // 处理可变参数

	va_start(arg, format);
	// 安全地格式化字符串到 buffer
	vsnprintf(temp_buffer, sizeof(temp_buffer), format, arg);
	va_end(arg);

    Easy_Menu_Display_String(col, line, temp_buffer);
}

