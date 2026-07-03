/**
  ******************************************************************************
  * @file    driver_button.c
  * @brief   按键驱动 - 外部中断触发 + 定时消抖实现
  ******************************************************************************
  */

#include "driver_button.h"
#include "main.h"

/*============================================================================
 *                              私有常量
 *===========================================================================*/
/* 消抖计数阈值 */
#define DEBOUNCE_THRESHOLD      (BTN_DEBOUNCE_TIME_MS / BTN_TIMER_TICK_MS)

/* 长按计数阈值 */
#define LONG_PRESS_THRESHOLD    (BTN_LONG_PRESS_TIME_MS / BTN_TIMER_TICK_MS)

/* GPIO引脚到按键ID的映射 */
static const struct {
    uint16_t pin;
    board_btn_t btn;
} pin_to_btn[] = {
    {BTN_UP_Pin,   BTN_UP},
    {BTN_OK_Pin,   BTN_OK},
    {BTN_DOWN_Pin, BTN_DOWN},
};
#define PIN_MAP_SIZE  (sizeof(pin_to_btn) / sizeof(pin_to_btn[0]))

/*============================================================================
 *                              私有变量
 *===========================================================================*/
/* 按键状态结构 */
typedef struct {
    uint8_t  debounce_cnt;      // 消抖计数器
    uint16_t hold_cnt;          // 按住计数器（用于长按检测）
    bool     debounce_active;   // 消抖进行中
    bool     stable_state;      // 稳定状态（消抖后确认）
    bool     long_press_fired;  // 长按事件已触发标志
} btn_state_t;

static btn_state_t btn_states[BTN_COUNT];

/* 回调函数指针 */
static btn_callback_t event_callback = NULL;

/* 定时器运行标志（有按键需要处理时为true） */
static bool timer_running = false;

/* 调试计数器 */
volatile uint32_t dbg_exti_matched = 0;
volatile uint32_t dbg_debounce_done = 0;
volatile uint32_t dbg_state_changed = 0;
volatile uint32_t dbg_callback_called = 0;
volatile uint32_t dbg_timer_active = 0;

/*============================================================================
 *                              初始化
 *===========================================================================*/
void btn_driver_init(void)
{
    // 清零所有按键状态
    for (int i = 0; i < BTN_COUNT; i++) {
        btn_states[i].debounce_cnt = 0;
        btn_states[i].hold_cnt = 0;
        btn_states[i].debounce_active = false;
        btn_states[i].stable_state = false;
        btn_states[i].long_press_fired = false;
    }
    
    timer_running = false;
    event_callback = NULL;
}

/*============================================================================
 *                              外部中断处理
 *===========================================================================*/
void btn_exti_handler(uint16_t gpio_pin)
{
    // 查找对应的按键
    for (int i = 0; i < (int)PIN_MAP_SIZE; i++) {
        if (pin_to_btn[i].pin == gpio_pin) {
            btn_state_t *state = &btn_states[pin_to_btn[i].btn];
            
            dbg_exti_matched++;
            
            // 启动/重启消抖
            state->debounce_cnt = 0;
            state->debounce_active = true;
            
            // 标记定时器需要运行
            timer_running = true;
            
            break;
        }
    }
}

/*============================================================================
 *                              定时器处理
 *===========================================================================*/
void btn_timer_process(void)
{
    // 如果没有按键活动，直接返回
    if (!timer_running) {
        return;
    }
    
    dbg_timer_active++;
    
    bool any_active = false;
    
    for (int i = 0; i < BTN_COUNT; i++) {
        btn_state_t *state = &btn_states[i];
        
        // 消抖处理
        if (state->debounce_active) {
            state->debounce_cnt++;
            
            if (state->debounce_cnt >= DEBOUNCE_THRESHOLD) {
                dbg_debounce_done++;
                
                // 消抖完成，读取当前GPIO状态
                bool current_state = board_btn_read((board_btn_t)i);
                
                // 状态变化确认
                if (current_state != state->stable_state) {
                    bool prev_state = state->stable_state;
                    state->stable_state = current_state;
                    
                    dbg_state_changed++;
                    
                    if (current_state && !prev_state) {
                        // 按下事件
                        if (event_callback) {
                            dbg_callback_called++;
                            event_callback((board_btn_t)i, BTN_EVENT_PRESS);
                        }
                        // 启动长按计数
                        state->hold_cnt = 0;
                    } else if (!current_state && prev_state) {
                        // 释放事件
                        if (event_callback) {
                            dbg_callback_called++;
                            event_callback((board_btn_t)i, BTN_EVENT_RELEASE);
                        }
                        
                        // 短按判定：释放时未触发长按
                        if (!state->long_press_fired) {
                            if (event_callback) {
                                dbg_callback_called++;
                                event_callback((board_btn_t)i, BTN_EVENT_SHORT_CLICK);
                            }
                        }
                        
                        // 重置长按标志
                        state->long_press_fired = false;
                    }
                }
                
                // 消抖结束
                state->debounce_active = false;
                state->debounce_cnt = 0;
            } else {
                any_active = true;
            }
        }
        
        // 长按检测（仅在稳定按下时计数）
        if (state->stable_state && !state->long_press_fired) {
            state->hold_cnt++;
            
            if (state->hold_cnt >= LONG_PRESS_THRESHOLD) {
                state->long_press_fired = true;
                if (event_callback) {
                    dbg_callback_called++;
                    event_callback((board_btn_t)i, BTN_EVENT_LONG_PRESS);
                }
            }
            any_active = true;
        }
    }
    
    // 更新定时器运行标志
    timer_running = any_active;
}

/*============================================================================
 *                              回调注册
 *===========================================================================*/
void btn_register_callback(btn_callback_t callback)
{
    event_callback = callback;
}

/*============================================================================
 *                              状态查询
 *===========================================================================*/
bool btn_is_pressed(board_btn_t btn)
{
    if (btn >= BTN_COUNT) return false;
    return btn_states[btn].stable_state;
}
