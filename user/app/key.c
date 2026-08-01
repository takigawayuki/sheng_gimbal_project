#include "common.h"

#define KEY_SCAN_FILTER_CNT  20   // 如果 key_update 每 1ms 调一次，就是约 20ms 消抖

key_t key_menu;
key_t key_enter;

void key_init(void)
{
    key_menu.port  = GPIOA;
    key_menu.pin   = GPIO_PIN_4;    // 菜单按键
    // key_menu.last_level = 1;        // 初始认为是松开（高电平）

    key_enter.port = GPIOC;
    key_enter.pin  = GPIO_PIN_3;    // 功能运行退出按键
    // key_enter.last_level = 1;

    key_menu.filter_cnt = 0;
    key_menu.press_flag = 0;

    key_enter.filter_cnt = 0;
    key_enter.press_flag = 0;
}


key_event_t key_update(key_t *k)
{
    // uint8_t cur = HAL_GPIO_ReadPin(k->port, k->pin);

    // key_event_t ev = KEY_EVENT_NONE;

    // if (k->last_level == 1 && cur == 0)   // 下降沿 = 刚按下
    // {
    //     ev = KEY_EVENT_SHORT;
    // }

    // k->last_level = cur;
    // return ev;

    key_event_t ev = KEY_EVENT_NONE;
    uint8_t cur = HAL_GPIO_ReadPin(k->port, k->pin);

    if (cur == GPIO_PIN_RESET)   // 上拉输入，低电平表示按下
    {
        if (k->filter_cnt < KEY_SCAN_FILTER_CNT)
        {
            k->filter_cnt++;
        }
        else if (k->press_flag == 0)
        {
            k->press_flag = 1;
            ev = KEY_EVENT_SHORT;
        }
    }
    else
    {
        if (k->filter_cnt > 0)
        {
            k->filter_cnt--;
        }
        else
        {
            k->press_flag = 0;
        }
    }

    return ev;
}
