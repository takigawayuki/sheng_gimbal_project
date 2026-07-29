#include "common.h"

menu_t menu;

extern volatile uint32_t target_lost_cnt;
// extern volatile uint8_t  target_valid;
extern gimbal_sm_t gimbal_sm_obj;

static gimbal_state menu_to_balance_state(menu_item_t item)
{
    switch (item)
    {
    case MENU_ITEM_TASK3_STATIC_PM5:
        return BALANCE_TASK3_STATIC_PLUS_TO_MINUS;
    case MENU_ITEM_TASK4_AB_CENTER:
        return BALANCE_TASK4_CAR_TO_B_CENTER;
    case MENU_ITEM_TASK5_LAP_CENTER:
        return BALANCE_TASK5_CAR_LAP_CENTER;
    case MENU_ITEM_TASK6_LAP_SETPOINT:
        return BALANCE_TASK6_CAR_LAP_SETPOINT;
    default:
        return GIMBAL_IDLE;
    }
}

void menu_init(void)
{
    menu.cur_item = MENU_ITEM_STANDBY;
    menu.in_running = 0;
}

void menu_update(key_event_t ev_menu, key_event_t ev_enter)
{
    if (menu.in_running == 0)
    {
        // ===== 菜单选择态 =====
        if (ev_menu == KEY_EVENT_SHORT)
        {
            menu.cur_item = (menu.cur_item + 1) % MENU_ITEM_COUNT;
        }
        if (ev_enter == KEY_EVENT_SHORT)
        {
            menu.in_running = 1;
            balance_task_start(menu_to_balance_state(menu.cur_item));
        }
    }
    else
    {
        // ===== 运行态 =====
        if (ev_enter == KEY_EVENT_SHORT)
        {
            menu.in_running = 0;
            balance_task_start(GIMBAL_IDLE);
        }
        // PA4 不响应
    }
}
