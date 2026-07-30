#include "user_tim.h"
#include "common.h"
// #include "mpu9250_app.h"
#include "ZhangDaTou.h"

float yaw_pos = 0.0f;
float pitch_pos = 0.0f;

// int key_1 = 0;

extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;

// extern volatile uint8_t target_valid;
extern volatile uint32_t target_lost_cnt;

// #define KEY_ON 1
// #define KEY_OFF 0
// uint8_t key_scan(void);

// test
volatile uint32_t key_menu_cnt = 0;
volatile uint32_t key_enter_cnt = 0;

void User_TIM_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim7); // TIM7 1ms 中断 1kHz的控制频率
    HAL_TIM_Base_Start_IT(&htim8); // TIM8 10ms 中断，给 MPU9250 预留的
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        if (target_lost_cnt < 500)
            target_lost_cnt++;
        // if (target_lost_cnt >= 100)
        //     target_valid = 0; // 100ms 没新数据 → 判丢失

        // gimbal_sm();

        // gimbal_task_state(); // 每 1ms 执行一次

        // if (key_scan() == KEY_ON)
        // {
        //     key_1 = 1;
        // }
        // if (key_1 == 1)
        // {
        //     static int status;
        //     if (status == 0)
        //     {
        //         ZhangDaTou_Speedctr(&yawmotor, 15.0f, yawmotor.setAcc);
        //         ZhangDaTou_Control(&yawmotor);

        //         if (sys.value.camera_x != 0)
        //         {
        //             status = 1;
        //             // ZhangDaTou_Speedctr(&yawmotor,0.0f, yawmotor.setAcc);
        //             // ZhangDaTou_Control(&yawmotor);
        //         }
        //     }
        //     if (status == 1)
        //     {
        //         camera_x_pid_ctrl(&sys, 0.0f);
        //         camera_y_pid_ctrl(&sys, 0.0f);
        //         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
        //     }
        // }

        // 接收两个轴电机的角度位置
        yaw_pos   = ZhangDaTou_getPositionDate(&yawmotor);
        pitch_pos = ZhangDaTou_getPositionDate(&pitchmotor);

        // test
        // camera_x_pid_ctrl(&sys, 0.0f);
        // camera_y_pid_ctrl(&sys, 0.0f);

        // static uint16_t pid_test_cnt = 0;
        // if (++pid_test_cnt >= 20)
        // {
        //     pid_test_cnt = 0;
        //     camera_x_pid_ctrl(&sys, 0.0f);
        //     camera_y_pid_ctrl(&sys, 0.0f); 
        // }   

        // camera_x_pid_run_ctrl(&sys, 0.0f);
        // camera_y_pid_run_ctrl(&sys, 0.0f);

        key_event_t ev_menu = key_update(&key_menu);
        key_event_t ev_enter = key_update(&key_enter);

        // if (ev_menu == KEY_EVENT_SHORT)
        //     HAL_GPIO_TogglePin(text_io_GPIO_Port, text_io_Pin);

        // if (ev_enter == KEY_EVENT_SHORT)
        //     HAL_GPIO_TogglePin(laser_GPIO_Port, laser_Pin);

        // test
        // if (ev_menu == KEY_EVENT_SHORT)
        //     key_menu_cnt++;
        // if (ev_enter == KEY_EVENT_SHORT)
        //     key_enter_cnt++;

        // menu_update(ev_menu, ev_enter);

        gimbal_task_state(); // 每 1ms 执行一次


    }
    // else if (htim->Instance == TIM8)
    // {
    //     MPU9250_Update10ms();
    // }





}
