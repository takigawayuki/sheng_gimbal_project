#include "common.h"
#include "ZhangDaTou.h"
#include "math.h"

// #define SEARCH_TIMEOUT 5000 // 假设找靶超时时间为5000ms

#define AIM_STABLE_CNT 300  // 1ms tick 计数，连续 300 次稳 → 开
#define AIM_STABLE_FRAMES 5 // 连续 5 帧稳定才开激光

#define AIM_UNSTABLE_CNT 20 // 连续 20 次不稳才关
#define AIM_THRESHOLD 3.0f  // 允许误差（像素）

// #define LASER_K 0.0f
// #define LASER_B 0.0f

#define LOST_BACK_TO_SEARCH_CNT 300 // 连续 300ms 丢靶 → 回搜索

sys_t sys;
// gimbal_sm_t gimbal_sm_obj = {GIMBAL_IDLE, 0, 0, 1};
gimbal_sm_t gimbal_sm_obj = {GIMBAL_IDLE, 0, 0};

volatile uint32_t target_lost_cnt = 0;
// volatile uint8_t target_valid = 0;
volatile uint8_t aim_stable_frames = 0;

uint16_t stable_cnt = 0;
uint16_t unstable_cnt = 0;
uint8_t laser_on = 0;

extern float yaw_pos;

// 给发挥题判断用的
#define AIM_CENTER_X_ERR 3.0f
#define AIM_CENTER_Y_ERR 3.0f

/*** target_stable ***/
uint8_t target_stable(void)
{
    // if (!target_found())
    //     return 0; // 没数据 → 不稳

    if (fabs(sys.value.camera_x) < AIM_THRESHOLD &&
        fabs(sys.value.camera_y) < AIM_THRESHOLD)
        return 1;
    return 0;
}

static void gimbal_pid_ctrl_10ms(void)
{
    static uint16_t pid_test_cnt = 0;

    if (++pid_test_cnt >= 10)
    {
        pid_test_cnt = 0;
        camera_x_pid_ctrl(&sys, 0.0f);
        camera_y_pid_ctrl(&sys, 0.0f);
        // camera_x_pid_run_ctrl(&sys, 0.0f);
        // camera_y_pid_run_ctrl(&sys, 0.0f);
    }
}

static void gimbal_pid_ctrl_run_10ms(void)
{

    static uint16_t pid_test_cnt = 0;

    if (++pid_test_cnt >= 10)
    {
        pid_test_cnt = 0;
        // camera_x_pid_ctrl(&sys, 0.0f);
        // camera_y_pid_ctrl(&sys, 0.0f);
        camera_x_pid_run_ctrl(&sys, 0.0f);
        camera_y_pid_run_ctrl(&sys, 0.0f);
    }
}

static void gimbal_pid_ctrl_run_1ms(void)
{
    camera_x_pid_run_ctrl(&sys, 0.0f);
    camera_y_pid_run_ctrl(&sys, 0.0f);
}

void gimbal_task_state(void)
{
    switch (gimbal_sm_obj.state)
    {
    case GIMBAL_IDLE:
        // 速度位置模式
        ZhangDaTou_PositionSpeedctr(&yawmotor, yawmotor.setSpeed, 0.0f, yawmotor.setAcc);
        ZhangDaTou_Control(&yawmotor);
        ZhangDaTou_PositionSpeedctr(&pitchmotor, pitchmotor.setSpeed, 0.0f, pitchmotor.setAcc);
        ZhangDaTou_Control(&pitchmotor);

        HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_RESET);
        stable_cnt = 0;
        unstable_cnt = 0;
        laser_on = 0;
        break;

    case GIMBAL_SEARCH_LEFT:
        // 转移：看到目标 → TRACK
        if (sys.value.camera_x != 0)
        {
            gimbal_sm_obj.state = GIMBAL_STATIC_TRACK;
            break; // 这一 tick 不执行 SEARCH 动作，下 tick 走 TRACK
        }
        // 速度模式
        ZhangDaTou_Speedctr(&yawmotor, -100.0f, 2000);
        ZhangDaTou_Control(&yawmotor);

        HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_RESET);
        break;

    case GIMBAL_SEARCH_RIGHT:
        // 转移：看到目标 → TRACK
        if (sys.value.camera_x != 0)
        {
            gimbal_sm_obj.state = GIMBAL_STATIC_TRACK;
            break; // 这一 tick 不执行 SEARCH 动作，下 tick 走 TRACK
        }
        // 速度模式
        ZhangDaTou_Speedctr(&yawmotor, +100.0f, 2000);
        ZhangDaTou_Control(&yawmotor);

        HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_RESET);
        break;

    case GIMBAL_STATIC_TRACK:
        // 激光控制：稳定够久才开，抖动够久才关
        if (target_stable())
        {
            // unstable_cnt = 0;
            if (stable_cnt < AIM_STABLE_CNT)
                stable_cnt++;
            if (stable_cnt >= AIM_STABLE_CNT && !laser_on)
            {
                HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_SET);
                laser_on = 1;
            }
        }
        else
        {
            stable_cnt = 0;
        }

        if (laser_on)
        {
            // 已锁定：保持锁定瞬间记录的位置，速度沿用配置的 setSpeed，PID 不跑
            ZhangDaTou_PositionSpeedctr(&yawmotor, yawmotor.setSpeed, yawmotor.setPosition, yawmotor.setAcc);
            ZhangDaTou_Control(&yawmotor);
            ZhangDaTou_PositionSpeedctr(&pitchmotor, pitchmotor.setSpeed, pitchmotor.setPosition, pitchmotor.setAcc);
            ZhangDaTou_Control(&pitchmotor);
        }
        else
        {
            gimbal_pid_ctrl_10ms();
        }
        break;

    case GIMBAL_DYNAMIC_TRACK:

        gimbal_pid_ctrl_10ms();

        // 激光控制：稳定够久才开，抖动够久才关
        if (target_stable())
        {
            // unstable_cnt = 0;
            if (stable_cnt < AIM_STABLE_CNT)
                stable_cnt++;
            if (stable_cnt >= AIM_STABLE_CNT && !laser_on)
            {
                HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_SET);
                laser_on = 1;
            }
        }
        else
        {
            stable_cnt = 0;
        }
        break;

    case GIMBAL_DYNAMIC_RUNNING:
        gimbal_pid_ctrl_run_10ms();

        if (target_stable())
        {
            if (stable_cnt < AIM_STABLE_CNT)
                stable_cnt++;
            if (stable_cnt >= AIM_STABLE_CNT && !laser_on)
            {
                HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_SET);
                laser_on = 1;
            }
        }
        else
        {
            stable_cnt = 0;
        }
        break;

    default:
        break;
    }
}
