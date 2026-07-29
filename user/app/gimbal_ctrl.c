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

void gimbal_task_state_legacy(void)
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

/* ================= H题：按题号划分的新状态机 =================
 * 说明：
 * 1. 本工程只负责摆杆和钢球控制，不负责小车循迹。
 * 2. 题1图传、题2小车一圈计时都不启动摆杆，本状态机只处理题3到题6。
 * 3. 视觉入口在 camera_data_update()，当前约定 dx 表示钢球相对 O 点的位置偏差，单位 cm。
 * 4. pitchmotor 暂时作为摆杆电机使用，控制方式是张大头步进电机的位置速度模式。
 */

/* 钢球认为稳定的误差范围。题目要求误差绝对值不大于 1cm，所以这里先用 1cm。 */
#define BALL_STABLE_ERR_CM       1.0f

/* 题3的两个目标点：先到 +5cm，再折返回 -5cm。 */
#define BALL_TASK3_PLUS_CM       5.0f
#define BALL_TASK3_MINUS_CM     -5.0f

/* 题3到达 +5cm 后至少稳定一小段时间，再切换到 -5cm，避免刚碰到目标就立刻折返。 */
#define BALL_TASK3_PLUS_HOLD_MS  300U

/* 题3总时间要求不超过 5s，这里用 5000ms 做结束判定参考。 */
#define BALL_TASK3_TOTAL_MS     5000U

/* 题4要求 A 到 B 不超过 8s，同时钢球保持在 O 点附近。 */
#define TASK4_AB_LIMIT_MS       8000U

/* 题5要求一圈不超过 30s，同时钢球保持在 O 点附近。 */
#define TASK5_LAP_LIMIT_MS     30000U

/* 题6要求一圈不超过 30s，同时钢球保持在指定位置附近。 */
#define TASK6_LAP_LIMIT_MS     30000U

/* 清空 H 题状态机的运行变量。
 * 每次从菜单启动一个题目时都会调用，防止上一次任务的计时、稳定计数、阶段标志残留。
 */
static void balance_reset_runtime(void)
{
    gimbal_sm_obj.elapsed_ms = 0;        // 当前状态运行时间清零
    gimbal_sm_obj.stable_ms = 0;         // 稳定计时清零
    gimbal_sm_obj.task3_phase = 0;       // 题3从第一阶段开始：先去 +5cm
    gimbal_sm_obj.finished = 0;          // 任务完成标志清零
    sys.value.task_run_time_ms = 0;      // 对外显示的任务计时清零
    sys.value.car_run_time_ms = 0;       // 对外显示的小车计时清零
}

/* 从菜单启动某一道 H 题。
 * state 传入 BALANCE_TASK3_STATIC_PLUS_TO_MINUS 到 BALANCE_TASK6_CAR_LAP_SETPOINT 中的一个。
 * 题6比较特殊，它的目标点来自 sys.ctrl.ball_task6_target_cm；其他题默认目标点先设为 O 点 0cm。
 */
void balance_task_start(gimbal_state state)
{
    balance_reset_runtime();
    gimbal_sm_obj.state = state;
    sys.ctrl.ball_target_cm = (state == BALANCE_TASK6_CAR_LAP_SETPOINT) ? sys.ctrl.ball_task6_target_cm : 0.0f;

    /* 记录当前电机位置作为 PID 位置控制基准，避免启动瞬间因为旧基准导致摆杆跳变。 */
    gimbal_pid_base_update_now();
}

/* 判断钢球是否已经稳定在当前目标点附近。
 * sys.value.ball_pos_cm 来自视觉滤波后的位置。
 * sys.ctrl.ball_target_cm 是当前题目希望钢球保持的位置。
 */
static uint8_t balance_ball_stable(void)
{
    return (fabsf(sys.value.ball_pos_cm - sys.ctrl.ball_target_cm) <= BALL_STABLE_ERR_CM) ? 1U : 0U;
}

/* 更新稳定计时。
 * 只要钢球还在允许误差范围内，stable_ms 就持续累加；一旦跑出误差范围就清零。
 */
static void balance_update_stable_counter(void)
{
    if (balance_ball_stable())
    {
        if (gimbal_sm_obj.stable_ms < 60000U)
            gimbal_sm_obj.stable_ms++;
    }
    else
    {
        gimbal_sm_obj.stable_ms = 0;
    }
}

/* 每 1ms 调用一次的任务计时。
 * gimbal_task_state() 放在主循环或定时任务里运行时，这里默认每进来一次就是 1ms。
 */
static void balance_tick_time(void)
{
    if (gimbal_sm_obj.elapsed_ms < 600000U)
        gimbal_sm_obj.elapsed_ms++;

    /* 这两个值用于 OLED/串口等地方显示运行时间。 */
    sys.value.task_run_time_ms = gimbal_sm_obj.elapsed_ms;
    sys.value.car_run_time_ms = gimbal_sm_obj.elapsed_ms;
}

/* 题3：小车静止，控制钢球从 O 点到 +5cm，再折返回 -5cm。
 * task3_phase = 0：控制目标为 +5cm。
 * task3_phase = 1：控制目标为 -5cm。
 */
static void balance_task3_static_pm5(void)
{
    if (gimbal_sm_obj.task3_phase == 0U)
    {
        /* 第一阶段：让钢球向 +5cm 运动。 */
        ball_balance_static_ctrl(&sys, BALL_TASK3_PLUS_CM);
        balance_update_stable_counter();

        /* 到 +5cm 附近并稳定一小段时间后，切到第二阶段。 */
        if (gimbal_sm_obj.stable_ms >= BALL_TASK3_PLUS_HOLD_MS)
        {
            gimbal_sm_obj.task3_phase = 1U;
            gimbal_sm_obj.stable_ms = 0;

            /* 切换目标点时清掉 PID 的积分和上次误差，减少折返瞬间拖尾。 */
            sys.camera_x_pid.i_term = 0.0f;
            sys.camera_x_pid.pre_err = 0.0f;
        }
    }
    else
    {
        /* 第二阶段：让钢球折返回 -5cm，并持续稳定在 -5cm 附近。 */
        ball_balance_static_ctrl(&sys, BALL_TASK3_MINUS_CM);
        balance_update_stable_counter();
    }

    /* 题3要求总运行时间不超过 5s。这里到 5s 后，如果钢球已经在最终目标附近，就置 finished。 */
    if (gimbal_sm_obj.elapsed_ms >= BALL_TASK3_TOTAL_MS && balance_ball_stable())
        gimbal_sm_obj.finished = 1U;
}

/* 题4/题5/题6共用的运动保持控制。
 * target_cm 是本题钢球目标位置。
 * limit_ms 是本题时间上限。到达时间上限后置 finished，实际停车/循迹由小车部分处理。
 * 这里调用运动 PID，并叠加 sys.ctrl.rod_chassis_ff_deg 这个底盘前馈占位量。
 */
static void balance_task_hold(float target_cm, uint32_t limit_ms)
{
    ball_balance_running_ctrl(&sys, target_cm);
    balance_update_stable_counter();

    if (gimbal_sm_obj.elapsed_ms >= limit_ms)
        gimbal_sm_obj.finished = 1U;
}

/* H 题总状态机。
 * 菜单确认后会调用 balance_task_start() 切换到题3到题6状态。
 * 每个分支只管本题的摆杆动作和计时，不直接控制小车循迹。
 * 题1和题2不经过这里，因为它们不需要启动摆杆控制。
 */
void gimbal_task_state(void)
{
    if (balance_rod_limit_test_enabled())
    {
        /* 摆杆限幅测试模式：只读 pitchmotor 反馈，不执行题3/4/5/6状态机。 */
        balance_rod_limit_test_update();
        return;
    }

    switch (gimbal_sm_obj.state)
    {
    case GIMBAL_IDLE:
        /* 空闲状态：摆杆回中并停止滚球控制，同时清空计时。 */
        ball_balance_stop();
        balance_reset_runtime();
        break;

    case BALANCE_TASK3_STATIC_PLUS_TO_MINUS:
        /* 题3：车静止，钢球 O 点 -> +5cm -> -5cm。 */
        balance_tick_time();
        balance_task3_static_pm5();
        break;

    case BALANCE_TASK4_CAR_TO_B_CENTER:
        /* 题4：小车从 A 到 B，钢球稳定在 O 点附近。
         * 目标位置是 0cm，时间上限是 8s。
         */
        balance_tick_time();
        balance_task_hold(0.0f, TASK4_AB_LIMIT_MS);
        break;

    case BALANCE_TASK5_CAR_LAP_CENTER:
        /* 题5：小车顺时针一圈，钢球稳定在 O 点附近。
         * 目标位置是 0cm，时间上限是 30s。
         */
        balance_tick_time();
        balance_task_hold(0.0f, TASK5_LAP_LIMIT_MS);
        break;

    case BALANCE_TASK6_CAR_LAP_SETPOINT:
        /* 题6：小车顺时针一圈，钢球稳定在任意指定位置附近。
         * 指定位置需要提前写入 sys.ctrl.ball_task6_target_cm。
         */
        balance_tick_time();
        balance_task_hold(sys.ctrl.ball_task6_target_cm, TASK6_LAP_LIMIT_MS);
        break;

    default:
        /* 旧云台菜单状态仍保留。
         * 如果误进入 GIMBAL_SEARCH_LEFT、GIMBAL_STATIC_TRACK 等旧状态，就走原来的云台逻辑，方便对照调试。
         */
        gimbal_task_state_legacy();
        break;
    }
}