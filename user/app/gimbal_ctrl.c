#include "common.h"
#include "math.h"

sys_t sys;
gimbal_sm_t gimbal_sm_obj = {.state = GIMBAL_IDLE};

volatile uint32_t target_lost_cnt = 0;
volatile uint8_t balance_state_machine_enable = 0U; // 默认不自动运行题号状态机，只接收视觉，菜单确认后启动题3

extern float yaw_pos;


/* ================= H题：按题号划分的新状态机 =================
 * 说明：
 * 1. 本工程只负责摆杆和钢球控制，不负责小车循迹。
 * 2. 题1图传、题2小车一圈计时都不启动摆杆，本状态机只处理题3到题6。
 * 3. 视觉入口在 camera_data_update()，当前约定 dx 表示钢球相对 O 点的位置偏差，单位 cm。
 * 4. pitchmotor 暂时作为摆杆电机使用，控制方式是张大头步进电机的位置速度模式。
 */

/* 钢球认为稳定的误差范围。题目要求误差绝对值不大于 1cm，所以这里先用 1cm。 */
#define BALL_STABLE_ERR_CM       1.0f

/* ================= 题3时间轨迹参数 =================
 * 总移动时间会按路程比例分配：0->+5 使用 1/3，+5->-5 使用 2/3。
 * 默认总时间 = 3300 + 300 + 500 = 4100ms，小于题目要求的 5000ms。
 */
#define BALL_TASK3_PLUS_CM       7.0f
#define TASK3_PLUS5_BIAS_CM      0.0f       // 偏置
#define BALL_TASK3_MINUS_CM     -5.65f
#define TASK3_TRAJECTORY_MOVE_TIME_MS       3300U
#define TASK3_PLUS5_WAIT_TIME_MS             300U
#define TASK3_MINUS5_WAIT_TIME_MS            500U
#define TASK3_REQUIRED_TOTAL_LIMIT_MS        5000U
#define TASK3_MOVE_TO_PLUS5_TIME_MS          (TASK3_TRAJECTORY_MOVE_TIME_MS / 3U)
#define TASK3_MOVE_TO_MINUS5_TIME_MS         (TASK3_TRAJECTORY_MOVE_TIME_MS - TASK3_MOVE_TO_PLUS5_TIME_MS)
#define TASK3_CONFIGURED_TOTAL_TIME_MS       (TASK3_TRAJECTORY_MOVE_TIME_MS + TASK3_PLUS5_WAIT_TIME_MS + TASK3_MINUS5_WAIT_TIME_MS)

#if (TASK3_CONFIGURED_TOTAL_TIME_MS > TASK3_REQUIRED_TOTAL_LIMIT_MS)
#error "Task 3 trajectory and wait time must not exceed 5000 ms"
#endif

volatile float dbg_task3_d_term_limit_deg = 2.0f;      // 题3 D 项输出限幅，避免 kd 一加摆杆就猛抽，单位 deg
volatile float dbg_task3_vel_used_cm_s = 0.0f;         // Watch 观察用：滤波后实际参与 D 项的速度

enum
{
    TASK3_PHASE_MOVE_TO_PLUS5 = 0,
    TASK3_PHASE_WAIT_AT_PLUS5,
    TASK3_PHASE_MOVE_TO_MINUS5,
    TASK3_PHASE_WAIT_AT_MINUS5,
    TASK3_PHASE_FINISHED_HOLD
};

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
    gimbal_sm_obj.task3_phase_elapsed_ms = 0;
    gimbal_sm_obj.finished = 0;          // 任务完成标志清零
    gimbal_sm_obj.task3_target_cmd_cm = 0.0f; // 题3平滑目标从 O 点开始
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
    balance_state_machine_enable = (state == GIMBAL_IDLE) ? 0U : 1U;
    gimbal_sm_obj.state = state;
    sys.ctrl.ball_target_cm = (state == BALANCE_TASK6_CAR_LAP_SETPOINT) ? sys.ctrl.ball_task6_target_cm : 0.0f;

}

/* 判断钢球是否已经稳定在当前目标点附近。
 * sys.value.ball_pos_cm 来自视觉滤波后的位置。
 * sys.ctrl.ball_target_cm 是当前题目希望钢球保持的位置。
 */
static uint8_t balance_ball_stable(void)
{
    return (fabsf(sys.value.ball_pos_cm - sys.ctrl.ball_target_cm) <= BALL_STABLE_ERR_CM) ? 1U : 0U;
}

static float task3_smooth_trajectory(float start_cm,
                                     float end_cm,
                                     uint32_t elapsed_ms,
                                     uint32_t duration_ms)
{
    float ratio;
    float smooth_ratio;

    if ((duration_ms == 0U) || (elapsed_ms >= duration_ms))
        return end_cm;

    ratio = (float)elapsed_ms / (float)duration_ms;
    smooth_ratio = ratio * ratio * (3.0f - 2.0f * ratio);
    return start_cm + (end_cm - start_cm) * smooth_ratio;
}

static void task3_enter_phase(uint8_t next_phase)
{
    gimbal_sm_obj.task3_phase = next_phase;
    gimbal_sm_obj.task3_phase_elapsed_ms = 0U;
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

/* 题3采用固定时间轨迹，不使用小球到位检测。 */
static void balance_task3_static_pm5(void)
{
    switch (gimbal_sm_obj.task3_phase)
    {
    case TASK3_PHASE_MOVE_TO_PLUS5:
        gimbal_sm_obj.task3_target_cmd_cm =
            task3_smooth_trajectory(0.0f,
                                    BALL_TASK3_PLUS_CM + TASK3_PLUS5_BIAS_CM,
                                    gimbal_sm_obj.task3_phase_elapsed_ms,
                                    TASK3_MOVE_TO_PLUS5_TIME_MS);
        ball_balance_static_ctrl(&sys, gimbal_sm_obj.task3_target_cmd_cm);
        if (++gimbal_sm_obj.task3_phase_elapsed_ms >= TASK3_MOVE_TO_PLUS5_TIME_MS)
            task3_enter_phase(TASK3_PHASE_WAIT_AT_PLUS5);
        break;

    case TASK3_PHASE_WAIT_AT_PLUS5:
        gimbal_sm_obj.task3_target_cmd_cm = BALL_TASK3_PLUS_CM;
        ball_balance_static_ctrl(&sys, gimbal_sm_obj.task3_target_cmd_cm);
        if (++gimbal_sm_obj.task3_phase_elapsed_ms >= TASK3_PLUS5_WAIT_TIME_MS)
            task3_enter_phase(TASK3_PHASE_MOVE_TO_MINUS5);
        break;

    case TASK3_PHASE_MOVE_TO_MINUS5:
        gimbal_sm_obj.task3_target_cmd_cm =
            task3_smooth_trajectory(BALL_TASK3_PLUS_CM,
                                    BALL_TASK3_MINUS_CM,
                                    gimbal_sm_obj.task3_phase_elapsed_ms,
                                    TASK3_MOVE_TO_MINUS5_TIME_MS);
        ball_balance_static_ctrl(&sys, gimbal_sm_obj.task3_target_cmd_cm);
        if (++gimbal_sm_obj.task3_phase_elapsed_ms >= TASK3_MOVE_TO_MINUS5_TIME_MS)
            task3_enter_phase(TASK3_PHASE_WAIT_AT_MINUS5);
        break;

    case TASK3_PHASE_WAIT_AT_MINUS5:
        gimbal_sm_obj.task3_target_cmd_cm = BALL_TASK3_MINUS_CM;
        ball_balance_static_ctrl(&sys, gimbal_sm_obj.task3_target_cmd_cm);
        if (++gimbal_sm_obj.task3_phase_elapsed_ms >= TASK3_MINUS5_WAIT_TIME_MS)
        {
            task3_enter_phase(TASK3_PHASE_FINISHED_HOLD);
            gimbal_sm_obj.finished = 1U;
        }
        break;

    case TASK3_PHASE_FINISHED_HOLD:
    default:
        gimbal_sm_obj.task3_target_cmd_cm = BALL_TASK3_MINUS_CM;
        ball_balance_static_ctrl(&sys, gimbal_sm_obj.task3_target_cmd_cm);
        gimbal_sm_obj.finished = 1U;
        break;
    }
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
        /* 摆杆机械限幅测试模式：失能 pitchmotor，只读反馈，不执行题3/4/5/6状态机。 */
        balance_rod_limit_test_update();
        return;
    }

    if (balance_rod_cmd_limit_test_enabled())
    {
        /* 摆杆状态反馈调试模式：Keil 手动调 kp/kd 和目标位置，位置/速度由视觉更新。 */
        balance_rod_cmd_limit_test_update();
        return;
    }

    if (!balance_state_machine_enable)
    {
        /* 上电及菜单选择态：始终运行静止串级位置环，把钢球保持在 O 点。
         * 这里只做 0cm 定点保持，不运行任何题号轨迹和任务计时。
         */
        gimbal_sm_obj.state = GIMBAL_IDLE;
        sys.ctrl.ball_target_cm = 0.0f;
        ball_balance_static_ctrl(&sys, 0.0f);
        return;
    }

    switch (gimbal_sm_obj.state)
    {
    case GIMBAL_IDLE:
        /* 防御分支：即使空闲状态被使能，也只保持钢球在 O 点。 */
        ball_balance_static_ctrl(&sys, 0.0f);
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
        /* 非 H 题状态统一回空闲，不再保留旧激光云台状态机。 */
        balance_task_start(GIMBAL_IDLE);
        break;
    }
}
