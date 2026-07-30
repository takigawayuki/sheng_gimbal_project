#include "common.h"
#include "math.h"

sys_t sys;
gimbal_sm_t gimbal_sm_obj = {GIMBAL_IDLE, 0, 0, 0, 0, 0.0f};

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

/* 题3的目标路径：O 点 -> +5cm -> O 点 -> -5cm。 */
#define BALL_TASK3_PLUS_CM       5.0f
#define BALL_TASK3_MINUS_CM     -5.0f
/* 题3把 +5cm 和 O 点都当过程点，不当长时间稳定点。
 * 进入 +5cm 附近后先回 O 点减速，经过 O 点后再继续去 -5cm。
 */
#define BALL_TASK3_PLUS_HOLD_MS  30U

/* ================= 题3 Keil Watch 集中调试变量 =================
 * 后续调题3时，Keil Watch 里直接搜 dbg_task3_。
 * 这些变量不用重编就能改，方便现场只改一个区域，不用在多个文件里找宏。
 */
volatile float dbg_task3_turn_margin_cm = 1.0f;        // +5cm 折返点提前量，球到 5-margin 后切回 O 点
volatile float dbg_task3_center_margin_cm = 0.5f;      // +5cm 回 O 点时，球进 O 点附近多少 cm 后进入去 -5cm 阶段
volatile float dbg_task3_target_reached_cm = 0.2f;     // 平滑目标回到 O 点附近的判断阈值
volatile float dbg_task3_target_slew_cm_s = 6.0f;      // 题3前两段目标斜坡速度，单位 cm/s
volatile float dbg_task3_minus_slew_cm_s = 4.0f;       // 题3第三段去 -5cm 的目标斜坡速度
volatile float dbg_task3_minus_brake_start_cm = -3.5f; // 题3第三段提前刹车起点，球到该位置后降低目标斜坡速度
volatile float dbg_task3_minus_brake_slew_cm_s = 0.5f; // 题3第三段提前刹车后的慢速斜坡
volatile float dbg_task3_overrun_margin_cm = 0.3f;     // 超限柔性回拉触发余量，超过目标多少 cm 且继续向外滚时启用
volatile float dbg_task3_overrun_vel_cm_s = 1.0f;      // 超限柔性回拉速度门槛，过滤小速度噪声
volatile float dbg_task3_overrun_pullback_cm = 0.0f;   // 超限后临时目标往 O 点方向退多少 cm，不再直接拉回 O 点
volatile float dbg_task3_minus_output_scale = 1.0f;   // 题3第三阶段去 -5cm 时的 PID 输出缩放
volatile float dbg_task3_minus_balance_ff_deg = 0.0f;  // 题3第三阶段 -5cm 静态平衡补偿角度，单位 deg
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

static uint8_t balance_task3_plus_turn_reached(void)
{
    /* +5cm 是折返点，不是最终停车点。
     * 注意：这里不能用 stable_ms 判断。题3平滑目标从 0cm 起步，球一开始就在 O 点附近，
     * 如果用 stable_ms 会误判为已经到达 +5cm，导致状态机直接跳到 -5cm。
     */
    return (sys.value.ball_pos_cm >= (BALL_TASK3_PLUS_CM - dbg_task3_turn_margin_cm)) ? 1U : 0U;
}

static uint8_t balance_task3_center_reached(void)
{
    /* 从 +5cm 折返回来时，经过 O 点附近就进入第三阶段。
     * 必须同时满足“平滑目标已经回到 O 点附近”和“球也回到 O 点附近”，
     * 防止第二阶段刚开始就因为球还在原点附近而直接跳到 -5cm。
     */
    if (fabsf(gimbal_sm_obj.task3_target_cmd_cm) > dbg_task3_target_reached_cm)
        return 0U;

    return (sys.value.ball_pos_cm <= dbg_task3_center_margin_cm) ? 1U : 0U;
}
static float balance_task3_minus_slew_cm_s(void)
{
    /* 题3第三阶段提前刹车：
     * 从 O 点去 -5cm 时，球到 -3.5cm 附近就把虚拟目标移动速度降下来。
     * 这样到 -5cm 前先减速，减少到点后靠 PID 来回救的次数。
     */
    if (sys.value.ball_pos_cm <= dbg_task3_minus_brake_start_cm)
        return dbg_task3_minus_brake_slew_cm_s;

    return dbg_task3_minus_slew_cm_s;
}
static float balance_slew_target_cm(float current_cm, float target_cm, float slew_cm_s)
{
    float step_cm = slew_cm_s * 0.001f;

    /* 题3目标斜坡：PID 看到的是一个会移动的虚拟终点，而不是 +5cm 到 -5cm 的瞬间跳变。
     * 这样折返时摆杆不会一下子打到极限，钢球速度更容易被拉住。
     */
    if (current_cm < (target_cm - step_cm))
        return current_cm + step_cm;

    if (current_cm > (target_cm + step_cm))
        return current_cm - step_cm;

    return target_cm;
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

/* 题3：小车静止，控制钢球从 O 点到 +5cm，再回到 O 点减速，最后运行到 -5cm。
 * task3_phase = 0：控制目标为 +5cm。
 * task3_phase = 1：控制目标为 O 点，用中点当阶段终点来压速度。
 * task3_phase = 2：控制目标为 -5cm，并稳定在 -5cm 附近。
 */
static void balance_task3_static_pm5(void)
{
    if (gimbal_sm_obj.task3_phase == 0U)
    {
        /* 第一阶段：让钢球向 +5cm 运动。
         * 这里给 PID 的不是硬目标 +5cm，而是逐步靠近 +5cm 的平滑目标。
         */
        gimbal_sm_obj.task3_target_cmd_cm = balance_slew_target_cm(gimbal_sm_obj.task3_target_cmd_cm,
                                                                    BALL_TASK3_PLUS_CM,
                                                                    dbg_task3_target_slew_cm_s);
        ball_balance_static_ctrl(&sys, gimbal_sm_obj.task3_target_cmd_cm);
        balance_update_stable_counter();

        /* 到 +5cm 折返点附近后，不直接去 -5cm，先回 O 点压速度。 */
        if (balance_task3_plus_turn_reached())
        {
            gimbal_sm_obj.task3_phase = 1U;
            gimbal_sm_obj.stable_ms = 0;

            /* 切换过程点时清掉 PID 的积分和上次误差，减少折返瞬间拖尾。 */
            sys.camera_x_pid.i_term = 0.0f;
            sys.camera_x_pid.pre_err = 0.0f;
        }
    }
    else if (gimbal_sm_obj.task3_phase == 1U)
    {
        /* 第二阶段：从 +5cm 折返回 O 点。
         * O 点只是过程终点，用来把球速压下来，不在这里长时间停留。
         */
        gimbal_sm_obj.task3_target_cmd_cm = balance_slew_target_cm(gimbal_sm_obj.task3_target_cmd_cm,
                                                                    0.0f,
                                                                    dbg_task3_target_slew_cm_s);
        ball_balance_static_ctrl(&sys, gimbal_sm_obj.task3_target_cmd_cm);
        balance_update_stable_counter();

        /* 经过 O 点附近后，再进入第三阶段去 -5cm。 */
        if (balance_task3_center_reached())
        {
            gimbal_sm_obj.task3_phase = 2U;
            gimbal_sm_obj.stable_ms = 0;

            sys.camera_x_pid.i_term = 0.0f;
            sys.camera_x_pid.pre_err = 0.0f;
        }
    }
    else
    {
        /* 第三阶段：让钢球从 O 点附近运行到 -5cm，并稳定在 -5cm 附近。 */
        gimbal_sm_obj.task3_target_cmd_cm = balance_slew_target_cm(gimbal_sm_obj.task3_target_cmd_cm,
                                                                    BALL_TASK3_MINUS_CM,
                                                                    balance_task3_minus_slew_cm_s());
        ball_balance_static_ctrl(&sys, gimbal_sm_obj.task3_target_cmd_cm);
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
        /* 默认只接收视觉，不自动回中、不跑题号状态机、不下发摆杆命令。 */
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
        /* 题3：车静止，钢球 O 点 -> +5cm -> O 点 -> -5cm。 */
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
