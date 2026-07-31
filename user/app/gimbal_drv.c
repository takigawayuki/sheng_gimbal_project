#include "common.h"
#include "ZhangDaTou.h"
#include "math.h"

void period_init(void);
void gimbal_init(void)
{
    period_init();
    balance_init();
}

void period_init(void)
{
    sys.period.sys_fs = 1000;                     // 系统运行频率 1000Hz
    sys.period.sys_ts = 1.0f / sys.period.sys_fs; // 系统周期 = 1 / 系统频率 = 1ms

    // 俯仰角pid运行频率初始化
    sys.period.camera_y_pid_fs = 1000;                                                // Y 轴 PID运行频率 1000Hz
    sys.period.camera_y_pid_ts = 1.0f / sys.period.camera_y_pid_fs;                   // PID周期 = 1 / PID频率 = 1ms
    sys.period.camera_y_pid_cnt_val = sys.period.sys_fs * sys.period.camera_y_pid_ts; // 每 1 次系统任务 → 跑 1 次 PID

    // 偏航角pid运行频率初始化
    sys.period.camera_x_pid_fs = 1000; // X 轴 PID运行频率 1000Hz
    sys.period.camera_x_pid_ts = 1.0f / sys.period.camera_x_pid_fs;
    sys.period.camera_x_pid_cnt_val = sys.period.sys_fs * sys.period.camera_x_pid_ts;

    // 俯仰角pid运行频率初始化
    sys.period.camera_y_pid_run_fs = 1000;                                                    // Y 轴 PID运行频率 1000Hz
    sys.period.camera_y_pid_run_ts = 1.0f / sys.period.camera_y_pid_run_fs;                   // PID周期 = 1 / PID频率 = 1ms
    sys.period.camera_y_pid_cnt_run_val = sys.period.sys_fs * sys.period.camera_y_pid_run_ts; // 每 1 次系统任务 → 跑 1 次 PID

    // 偏航角pid运行频率初始化
    sys.period.camera_x_pid_run_fs = 1000; // X 轴 PID运行频率 1000Hz
    sys.period.camera_x_pid_run_ts = 1.0f / sys.period.camera_x_pid_run_fs;
    sys.period.camera_x_pid_cnt_run_val = sys.period.sys_fs * sys.period.camera_x_pid_run_ts;
}

/* ================= H题：摆杆滚球控制 =================
 * pitchmotor 复用为摆杆电机。
 * ROD_CENTER_DEG / ROD_MIN_DEG / ROD_MAX_DEG 必须实测后填写。
 * 题3使用静态状态反馈；题4/5/6使用运动状态反馈，并预留底盘前馈补偿量。
 * 测摆杆机械限幅时，把 ROD_LIMIT_TEST_ENABLE 改成 1U。
 * 手动调状态反馈时，打开 rod_cmd_limit_test_enable，反馈位置和速度由视觉更新。
 */
#define ROD_CENTER_DEG          0.0f        // 摆杆物理水平位置对应的 pitchmotor 反馈角度，必须实测填写
#define ROD_MIN_DEG            -10.0f
#define ROD_MAX_DEG             8.0f
#define ROD_DEFAULT_SPEED_DPS  1000.0f
#define ROD_DEFAULT_ACC      1000U
#define BALL_I_TERM_LIMIT_DEG 10000.0f       // H题积分项默认限幅，单位 deg；积分调试先集中看 sys.camera_x_pid.i_term
#define BALL_TASK3_MINUS_TARGET_CM -5.0f // 题3最终目标 -5cm，gimbal_drv.c 内部用于到点制动触发保护

/* 题3调试参数已经集中定义在 gimbal_ctrl.c 的 dbg_task3_ 区域。
 * Keil Watch 里直接搜 dbg_task3_，这里不再单独放一组宏，避免现场调试到处找。
 */
/* 摆杆限幅测试开关。
 * 0U：正常运行题3/4/5/6状态机和 PID。
 * 1U：进入测试后会失能 pitchmotor，只读取反馈角度，不执行状态机 PID，不给摆杆下发位置命令。
 * 测试方法：打开后烧录，手动转摆杆到机械最低/最高安全位置，记录 pitch_pos 或 sys.value.rod_angle_deg。
 * 记录值分别填入 ROD_MIN_DEG / ROD_MAX_DEG，水平位置填入 ROD_CENTER_DEG。
 */
#define ROD_LIMIT_TEST_ENABLE   0U

/* 摆杆状态反馈限幅调试变量。
 * 这些变量是给 Keil Watch 手动改的，不需要每次重编。
 * rod_cmd_limit_test_enable = 1：进入状态反馈限幅测试模式，状态机不跑题3/4/5/6。
 * sys.ctrl.ball_target_cm：手动给 PID 目标位置，单位 cm。
 * sys.value.ball_pos_cm：视觉更新的钢球位置反馈，单位 cm。
 * sys.camera_x_pid.kp/ki/kd 在 Keil 里直接改：kp 是位置增益，ki 是小量积分补偿，kd 是速度阻尼增益。
 * 最终命令 = ROD_CENTER_DEG + 状态反馈输出，然后经过 ROD_MIN_DEG/ROD_MAX_DEG 限幅。
 */
volatile uint8_t rod_cmd_limit_test_enable = 0U;
volatile uint32_t rod_pid_test_run_cnt = 0U;

/* 运动题前馈补偿角度的安全限幅。
 * 先占位用，默认前馈为 0deg；后面底盘给出加速度/速度补偿后，再把值写入 sys.ctrl.rod_chassis_ff_deg。
 */
#define ROD_CHASSIS_FF_MIN_DEG -5.0f
#define ROD_CHASSIS_FF_MAX_DEG  5.0f

static void ball_balance_apply_cmd(sys_t *sys_obj, float rod_cmd);

static float balance_clampf(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void ball_pid_reset(pid_para_t *pid)
{
    pid->i_term = 0.0f;
    pid->pre_err = 0.0f;
    pid->out_value = 0.0f;
}

static float ball_state_feedback_calc(pid_para_t *pid, float target_cm, float pos_cm, float vel_cm_s)
{
    static float vel_filter_cm_s = 0.0f;
    float vel_error_cm_s;
    float alpha;

    /* 状态反馈外环：位置误差 + 速度阻尼。
     * kp：位置增益，把钢球拉向目标位置。
     * ki：小量积分，用来补偿摆杆零点误差或固定斜坡偏置。
     * kd：速度阻尼，钢球速度越大，越提前反向刹车。
     * 积分必须限幅，滚球系统不能让积分无限累加。
     */
    pid->ref_value = target_cm;
    pid->fback_value = pos_cm;
    pid->error = pid->ref_value - pid->fback_value;

    alpha = balance_clampf(dbg_task3_vel_filter_alpha, 0.0f, 1.0f);
    vel_filter_cm_s = vel_filter_cm_s * (1.0f - alpha) + vel_cm_s * alpha;
    dbg_task3_vel_used_cm_s = vel_filter_cm_s;
    vel_error_cm_s = 0.0f - vel_filter_cm_s;

    pid->p_term = pid->kp * pid->error;
    pid->i_term += pid->ki * pid->error * pid->ts;
    pid->i_term = balance_clampf(pid->i_term, pid->i_term_min, pid->i_term_max);
    pid->d_term = pid->kd * vel_error_cm_s;
    pid->d_term = balance_clampf(pid->d_term,
                                 -dbg_task3_d_term_limit_deg,
                                  dbg_task3_d_term_limit_deg);
    pid->pre_err = pid->error;

    pid->out_value = pid->p_term + pid->i_term + pid->d_term;

    if (pid->out_value > pid->out_max)
        pid->out_value = pid->out_max;
    else if (pid->out_value < pid->out_min)
        pid->out_value = pid->out_min;

    return pid->out_value;
}

uint8_t balance_rod_limit_test_enabled(void)
{
#if ROD_LIMIT_TEST_ENABLE
    return 1U;
#else
    return 0U;
#endif
}

void balance_rod_limit_test_update(void)
{
#if ROD_LIMIT_TEST_ENABLE
    static uint8_t pitch_disable_sent = 0U;

    /* 限幅测试模式只读电机反馈，不下发任何摆杆控制命令。
     * 第一次进入测试模式时失能 pitchmotor，这样可以手动转动摆杆。
     */
    if (!pitch_disable_sent)
    {
        ZhangDaTou_Enable(&pitchmotor, 0);
        pitch_disable_sent = 1U;
    }

    yaw_pos = ZhangDaTou_getPositionDate(&yawmotor);
    pitch_pos = ZhangDaTou_getPositionDate(&pitchmotor);

    sys.value.rod_angle_deg = pitch_pos;
    sys.ctrl.rod_angle_cmd_deg = pitch_pos;

    ball_pid_reset(&sys.camera_x_pid);
    ball_pid_reset(&sys.camera_x_pid_run);
#endif
}

uint8_t balance_rod_cmd_limit_test_enabled(void)
{
    return (rod_cmd_limit_test_enable != 0U) ? 1U : 0U;
}

void balance_rod_cmd_limit_test_update(void)
{
    static uint8_t pitch_enable_sent = 0U;
    float rod_cmd;

    /* 状态反馈限幅调试模式会真正运行 sys.camera_x_pid。
     * 在 Keil Watch 里手动修改：
     * 1. sys.camera_x_pid.kp：小球位置误差增益
     * 2. sys.camera_x_pid.kd：小球速度阻尼增益，sys.camera_x_pid.ki 暂时不用
     * 3. sys.ctrl.ball_target_cm 作为小球目标位置，单位 cm
     * 4. sys.value.ball_pos_cm / sys.value.ball_vel_cm_s 作为视觉反馈，单位 cm / cm/s
     * 然后观察 rod_pid_test_run_cnt、sys.camera_x_pid.error/p_term/d_term/out_value 和 sys.ctrl.rod_angle_cmd_deg。
     * 注意：这里不覆盖 sys.value.ball_pos_cm，方便直接使用视觉实时反馈。
     */
    if (!pitch_enable_sent)
    {
        /* 这里在 TIM 中断里执行，不能调用 HAL_Delay()，否则 PID 可能根本跑不到。 */
        ZhangDaTou_Enable(&pitchmotor, 1);
        pitch_enable_sent = 1U;
    }

    rod_pid_test_run_cnt++;
    sys.value.ball_pos_raw_cm = sys.value.ball_pos_cm;
    sys.ctrl.rod_chassis_ff_deg = 0.0f;

    ball_state_feedback_calc(&sys.camera_x_pid,
                             sys.ctrl.ball_target_cm,
                             sys.value.ball_pos_cm,
                             sys.value.ball_vel_cm_s);

    rod_cmd = ROD_CENTER_DEG + sys.camera_x_pid.out_value;
    ball_balance_apply_cmd(&sys, rod_cmd);
}

void balance_init(void)
{
    sys.ctrl.ball_target_cm = 0.0f;
    sys.ctrl.ball_task6_target_cm = 0.0f;
    sys.ctrl.rod_angle_cmd_deg = ROD_CENTER_DEG;
    sys.ctrl.rod_chassis_ff_deg = 0.0f;

    /* 题3静止滚球 PID：钢球位置 cm -> 摆杆角度 deg。
     * 静止时没有底盘加减速扰动，先只靠位置闭环。
     * 方向不对就反转 kp/ki/kd 符号。
     */
    sys.camera_x_pid.kp = 0.65f;
    sys.camera_x_pid.ki = 0.06f;
    sys.camera_x_pid.kd = 0.01f;
    // sys.camera_x_pid.kp = 1.5f;
    // sys.camera_x_pid.ki = 0.75f;
    // sys.camera_x_pid.kd = 0.01f;

    sys.camera_x_pid.ts = 0.001f;
    sys.camera_x_pid.out_max = 20000.0f;
    sys.camera_x_pid.out_min = -20000.0f;
    sys.camera_x_pid.i_term_max = BALL_I_TERM_LIMIT_DEG;
    sys.camera_x_pid.i_term_min = -BALL_I_TERM_LIMIT_DEG;

    /* 题4/5/6运动滚球 PID：钢球位置 cm -> 摆杆角度 deg。
     * 小车运动时会叠加底盘前馈 sys.ctrl.rod_chassis_ff_deg。
     * 这套参数和题3分开调，避免静止题和运动题互相影响。
     */
    sys.camera_x_pid_run.kp = 0.65f;
    sys.camera_x_pid_run.ki = 0.06f;
    sys.camera_x_pid_run.kd = 0.01f;
    sys.camera_x_pid_run.ts = 0.001f;
    sys.camera_x_pid_run.out_max = 20000.0f;
    sys.camera_x_pid_run.out_min = -20000.0f;
    sys.camera_x_pid_run.i_term_max = BALL_I_TERM_LIMIT_DEG;
    sys.camera_x_pid_run.i_term_min = -BALL_I_TERM_LIMIT_DEG;

    pitchmotor.setSpeed = ROD_DEFAULT_SPEED_DPS;
    pitchmotor.setAcc = ROD_DEFAULT_ACC;
    pitchmotor.mod = 1;
}

void ball_balance_stop(void)
{
    ball_pid_reset(&sys.camera_x_pid);
    ball_pid_reset(&sys.camera_x_pid_run);
    sys.ctrl.rod_angle_cmd_deg = ROD_CENTER_DEG;
    sys.ctrl.rod_chassis_ff_deg = 0.0f;

    /* 回中也走 ball_balance_apply_cmd()，保证 ROD_CENTER_DEG 写错或未实测时不会越过 pitch 轴限幅。 */
    ball_balance_apply_cmd(&sys, ROD_CENTER_DEG);
}

// 静止时的一套
void ball_balance_static_ctrl(sys_t *sys_obj, float target_cm)
{
    float rod_cmd;
    float vel_brake_deg;
    float brake_vel_cm_s;

    sys_obj->ctrl.ball_target_cm = target_cm;
    dbg_task3_ff_applied_deg = 0.0f;

    /* 题3第一阶段开环测试：0->+5cm 直接给固定摆杆角度，不运行 PID。
     * 用来把“送到 +5”和“PID 从 +5 到 -5 停住”两个问题拆开。
     */
    if ((gimbal_sm_obj.state == BALANCE_TASK3_STATIC_PLUS_TO_MINUS) &&
        (gimbal_sm_obj.task3_phase == 0U) &&
        (dbg_task3_plus_openloop_enable != 0U))
    {
        dbg_task3_ff_applied_deg = dbg_task3_plus_openloop_deg;
        rod_cmd = ROD_CENTER_DEG + dbg_task3_plus_openloop_deg;
        ball_balance_apply_cmd(sys_obj, rod_cmd);
        return;
    }

    dbg_task3_minus_hold_active = 0U;
    dbg_task3_minus_overrun_ff_applied_deg = 0.0f;

    /* 题3主策略：位置误差 + 视觉速度阻尼 -> 摆杆目标角度。
     * 这里只保留很薄的一层分段固定补偿，便于 Keil Watch 现场调试。
     */
    ball_state_feedback_calc(&sys_obj->camera_x_pid,
                             target_cm,
                             sys_obj->value.ball_pos_cm,
                             sys_obj->value.ball_vel_cm_s);

    if (gimbal_sm_obj.state == BALANCE_TASK3_STATIC_PLUS_TO_MINUS)
    {
        if (gimbal_sm_obj.task3_phase == 0U)
        {
            dbg_task3_ff_applied_deg = dbg_task3_plus_ff_deg;
        }
        else if (gimbal_sm_obj.task3_phase == 2U)
        {
            dbg_task3_ff_applied_deg = dbg_task3_minus_ff_deg;


            /* 第三阶段从 +5cm 拉到 -5cm 时，额外按球速反向刹车。
             * 这层只负责吸收大幅往返的能量，不改变位置目标。
             */
            if (fabsf(dbg_task3_vel_used_cm_s) <= dbg_task3_minus_vel_brake_dead_cm_s)
            {
                brake_vel_cm_s = 0.0f;
            }
            else if (dbg_task3_vel_used_cm_s > 0.0f)
            {
                brake_vel_cm_s = dbg_task3_vel_used_cm_s - dbg_task3_minus_vel_brake_dead_cm_s;
            }
            else
            {
                brake_vel_cm_s = dbg_task3_vel_used_cm_s + dbg_task3_minus_vel_brake_dead_cm_s;
            }

            vel_brake_deg = -dbg_task3_minus_vel_brake_kd * brake_vel_cm_s;
            vel_brake_deg = balance_clampf(vel_brake_deg,
                                           -dbg_task3_minus_vel_brake_limit_deg,
                                            dbg_task3_minus_vel_brake_limit_deg);
            dbg_task3_minus_vel_brake_deg = vel_brake_deg;
            dbg_task3_ff_applied_deg += vel_brake_deg;

            if ((sys_obj->value.ball_pos_cm <= dbg_task3_minus_overrun_start_cm) &&
                (dbg_task3_vel_used_cm_s <= dbg_task3_minus_overrun_vel_cm_s))
            {
                dbg_task3_minus_overrun_ff_applied_deg = dbg_task3_minus_overrun_ff_deg;
                dbg_task3_ff_applied_deg += dbg_task3_minus_overrun_ff_applied_deg;
            }
            else
            {
                dbg_task3_minus_overrun_ff_applied_deg = 0.0f;
            }
        }
        else
        {
            dbg_task3_minus_vel_brake_deg = 0.0f;
        }

        sys_obj->camera_x_pid.out_value += dbg_task3_ff_applied_deg;
    }

    rod_cmd = ROD_CENTER_DEG + sys_obj->camera_x_pid.out_value;
    ball_balance_apply_cmd(sys_obj, rod_cmd);
}
// 运动时的一套
void ball_balance_set_chassis_ff(float ff_deg)
{
    /* 底盘前馈补偿角度占位接口。
     * 现在可以一直传 0；后面底盘给出加速度/速度补偿量时，在这里写入即可。
     */
    sys.ctrl.rod_chassis_ff_deg = balance_clampf(ff_deg, ROD_CHASSIS_FF_MIN_DEG, ROD_CHASSIS_FF_MAX_DEG);
}

// 运动时的一套
void ball_balance_running_ctrl(sys_t *sys_obj, float target_cm)
{
    float rod_cmd;
    float chassis_ff_deg;

    sys_obj->ctrl.ball_target_cm = target_cm;
    ball_state_feedback_calc(&sys_obj->camera_x_pid_run,
                             target_cm,
                             sys_obj->value.ball_pos_cm,
                             sys_obj->value.ball_vel_cm_s);

    /* 运动题前馈占位量。
     * 当前默认是 0deg，不影响闭环；后续底盘补偿只需要改 sys.ctrl.rod_chassis_ff_deg。
     */
    chassis_ff_deg = balance_clampf(sys_obj->ctrl.rod_chassis_ff_deg, ROD_CHASSIS_FF_MIN_DEG, ROD_CHASSIS_FF_MAX_DEG);

    rod_cmd = ROD_CENTER_DEG + sys_obj->camera_x_pid_run.out_value + chassis_ff_deg;
    ball_balance_apply_cmd(sys_obj, rod_cmd);
}

static void ball_balance_apply_cmd(sys_t *sys_obj, float rod_cmd)
{
    /* pitch 轴最终安全限幅。
     * 所有题3/4/5/6的摆杆目标角度，最后都必须经过这里再下发给 pitchmotor。
     */
    rod_cmd = balance_clampf(rod_cmd, ROD_MIN_DEG, ROD_MAX_DEG);

    sys_obj->ctrl.rod_angle_cmd_deg = rod_cmd;         // 记录最终发给摆杆电机的目标角度，单位 deg
    sys_obj->value.rod_angle_deg = pitchmotor.Position; // 记录摆杆当前实际角度，这里 pitchmotor 就是摆杆电机

    ZhangDaTou_PositionSpeedctr(&pitchmotor, ROD_DEFAULT_SPEED_DPS, rod_cmd, ROD_DEFAULT_ACC);
    ZhangDaTou_Control(&pitchmotor);
}
