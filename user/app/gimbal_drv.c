#include "common.h"
#include "ZhangDaTou.h"

void period_init(void);
void pid_init(void);

uint8_t gimbal_pid_base_ready = 0U;
static float yaw_pid_base_pos = 0.0f;
static float pitch_pid_base_pos = 0.0f;
// float test_x = 0;
// float kp = 0, kd = 0, ki = 0;
// float error = 0, pre_error = 0;
// float i = 0, i_last = 0;
// float p_term = 0, i_term = 0, d_term = 0;

static void gimbal_pid_base_update_once(void)
{
    if (!gimbal_pid_base_ready)
    {
        yaw_pid_base_pos = yawmotor.Position;
        pitch_pid_base_pos = pitchmotor.Position;
        gimbal_pid_base_ready = 1U;
    }
}

void gimbal_pid_base_update_now(void)
{
    yaw_pid_base_pos = yawmotor.Position;
    pitch_pid_base_pos = pitchmotor.Position;
    gimbal_pid_base_ready = 1U;
}

void gimbal_init(void)
{
    period_init();
    pid_init();
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

void pid_init(void)
{
    // pitch
    sys.camera_y_pid.kp = 0.040f;
    sys.camera_y_pid.ki = 5.0f;
    sys.camera_y_pid.kd = 0.0f;
    sys.camera_y_pid.out_max = 200.0f;
    sys.camera_y_pid.out_min = -200.0f;
    sys.camera_y_pid.i_term_max = 150.0f;
    sys.camera_y_pid.i_term_min = -150.0f;
    sys.camera_y_pid.ts = sys.period.camera_y_pid_ts;
    sys.camera_y_pid.i_isolate_flag = 0U;

    // yaw
    // sys.camera_x_pid.kp = -0.012f;
    // sys.camera_x_pid.ki = 10.0f;
    // sys.camera_x_pid.kd = 0.008f;
    sys.camera_x_pid.kp = -0.012f;
    sys.camera_x_pid.ki = 8.0f;
    sys.camera_x_pid.kd = 0.008f;

    sys.camera_x_pid.out_max = 200.0f;
    sys.camera_x_pid.out_min = -200.0f;
    sys.camera_x_pid.i_term_max = 150.0f;
    sys.camera_x_pid.i_term_min = -150.0f;
    sys.camera_x_pid.ts = sys.period.camera_x_pid_ts;
    sys.camera_x_pid.i_isolate_flag = 0U;

    // pitch
    sys.camera_y_pid_run.kp = 0.040f;
    sys.camera_y_pid_run.ki = 5.0f;
    sys.camera_y_pid_run.kd = 0.0f;
    sys.camera_y_pid_run.out_max = 200.0f;
    sys.camera_y_pid_run.out_min = -200.0f;
    sys.camera_y_pid_run.i_term_max = 150.0f;
    sys.camera_y_pid_run.i_term_min = -150.0f;
    sys.camera_y_pid_run.ts = sys.period.camera_y_pid_run_ts;
    sys.camera_y_pid_run.i_isolate_flag = 0U;

    // yaw
    sys.camera_x_pid_run.kp = -0.015f;
    sys.camera_x_pid_run.ki = 12.0f;
    sys.camera_x_pid_run.kd = 0.050f;
    sys.camera_x_pid_run.out_max = 2000.0f;
    sys.camera_x_pid_run.out_min = -2000.0f;
    sys.camera_x_pid_run.i_term_max = 1500.0f;
    sys.camera_x_pid_run.i_term_min = -1500.0f;
    sys.camera_x_pid_run.ts = sys.period.camera_x_pid_run_ts;
    sys.camera_x_pid_run.i_isolate_flag = 0U;
}

#define PITCH_MAX 50.0f
#define PITCH_MIN -30.0f
extern float pitch_pos;

void camera_y_pid_ctrl(sys_t *sys, float ref_value)
{
    gimbal_pid_base_update_once();

    if (++sys->period.camera_y_pid_cnt >= sys->period.camera_y_pid_cnt_val)
    {
        sys->period.camera_y_pid_cnt = 0;
        parallel_pid_ctrl(&sys->camera_y_pid, ref_value, sys->value.camera_y);
    }

    // 速度模式
    // 张大头电机控制
    // 这个变量名字要改
    //     float motor_speed = sys->camera_y_pid.out_value;

    //    // 限制俯仰角在安全范围内，防止过度旋转导致机械损伤
    //     if ((pitchmotor.Position > PITCH_MAX && motor_speed > 0.0f) ||
    //         (pitchmotor.Position < PITCH_MIN && motor_speed < 0.0f))
    //     {
    //         // ZhangDaTou_Speedctr(&pitchmotor, 0.0f, 0);
    //         ZhangDaTou_PositionSpeedctr(&pitchmotor, 0.0f, 0.0f, 0);
    //     }
    //     else
    //     {
    //         // ZhangDaTou_Speedctr(&pitchmotor, motor_speed, 1000);
    //         ZhangDaTou_PositionSpeedctr(&pitchmotor, pitchmotor.setSpeed, motor_speed, yawmotor.setAcc);
    //     }

    //     ZhangDaTou_Control(&pitchmotor);

    // 张大头电机控制
    float pos_delta = sys->camera_y_pid.out_value; // 位置模式PID的输出是角度的修正量，单位是deg
    float pitch_goal = pitch_pid_base_pos + pos_delta;

    // 限制俯仰角在安全范围内，防止过度旋转导致机械损伤
    if (pitch_goal > PITCH_MAX)
        pitch_goal = PITCH_MAX;
    else if (pitch_goal < PITCH_MIN)
        pitch_goal = PITCH_MIN;

    if ((pitchmotor.Position > PITCH_MAX && pitch_goal > pitchmotor.Position) ||
        (pitchmotor.Position < PITCH_MIN && pitch_goal < pitchmotor.Position))
    {
        // ZhangDaTou_Speedctr(&pitchmotor, 0.0f, 0);
        ZhangDaTou_PositionSpeedctr(&pitchmotor, pitchmotor.setSpeed, 0.0f, 0);
    }
    else
    {
        // ZhangDaTou_Speedctr(&pitchmotor, motor_speed, pitchmotor.setAcc);
        ZhangDaTou_PositionSpeedctr(&pitchmotor, pitchmotor.setSpeed, pitch_goal, pitchmotor.setAcc);
    }

    ZhangDaTou_Control(&pitchmotor);
}

#define YAW_MAX 50.0f
#define YAW_MIN -50.0f
extern float yaw_pos;

void camera_x_pid_ctrl(sys_t *sys, float ref_value)
{
    gimbal_pid_base_update_once();

    if (++sys->period.camera_x_pid_cnt >= sys->period.camera_x_pid_cnt_val)
    {
        sys->period.camera_x_pid_cnt = 0;
        parallel_pid_ctrl(&sys->camera_x_pid, ref_value, sys->value.camera_x);
    }

    // 速度模式
    // 张大头电机控制
    // float motor_speed = sys->camera_x_pid.out_value;

    float pos_delta = sys->camera_x_pid.out_value;

    // // 限制偏航角在安全范围内，防止过度旋转导致机械损伤
    // if ((yawmotor.Position > YAW_MAX && motor_speed > 0.0f) ||
    //     (yawmotor.Position < YAW_MIN && motor_speed < 0.0f))
    // {
    //     ZhangDaTou_Speedctr(&yawmotor, 0.0f, 0);
    // }
    // else
    // {
    //     ZhangDaTou_Speedctr(&yawmotor, motor_speed, 1000);
    // }

    // ZhangDaTou_Speedctr(&yawmotor, motor_speed, yawmotor.setAcc);
    float yaw_goal = yaw_pid_base_pos + pos_delta;

    // if (yaw_goal > YAW_MAX)
    //     yaw_goal = YAW_MAX;
    // else if (yaw_goal < YAW_MIN)
    //     yaw_goal = YAW_MIN;

    ZhangDaTou_PositionSpeedctr(&yawmotor, yawmotor.setSpeed, yaw_goal, yawmotor.setAcc);
    ZhangDaTou_Control(&yawmotor);
}

void camera_y_pid_run_ctrl(sys_t *sys, float ref_value)
{
    gimbal_pid_base_update_once();

    if (++sys->period.camera_y_pid_run_cnt >= sys->period.camera_y_pid_cnt_run_val)
    {
        sys->period.camera_y_pid_run_cnt = 0;
        parallel_pid_ctrl(&sys->camera_y_pid_run, ref_value, sys->value.camera_y);
    }

    // 速度模式
    // 张大头电机控制
    // 这个变量名字要改
    //     float motor_speed = sys->camera_y_pid.out_value;

    //    // 限制俯仰角在安全范围内，防止过度旋转导致机械损伤
    //     if ((pitchmotor.Position > PITCH_MAX && motor_speed > 0.0f) ||
    //         (pitchmotor.Position < PITCH_MIN && motor_speed < 0.0f))
    //     {
    //         // ZhangDaTou_Speedctr(&pitchmotor, 0.0f, 0);
    //         ZhangDaTou_PositionSpeedctr(&pitchmotor, 0.0f, 0.0f, 0);
    //     }
    //     else
    //     {
    //         // ZhangDaTou_Speedctr(&pitchmotor, motor_speed, 1000);
    //         ZhangDaTou_PositionSpeedctr(&pitchmotor, pitchmotor.setSpeed, motor_speed, yawmotor.setAcc);
    //     }

    //     ZhangDaTou_Control(&pitchmotor);

    // 张大头电机控制
    float pos_delta = sys->camera_y_pid_run.out_value; // 位置模式PID的输出是角度的修正量，单位是deg
    float pitch_goal = pitch_pid_base_pos + pos_delta;

    // 限制俯仰角在安全范围内，防止过度旋转导致机械损伤
    if (pitch_goal > PITCH_MAX)
        pitch_goal = PITCH_MAX;
    else if (pitch_goal < PITCH_MIN)
        pitch_goal = PITCH_MIN;

    if ((pitchmotor.Position > PITCH_MAX && pitch_goal > pitchmotor.Position) ||
        (pitchmotor.Position < PITCH_MIN && pitch_goal < pitchmotor.Position))
    {
        // ZhangDaTou_Speedctr(&pitchmotor, 0.0f, 0);
        ZhangDaTou_PositionSpeedctr(&pitchmotor, pitchmotor.setSpeed, 0.0f, 0);
    }
    else
    {
        // ZhangDaTou_Speedctr(&pitchmotor, motor_speed, pitchmotor.setAcc);
        ZhangDaTou_PositionSpeedctr(&pitchmotor, pitchmotor.setSpeed, pitch_goal, pitchmotor.setAcc);
    }

    ZhangDaTou_Control(&pitchmotor);
}

void camera_x_pid_run_ctrl(sys_t *sys, float ref_value)
{
    gimbal_pid_base_update_once();

    if (++sys->period.camera_x_pid_run_cnt >= sys->period.camera_x_pid_cnt_run_val)
    {
        sys->period.camera_x_pid_run_cnt = 0;
        parallel_pid_ctrl(&sys->camera_x_pid_run, ref_value, sys->value.camera_x);
    }
    // float err, err_last;
    // err = sys->value.camera_x - test_x;
    // d_term = err - err_last;
    // err_last = err;
    // float pos_delta = d_term * kd + sys->camera_x_pid_run.out_value;
    float pos_delta = sys->camera_x_pid_run.out_value;

    // 速度模式
    // 张大头电机控制
    // float motor_speed = sys->camera_x_pid.out_value;

    // float pos_delta = sys->camera_x_pid_run.out_value;

    // // 限制偏航角在安全范围内，防止过度旋转导致机械损伤
    // if ((yawmotor.Position > YAW_MAX && motor_speed > 0.0f) ||
    //     (yawmotor.Position < YAW_MIN && motor_speed < 0.0f))
    // {
    //     ZhangDaTou_Speedctr(&yawmotor, 0.0f, 0);
    // }
    // else
    // {
    //     ZhangDaTou_Speedctr(&yawmotor, motor_speed, 1000);
    // }

    // ZhangDaTou_Speedctr(&yawmotor, motor_speed, yawmotor.setAcc);
    static float delta = 0;
    // static float pos_filter = 0;
    static float pos_learn = 0;
    delta = car_speak_rx.data_1 * 0.035f + delta * 0.965f; // 小车的滤波
    // pos_filter = pos_delta * 0.8f + pos_filter * 0.2f;      // 摄像头的滤波
//    pos_learn = pos_delta * 0.01f + pos_learn * 0.92f;    // 摄像头的学习率

    float yaw_goal = yaw_pid_base_pos + pos_delta + delta + pos_learn;
    // float yaw_goal = 180  + pos_delta + delta + pos_learn;

    // float speed_goal = pos_delta;

    // if (yaw_goal > YAW_MAX)
    //     yaw_goal = YAW_MAX;
    // else if (yaw_goal < YAW_MIN)
    //     yaw_goal = YAW_MIN;

    ZhangDaTou_PositionSpeedctr(&yawmotor, yawmotor.setSpeed, yaw_goal, yawmotor.setAcc);
    // ZhangDaTou_Speedctr(&yawmotor,speed_goal,yawmotor.setAcc);

    ZhangDaTou_Control(&yawmotor);
}

/* ================= H题：摆杆滚球控制 =================
 * pitchmotor 复用为摆杆电机。
 * ROD_CENTER_DEG / ROD_MIN_DEG / ROD_MAX_DEG 必须实测后填写。
 * 题3使用静态 PID；题4/5/6使用运动 PID，并预留底盘前馈补偿量。
 * 测摆杆机械限幅时，把 ROD_LIMIT_TEST_ENABLE 改成 1U。
 * 手动调 PID 时，打开 rod_cmd_limit_test_enable，反馈位置由视觉更新到 sys.value.ball_pos_cm。
 */
#define ROD_CENTER_DEG          0.0f        // 摆杆物理水平位置对应的 pitchmotor 反馈角度，必须实测填写
#define ROD_MIN_DEG            -10.0f
#define ROD_MAX_DEG             8.0f
#define ROD_DEFAULT_SPEED_DPS  80.0f
#define ROD_DEFAULT_ACC      1000U

/* 摆杆限幅测试开关。
 * 0U：正常运行题3/4/5/6状态机和 PID。
 * 1U：进入测试后会失能 pitchmotor，只读取反馈角度，不执行状态机 PID，不给摆杆下发位置命令。
 * 测试方法：打开后烧录，手动转摆杆到机械最低/最高安全位置，记录 pitch_pos 或 sys.value.rod_angle_deg。
 * 记录值分别填入 ROD_MIN_DEG / ROD_MAX_DEG，水平位置填入 ROD_CENTER_DEG。
 */
#define ROD_LIMIT_TEST_ENABLE   0U

/* 摆杆 PID 限幅调试变量。
 * 这些变量是给 Keil Watch 手动改的，不需要每次重编。
 * rod_cmd_limit_test_enable = 1：进入 PID 限幅测试模式，状态机不跑题3/4/5/6。
 * sys.ctrl.ball_target_cm：手动给 PID 目标位置，单位 cm。
 * sys.value.ball_pos_cm：视觉更新的 PID 反馈位置，单位 cm。
 * sys.camera_x_pid.kp/ki/kd 在 Keil 里直接改，例如 kp 改成 0.01f。
 * 最终命令 = ROD_CENTER_DEG + sys.camera_x_pid.out_value，然后经过 ROD_MIN_DEG/ROD_MAX_DEG 限幅。
 */
volatile uint8_t rod_cmd_limit_test_enable = 1U;
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

    /* PID 限幅测试模式会真正运行 sys.camera_x_pid。
     * 在 Keil Watch 里手动修改：
     * 1. sys.camera_x_pid.kp / ki / kd
     * 2. sys.ctrl.ball_target_cm 作为 PID 目标位置，单位 cm
     * 3. sys.value.ball_pos_cm 作为 PID 反馈位置，单位 cm
     * 然后观察 rod_pid_test_run_cnt、sys.camera_x_pid.error/out_value 和 sys.ctrl.rod_angle_cmd_deg。
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

    parallel_pid_ctrl(&sys.camera_x_pid, sys.ctrl.ball_target_cm, sys.value.ball_pos_cm);

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
    sys.camera_x_pid.kp = 0.0f;
    sys.camera_x_pid.ki = 0.0f;
    sys.camera_x_pid.kd = 0.0f;
    sys.camera_x_pid.out_max = 20000.0f;
    sys.camera_x_pid.out_min = -20000.0f;
    sys.camera_x_pid.i_term_max = 20000.0f;
    sys.camera_x_pid.i_term_min = -20000.0f;

    /* 题4/5/6运动滚球 PID：钢球位置 cm -> 摆杆角度 deg。
     * 小车运动时会叠加底盘前馈 sys.ctrl.rod_chassis_ff_deg。
     * 这套参数和题3分开调，避免静止题和运动题互相影响。
     */
    sys.camera_x_pid_run.kp = 0.0f;
    sys.camera_x_pid_run.ki = 0.0f;
    sys.camera_x_pid_run.kd = 0.0f;
    sys.camera_x_pid_run.out_max = 20000.0f;
    sys.camera_x_pid_run.out_min = -20000.0f;
    sys.camera_x_pid_run.i_term_max = 20000.0f;
    sys.camera_x_pid_run.i_term_min = -20000.0f;

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

    sys_obj->ctrl.ball_target_cm = target_cm;
    parallel_pid_ctrl(&sys_obj->camera_x_pid, target_cm, sys_obj->value.ball_pos_cm);

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
    parallel_pid_ctrl(&sys_obj->camera_x_pid_run, target_cm, sys_obj->value.ball_pos_cm);

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