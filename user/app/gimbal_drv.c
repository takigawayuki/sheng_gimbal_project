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

// void pid()
// {
//     error = sys->value.camera_x - test_x;

// 	p_term = kp * error;

// }
