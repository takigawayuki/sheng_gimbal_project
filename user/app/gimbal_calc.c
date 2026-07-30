#include "common.h"
#include "math.h"

// extern volatile uint8_t target_valid;
// extern volatile uint32_t target_lost_cnt;


/* ================= H题：视觉数据换算 =================
 * 这个文件负责把视觉模块发来的钢球位置数据，整理成控制器能直接使用的量。
 * 当前约定：camera_data_update(dx, dy) 里的 dx 表示钢球相对摆杆中心 O 点的位置偏差。
 * 单位约定：默认 dx 已经是 cm；如果视觉发来的是像素，就修改下面的比例系数。
 */

/* 视觉单位到 cm 的比例系数。
 * 如果视觉直接输出 cm，就保持 1.0f。
 * 如果视觉输出像素，比如 10 像素代表 1cm，就填 0.1f。
 */
#define BALL_VISION_SCALE_CM_PER_UNIT  1.0f

/* 钢球位置一阶低通滤波系数。
 * 视觉端已经做 alpha-beta 跟踪和前视预测，串口 position_cm 已经是控制用位置。
 * 这里保持 1.0f，避免下位机再次滤波导致响应慢半拍。
 * 如果后续关闭视觉端滤波/预测，再根据噪声情况改回 0.3f~0.6f。
 */
#define BALL_POS_FILTER_ALPHA          1.0f

/* 更新钢球位置数据。
 * ball_pos_cm 是视觉给出的钢球位置偏差，正负方向需要和摆杆控制方向配合实测确认。
 * 输出结果会写入：
 * sys.value.ball_pos_raw_cm：未滤波原始位置，方便观察视觉噪声。
 * sys.value.ball_pos_cm：控制用位置；当前直接采用视觉端预测后的 position_cm。
 * sys.value.ball_vel_cm_s：由位置差分得到的速度估计，后面需要速度反馈时可以用。
 */
void ball_data_update(float ball_pos_cm)
{
    static uint8_t initialized = 0U;     // 第一次收到视觉数据标志，用于避免滤波初值从 0 慢慢爬过去
    static float last_pos_cm = 0.0f;     // 上一次滤波后的钢球位置，用于计算速度
    float scaled_pos = ball_pos_cm * BALL_VISION_SCALE_CM_PER_UNIT; // 把视觉单位换算成 cm

    /* 保存原始位置，便于调试时对比滤波前后的数据。 */
    sys.value.ball_pos_raw_cm = scaled_pos;

    if (!initialized)
    {
        /* 第一帧数据直接采用视觉值，避免滤波初始值造成很大的假误差。 */
        sys.value.ball_pos_cm = scaled_pos;
        last_pos_cm = scaled_pos;
        initialized = 1U;
    }
    else
    {
        /* 当前 alpha=1.0f：直接使用视觉端预测位置；保留公式方便后续需要时重新启用低通。 */
        sys.value.ball_pos_cm = scaled_pos * BALL_POS_FILTER_ALPHA + sys.value.ball_pos_cm * (1.0f - BALL_POS_FILTER_ALPHA);
    }

    /* 用相邻两次滤波后的位置差分估计速度。
     * sys.period.sys_fs 是系统运行频率，当前一般是 1000Hz。
     */
    sys.value.ball_vel_cm_s = (sys.value.ball_pos_cm - last_pos_cm) * sys.period.sys_fs;
    last_pos_cm = sys.value.ball_pos_cm;

    /* 兼容旧云台代码里使用 camera_x/camera_y 的地方。
     * H题只用一维摆杆位置，所以把钢球位置同步到 camera_x，camera_y 固定为 0。
     */
    sys.value.camera_x = sys.value.ball_pos_cm;
    sys.value.camera_y = 0.0f;

    /* 收到视觉数据后清除丢目标计数。 */
    target_lost_cnt = 0;
}

/* 视觉数据统一入口。
 * 旧云台工程里 dx/dy 表示二维靶心偏差。
 * H题摆杆只关心钢球沿摆杆方向的一维位置，所以这里只使用 dx，dy 暂时不用。
 */
void camera_data_update(float dx, float dy)
{
    ball_data_update(dx);
    (void)dy; // 防止编译器提示未使用参数
}
