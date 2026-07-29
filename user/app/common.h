#ifndef __COMMON_H
#define __COMMON_H

#include "main.h"
#include "string.h"
#include <stdbool.h>

#define ANGLE_TO_RAD  M_PI / 180.0f  // 角度转rad
#define RAD_TO_ANGLE   180.0f / M_PI   

/* ========== 数学运算宏 ========== */
#define SIGN(x) (((x) < 0.0f) ? -1.0f : 1.0f) // 返回符号（-1 或 1）
#define NORM2_f(x, y) (sqrtf(SQ(x) + SQ(y)))  // 二维向量二范数

/* ========== 浮点异常处理宏 ========== */
#define UTILS_IS_INF(x) ((x) == (1.0f / 0.0f) || (x) == (-1.0f / 0.0f)) // 判断无穷大
#define UTILS_IS_NAN(x) ((x) != (x))                                    // 判断 NaN
#define UTILS_NAN_ZERO(x) (x = UTILS_IS_NAN(x) ? 0.0f : x)              // NaN 置零

#define MIN_MAX_LIMT(in, low, high)       \
  (in = in > high ? high : in < low ? low \
                                    : in)
#define MAX_LIMT(in, outmax)                              \
  (in = in > outmax ? outmax : in < (-outmax) ? (-outmax) \
                                              : in)

/* ========== 常用数学宏 ========== */
#define SQ(x) ((x) * (x))             // 平方
#define ABS(x) ((x) > 0 ? (x) : -(x)) // 绝对值
// #define MAX(x, y)     (((x) > (y)) ? (x) : (y))
// #define MIN(x, y)     (((x) < (y)) ? (x) : (y))
#define min(x, y) (((x) < (y)) ? (x) : (y))                                // 返回较小值
#define max(x, y) (((x) > (y)) ? (x) : (y))                                // 返回较大值
#define CLAMP(x, lower, upper) (MIN(upper, MAX(x, lower)))                 // 限幅（依赖 MIN/MAX 定义）
#define FLOAT_EQU(floatA, floatB) ((ABS((floatA) - (floatB))) < 0.000001f) // 浮点近似相等比较

/* ========== 角度归一化宏 ========== */
#define wrap_pm_pi(theta)                         \
  theta = (theta > M_PI) ? theta - M_2PI : theta; \
  theta = (theta < -M_PI) ? theta + M_2PI : theta;
#define wrap_0_2pi(theta)                          \
  theta = (theta > M_2PI) ? theta - M_2PI : theta; \
  theta = (theta < 0.0f) ? theta + M_2PI : theta;
#define wrap_pm_2pi(theta)                         \
  theta = (theta > M_2PI) ? theta - M_2PI : theta; \
  theta = (theta < -M_2PI) ? theta + M_2PI : theta;

/* ========== 数学常量 ========== */
#define M_PI (3.14159265359f)             // π
#define M_2PI (6.28318530718f)            // 2π
#define M_2_PI (6.28318530718f)           // 2π（同义宏）
#define div_M_2PI (0.159154943092391467f) // 1/(2π)
#define SQRT3 (1.73205080757f)            // √3
#define SQRT3_BY_2 (0.86602540378f)       // √3/2
#define ONE_BY_SQRT3 (0.57735026919f)     // 1/√3
#define TWO_BY_SQRT3 (1.15470053838f)     // 2/√3

/* ========== 位操作宏 ========== */
#define setbit(x, y) (x |= (1 << y))     // 置位第 y 位
#define clrbit(x, y) (x &= ~(1 << y))    // 清零第 y 位
#define reversebit(x, y) (x ^= (1 << y)) // 翻转第 y 位
#define getbit(x, y) ((x) >> (y) & 1)    // 读取第 y 位

/**
***********************************************************************
* @brief 系统周期控制结构体
* @note 包含启动计数器、测量计数器等多个计数器，以及对应的执行周期和频率参数
***********************************************************************
**/
typedef struct
{
  // 计数器
  uint32_t sys_cnt;
  uint32_t camera_y_pid_cnt;
  uint32_t camera_x_pid_cnt;
  uint32_t camera_y_pid_run_cnt;
  uint32_t camera_x_pid_run_cnt;

  // 周期
  float sys_fs;
  float sys_ts;
  float camera_y_pid_fs;
  float camera_y_pid_ts;
  float camera_x_pid_fs;
  float camera_x_pid_ts;

  float camera_y_pid_run_fs;
  float camera_y_pid_run_ts;
  float camera_x_pid_run_fs;
  float camera_x_pid_run_ts;

  // 计数值
  uint32_t sys_ts_cnt_val;
  uint32_t camera_y_pid_cnt_val;
  uint32_t camera_x_pid_cnt_val;
  uint32_t camera_y_pid_cnt_run_val;
  uint32_t camera_x_pid_cnt_run_val;

} period_t;

/**
***********************************************************************
* @brief PID 控制器参数结构体
* @note
***********************************************************************
**/
typedef struct
{
  volatile float kp; // 比例增益
  volatile float ki; // 积分增益
  volatile float kd; // 微分增益

  volatile float kfp;     // 前馈比例增益
  volatile float kf_damp; // 前馈阻尼系数

  volatile float p_term;           // 比例项输出
  volatile float i_term;           // 积分项输出
  volatile uint8_t i_isolate_flag; // 积分清除隔离标志（1:清零并隔离积分, 0:正常积分）
  volatile float d_term;           // 微分项输出

  volatile float i_term_max; // 积分项上限
  volatile float i_term_min; // 积分项下限

  volatile float ts; // 控制器采样周期

  volatile float ref_value;   // 参考输入值
  volatile float fback_value; // 反馈输入值

  volatile float error;   // 当前误差
  volatile float pre_err; // 上一周期误差

  volatile float out_min; // 输出下限
  volatile float out_max; // 输出上限

  volatile float out_value; // 控制器输出值
} pid_para_t;

typedef struct
{
  float camera_x;
  float camera_y;

  float gimbal_pitch;
  float gimbal_yaw;

} gimbal_value_t;

typedef struct
{
  float pitch_set;
  float yaw_set;
  float camera_x_set;
  float camera_y_set;
} gimbal_ctrl_t;


/**
***********************************************************************
* @brief 小车云台通信结构体
* @note
***********************************************************************
**/
typedef struct
{
  volatile float data_1;
  volatile float data_2;
  volatile float data_3;
  volatile uint8_t flag;
  volatile uint8_t frame_ok;
  volatile uint16_t last_rx_size;
  volatile uint32_t rx_count;
  volatile uint32_t error_count;
  volatile uint8_t raw[16];
} car_speak_rx_t;

extern car_speak_rx_t car_speak_rx;

typedef struct
{
  period_t period;
  gimbal_value_t value;
  gimbal_ctrl_t ctrl;
  pid_para_t camera_y_pid;
  pid_para_t camera_x_pid;
	
	pid_para_t camera_y_pid_run;
	pid_para_t camera_x_pid_run;
	
} sys_t;

extern sys_t sys;


/**
***********************************************************************
* @brief 云台状态机参数结构体
* @note
***********************************************************************
**/
// 云台状态机状态枚举
typedef enum
{
  GIMBAL_IDLE = 0, // 待机，等待按键启动，电机停，激光关  基础1
  // GIMBAL_SEARCH,   
  GIMBAL_SEARCH_LEFT,   // 基础2
  GIMBAL_SEARCH_RIGHT,  // 基础2
  GIMBAL_STATIC_TRACK,  // 基础2  
  GIMBAL_DYNAMIC_TRACK,   // 基础3
  GIMBAL_DYNAMIC_RUNNING,   // 发挥
} gimbal_state;

// 云台状态机变量
typedef struct
{
  gimbal_state state;          // 当前状态
  uint32_t search_timeout_cnt; // 找靶超时计数器
  uint32_t aim_stable_cnt;     // 对准稳定计数器
  // int8_t scan_dir;             // 扫描方向，1和-1
} gimbal_sm_t;

extern gimbal_sm_t gimbal_sm_obj;
extern volatile uint32_t target_lost_cnt;
extern volatile uint8_t aim_stable_frames;

/**
***********************************************************************
* @brief 按键参数结构体
* @note
***********************************************************************
**/
// 按键
typedef enum
{
  KEY_EVENT_NONE = 0, // 没有事件
  KEY_EVENT_SHORT,    // 短按（下降沿触发）
  // KEY_EVENT_LONG,    // 长按，暂时不做，先留位置
} key_event_t;

typedef struct
{
  GPIO_TypeDef *port; // 哪个 GPIO 端口
  uint16_t pin;       // 哪一根 pin
  // uint8_t last_level; // 上一次读到的电平，用来判下降沿

  uint8_t filter_cnt;   // 消抖计数
  uint8_t press_flag;   // 已经确认按下，防止长按重复触发
} key_t;

extern key_t key_menu;  // PA4，用来切菜单项
extern key_t key_enter; // PC3，用来确认/退出


/**
***********************************************************************
* @brief 菜单参数结构体
* @note
***********************************************************************
**/
// 菜单
typedef enum
{
  MENU_ITEM_STANDBY = 0,   // 待机（对应基础1）
  // MENU_ITEM_TRACK_STATIC,  // 静态跟随（对应基础2）
  MENU_ITEM_TRACK_STATIC_LEFT,    // ← 原 MENU_ITEM_TRACK_STATIC 改名/拆分
  MENU_ITEM_TRACK_STATIC_RIGHT,   // ← 新加
  MENU_ITEM_TRACK_DYNAMIC, // 动态跟随（对应基础3）
  MENU_ITEM_RUNNING_DYNAMIC,  // 发挥
  MENU_ITEM_COUNT          // 循环菜单状态
} menu_item_t;

typedef struct
{
  menu_item_t cur_item; // 当前光标停在哪项
  uint8_t in_running;   // 0 = 菜单选择态, 1 = 已进入某功能运行态
} menu_t;


extern menu_t menu;
extern car_speak_rx_t car_speak_rx;


/* ========== 函数声明 ========== */
/*** gimbal_ctrl.c ***/
// uint8_t key_scan(void);
// uint8_t target_found(void);
uint8_t target_stable(void);
// void gimbal_sm(void);
void gimbal_task_state(void);

/*** key.c ***/
void key_init(void);
key_event_t key_update(key_t *k);

/*** menu.c ***/
void menu_init(void);
void menu_update(key_event_t ev_menu, key_event_t ev_enter);

/*** gimbal_calc.c ***/
void camera_data_update(float dx, float dy);

/*** gimbal_drv.c ***/
void gimbal_init(void);

void gimbal_pid_base_update_now(void);

void camera_y_pid_ctrl(sys_t *sys, float ref_value);
void camera_x_pid_ctrl(sys_t *sys, float ref_value);

void camera_y_pid_run_ctrl(sys_t *sys, float ref_value);
void camera_x_pid_run_ctrl(sys_t *sys, float ref_value);

/*** car_speak.c ***/
void CarSpeak_UART_RxStart(void);
void CarSpeak_UART_RecoverReceive(void);
void CarSpeak_UART_ParseData(uint16_t size);

/*** pid_drv.c ***/
float parallel_pid_ctrl(pid_para_t *pid, float ref_value, float fdback_value);

#endif
