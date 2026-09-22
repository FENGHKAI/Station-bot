/* *file  tracking.h
 * *brief 红外寻迹驱动声明
 * note 四路红外从右往左编号 x1~x4，对应 IO：PC4, PC5, PB0, PB1
 * 检测到黑线输出低电平(0)，白地输出高电平(1)（已实测确认）
 * 权重 = 探头物理坐标(mm)，中间间距12.5mm，两侧26mm
 * 偏差单位为 mm，正=黑线偏右（需左转修正）
 * 外环方向PID输出转向量，内环速度PID由motor模块(TIM6中断)提供
 * 速度平滑基于 sys_time 模块的微秒时间戳
 */
#ifndef __TRACKING_H
#define __TRACKING_H

#include "sys.h"        // 包含所有官方库
#include "motor.h"      // 使用 Motor_SetTargetSpeeds
#include "sys_time.h"   // 使用 get_us()

// ----- 红外传感器引脚定义（从右往左）-----
#define IR_X1_PIN  GPIO_Pin_4    // PC4 最右
#define IR_X2_PIN  GPIO_Pin_5    // PC5
#define IR_X3_PIN  GPIO_Pin_0    // PB0
#define IR_X4_PIN  GPIO_Pin_1    // PB1 最左

// ----- 红外传感器位置权重（从右往左，单位 mm）
// 探头以传感器排中心为原点的坐标：-32.25, -6.25, +6.25, +32.25
// 偏差 = 压线探头坐标加权，即黑线中心偏离车体中心多少mm
// 单探头压线：±6.25 或 ±32.25；中间两只同时压线：0（线在正中）
#define IR_X1_WEIGHT  ( 32.25f)   // 最右
#define IR_X2_WEIGHT  (  6.25f)
#define IR_X3_WEIGHT  ( -6.25f)
#define IR_X4_WEIGHT  (-32.25f)   // 最左

// ----- 速度平滑参数（m/s²）-----
#define SPEED_ACCEL_LIMIT  0.6f   // 最大加速度，值越小加速越平缓

// ----- 转向修正参数 -----
#define TRACK_TURN_FACTOR  0.15f   // 低速测试：转向幅度减半

// ----- 偏差低通滤波系数（0~1，越小越平滑）-----
#define DEV_FILTER_ALPHA   0.3f

// ----- 外环方向 PID 参数结构体 -----
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float target;         // 目标偏差（通常为 0）
    float integral;
    float last_error;
    float output;         // 转向控制量（-output_limit ~ +output_limit）
    float integral_limit;
    float output_limit;
} DirectionPID_Handle_t;

void  Tracking_Init(void);
float Tracking_GetDeviation(void);                              // 偏差，单位mm
float DirectionPID_Calculate(DirectionPID_Handle_t *pid, float deviation);
void  Tracking_Control(float base_speed);                       // 核心：每20ms调用一次

// 外环方向 PID 实例（外部可访问，方便调试）
extern DirectionPID_Handle_t g_dir_pid;

#endif

