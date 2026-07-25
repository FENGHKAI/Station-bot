/*
*file tracking.h
*brief 红外寻迹驱动声明
*note  四路红外从右往左编号 x1~x4，对应 IO：PC4, PC5, PB0, PB1
*       外环方向 PID 输出转向量，内环速度 PID 由 motor 模块提供
*       速度平滑基于 sys_time 模块的微秒时间戳
*/

#ifndef __TRACKING_H
#define __TRACKING_H

#include "sys.h"          // 包含所有官方库
#include "motor.h"        // 使用 PID_Handle_t 和电机控制
#include "sys_time.h"     // 使用 get_us()

// ----- 红外传感器引脚定义（从右往左）-----
#define IR_X1_PIN  GPIO_Pin_4   // PC4
#define IR_X2_PIN  GPIO_Pin_5   // PC5
#define IR_X3_PIN  GPIO_Pin_0   // PB0
#define IR_X4_PIN  GPIO_Pin_1   // PB1

// ----- 红外传感器位置权重（从右往左，偏差范围 -3 ~ +3）-----
#define IR_X1_WEIGHT  3
#define IR_X2_WEIGHT  1
#define IR_X3_WEIGHT -1
#define IR_X4_WEIGHT -3

// ----- 速度平滑参数（m/s²）-----
#define SPEED_ACCEL_LIMIT  0.6f   // 最大加速度，值越小加速越平缓

// ----- 外环方向 PID 参数结构体 -----
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float target;         // 目标偏差（通常为 0）
    float integral;
    float last_error;
    float output;         // 转向控制量（-100 ~ +100）
    float integral_limit;
    float output_limit;
} DirectionPID_Handle_t;

void Tracking_Init(void);
int16_t Tracking_GetDeviation(void);
float DirectionPID_Calculate(DirectionPID_Handle_t *pid, int16_t deviation);
void Tracking_Control(float base_speed);

// 外环方向 PID 实例（外部可访问，方便调试）
extern DirectionPID_Handle_t g_dir_pid;

#endif
