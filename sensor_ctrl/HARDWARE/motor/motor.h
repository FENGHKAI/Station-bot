#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"
#include "encoder.h"   // 包含编码器枚举 Encoder_Index_t

// -------------------- 电机句柄结构体 --------------------
typedef struct {
    TIM_TypeDef *PWM_TIM;           // PWM定时器
    uint32_t PWM_CHANNEL;           // PWM通道 (TIM_Channel_1/2/3/4)
    GPIO_TypeDef *IN1_PORT;         // 方向引脚1
    uint16_t IN1_PIN;
    GPIO_TypeDef *IN2_PORT;         // 方向引脚2
    uint16_t IN2_PIN;
    Encoder_Index_t enc_index;      // 对应的编码器索引 (ENC_LF, ENC_LR, ENC_RF, ENC_RR)
    int16_t current_duty;           // 当前输出占空比 (-100 ~ 100)
} Motor_Handle_t;

// -------------------- PID结构体 --------------------
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float target;          // 目标速度 (m/s)
    float integral;
    float last_error;
    float output;          // 输出占空比 (-100 ~ 100)
    float integral_limit;
    float output_limit;
} PID_Handle_t;

// -------------------- 全局电机实例（外部声明） --------------------
extern Motor_Handle_t motor_LF;
extern Motor_Handle_t motor_RF;
extern Motor_Handle_t motor_LR;
extern Motor_Handle_t motor_RR;

// -------------------- 公共函数声明 --------------------
void Motor_Init(Motor_Handle_t *motor);
void Motor_SetSpeed(Motor_Handle_t *motor, int16_t duty);

void PID_Init(PID_Handle_t *pid);
float PID_Calculate(PID_Handle_t *pid, float target, float feedback);

void MotorControl_Init(void);             // 初始化所有电机和PID
void MotorControl_Update(void);           // 在定时中断中调用，执行速度闭环

#endif