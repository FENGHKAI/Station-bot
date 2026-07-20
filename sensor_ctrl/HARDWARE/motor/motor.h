/*
*file motor.h
*brief 电机驱动与PID控制声明
*/

#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"
#include "encoder.h"

#define MOTOR_PWM_PERIOD 1000   // PWM周期（自动重装载值）

/**
  * @brief 电机硬件句柄
  * @note  PWM_Positive 和 PWM_Negative 取值：TIM_Channel_1 ~ TIM_Channel_4
  *        GPIO_AF 取值：GPIO_AF_TIM1 / GPIO_AF_TIM9 / GPIO_AF_TIM12
  */
typedef struct {
    TIM_TypeDef *PWM_TIM;           // PWM定时器
    uint32_t PWM_Positive;          // 正转PWM通道
    uint32_t PWM_Negative;          // 反转PWM通道
    GPIO_TypeDef *GPIO_Port_Pos;    // 正转引脚端口
    uint16_t GPIO_Pin_Pos;          // 正转引脚号
    GPIO_TypeDef *GPIO_Port_Neg;    // 反转引脚端口
    uint16_t GPIO_Pin_Neg;          // 反转引脚号
    uint8_t GPIO_AF;                // 复用功能编号
    Encoder_Index_t enc_index;      // 对应编码器索引（取值见 Encoder_Index_t）
    int16_t current_duty;           // 当前占空比（-100 ~ 100）
} Motor_Handle_t;

/**
  * @brief PID控制器句柄
  * @note  output_limit 默认 100，对应占空比上限
  */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float target;          // 目标速度 (m/s)
    float integral;
    float last_error;
    float output;
    float integral_limit;
    float output_limit;
} PID_Handle_t;

extern Motor_Handle_t motor_LF;
extern Motor_Handle_t motor_RF;
extern Motor_Handle_t motor_LR;
extern Motor_Handle_t motor_RR;

void Motor_Init(Motor_Handle_t *motor);
void Motor_SetSpeed(Motor_Handle_t *motor, int16_t duty);
void PID_Init(PID_Handle_t *pid);
float PID_Calculate(PID_Handle_t *pid, float target, float feedback);
void MotorControl_Init(void);
void MotorControl_Update(void);

#endif
