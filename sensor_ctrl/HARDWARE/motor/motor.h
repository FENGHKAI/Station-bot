/*
*file motor.h
*brief 电机驱动与PID控制声明
*note 四轮独立驱动，速度闭环控制
*     底层驱动自动补偿电机安装方向（reverse标志）
*     目标速度通过 Motor_SetTargetSpeed() 设置
*     PID采用增量式算法，输出范围 -1000 ~ 1000
*/
#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"
#include "encoder.h"

#define MOTOR_PWM_PERIOD 1000 // PWM周期（自动重装载值），ARR = PERIOD - 1

// ----- 全局PID参数（可在线调试）-----
extern float g_motor_Kp;
extern float g_motor_Kd;

// ----- 电机句柄结构体 -----
typedef struct {
    TIM_TypeDef *PWM_TIM;        // PWM定时器
    uint32_t PWM_Positive;       // 正转PWM通道
    uint32_t PWM_Negative;       // 反转PWM通道
    GPIO_TypeDef *GPIO_Port_Pos; // 正转引脚端口
    uint16_t GPIO_Pin_Pos;       // 正转引脚号（位掩码）
    uint8_t GPIO_PinSource_Pos;  // 正转引脚源编号（0~15）
    GPIO_TypeDef *GPIO_Port_Neg; // 反转引脚端口
    uint16_t GPIO_Pin_Neg;       // 反转引脚号（位掩码）
    uint8_t GPIO_PinSource_Neg;  // 反转引脚源编号（0~15）
    uint8_t GPIO_AF;             // 复用功能编号
    Encoder_Index_t enc_index;   // 对应编码器索引
    int16_t current_duty;        // 当前占空比（-1000 ~ 1000）
    uint8_t reverse;             // 1=反向驱动（占空比取反），0=正常
} Motor_Handle_t;

// ----- PID结构体（增量式）-----
typedef struct {
    float Kp;
    float Ki;            // 一般设为0
    float Kd;
    float target;        // 目标速度（m/s）
    float last_error;    // 上一次误差
    float output;        // 输出占空比（-1000 ~ 1000）
    float output_limit;  // 输出限幅（建议1000）
} PID_Handle_t;

// ----- 全局电机句柄 -----
extern Motor_Handle_t motor_LF;
extern Motor_Handle_t motor_RF;
extern Motor_Handle_t motor_LR;
extern Motor_Handle_t motor_RR;

// ----- 公共接口 -----
// 电机驱动
void Motor_Init(Motor_Handle_t *motor);
void Motor_SetSpeed(Motor_Handle_t *motor, int16_t duty);

// PID
void PID_Init(PID_Handle_t *pid);
float PID_Calculate(PID_Handle_t *pid, float target, float feedback);

// 速度控制
void MotorControl_Init(void);
void MotorControl_Update(void);

// ----- 目标速度设置接口 -----
void Motor_SetTargetSpeed(float speed);                                // 统一设置
void Motor_SetTargetSpeeds(float lf, float rf, float lr, float rr);   // 分别设置

/* ----- 开环直控与速度环冻结（导航旋转/急停用） -----
 * Pause/Resume 的唯一实现在 timer.c（关/开 TIM6 中断），
 * 此处仅声明供 app 层调用。 */
void Motor_PauseControl(void);   /* 冻结速度环（TIM6停） */
void Motor_ResumeControl(void);  /* 解除冻结，恢复闭环 */
void Motor_ResetSpeedPID(void);  /* ★新增：PID状态复位，Resume 内部调用 */
void Motor_SetPWM_Raw(int16_t lf, int16_t rf, int16_t lr, int16_t rr);
void Motor_AllStop(void);        /* 目标清零 + 开环清零（不解冻） */

#endif

