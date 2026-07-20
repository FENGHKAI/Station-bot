/*
*file motor.c
*brief 电机驱动与PID控制实现
*/

#include "motor.h"

Motor_Handle_t motor_LF = {
    .PWM_TIM = TIM1,
    .PWM_Positive = TIM_Channel_1,
    .PWM_Negative = TIM_Channel_2,
    .GPIO_Port_Pos = GPIOE,
    .GPIO_Pin_Pos = GPIO_Pin_9,
    .GPIO_Port_Neg = GPIOE,
    .GPIO_Pin_Neg = GPIO_Pin_11,
    .GPIO_AF = GPIO_AF_TIM1,
    .enc_index = ENC_LF,
    .current_duty = 0
};  // 左前电机 (LF) : PE9(TIM1_CH1) 正转, PE11(TIM1_CH2) 反转

Motor_Handle_t motor_RF = {
    .PWM_TIM = TIM1,
    .PWM_Positive = TIM_Channel_3,
    .PWM_Negative = TIM_Channel_4,
    .GPIO_Port_Pos = GPIOE,
    .GPIO_Pin_Pos = GPIO_Pin_13,
    .GPIO_Port_Neg = GPIOE,
    .GPIO_Pin_Neg = GPIO_Pin_14,
    .GPIO_AF = GPIO_AF_TIM1,
    .enc_index = ENC_RF,
    .current_duty = 0
};  // 右前电机 (RF) : PE13(TIM1_CH3) 正转, PE14(TIM1_CH4) 反转

Motor_Handle_t motor_LR = {
    .PWM_TIM = TIM12,
    .PWM_Positive = TIM_Channel_1,
    .PWM_Negative = TIM_Channel_2,
    .GPIO_Port_Pos = GPIOB,
    .GPIO_Pin_Pos = GPIO_Pin_14,
    .GPIO_Port_Neg = GPIOB,
    .GPIO_Pin_Neg = GPIO_Pin_15,
    .GPIO_AF = GPIO_AF_TIM12,
    .enc_index = ENC_LR,
    .current_duty = 0
};  // 左后电机 (LR) : PB14(TIM12_CH1) 正转, PB15(TIM12_CH2) 反转

Motor_Handle_t motor_RR = {
    .PWM_TIM = TIM9,
    .PWM_Positive = TIM_Channel_1,
    .PWM_Negative = TIM_Channel_2,
    .GPIO_Port_Pos = GPIOE,
    .GPIO_Pin_Pos = GPIO_Pin_5,
    .GPIO_Port_Neg = GPIOE,
    .GPIO_Pin_Neg = GPIO_Pin_6,
    .GPIO_AF = GPIO_AF_TIM9,
    .enc_index = ENC_RR,
    .current_duty = 0
};  // 右后电机 (RR) : PE5(TIM9_CH1) 正转, PE6(TIM9_CH2) 反转

static PID_Handle_t pid_LF, pid_RF, pid_LR, pid_RR;

/*
*brief 初始化单个电机（GPIO + 定时器PWM）
*note  高级定时器 TIM1 需额外调用 TIM_CtrlPWMOutputs() 使能主输出
*param motor 电机句柄指针
*/
void Motor_Init(Motor_Handle_t *motor)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_OCInitTypeDef TIM_OCStruct;
    uint16_t timer_period = MOTOR_PWM_PERIOD - 1;
    
    // 使能定时器时钟
    if (motor->PWM_TIM == TIM1 || motor->PWM_TIM == TIM9) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    } else if (motor->PWM_TIM == TIM12) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
    }

    // 使能GPIO时钟
    if (motor->GPIO_Port_Pos == GPIOE || motor->GPIO_Port_Neg == GPIOE) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    }
    if (motor->GPIO_Port_Pos == GPIOB || motor->GPIO_Port_Neg == GPIOB) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    }

    // 配置两个GPIO引脚为复用推挽输出
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_InitStruct.GPIO_Pin = motor->GPIO_Pin_Pos;
    GPIO_Init(motor->GPIO_Port_Pos, &GPIO_InitStruct);
    GPIO_PinAFConfig(motor->GPIO_Port_Pos, motor->GPIO_Pin_Pos, motor->GPIO_AF);

    GPIO_InitStruct.GPIO_Pin = motor->GPIO_Pin_Neg;
    GPIO_Init(motor->GPIO_Port_Neg, &GPIO_InitStruct);
    GPIO_PinAFConfig(motor->GPIO_Port_Neg, motor->GPIO_Pin_Neg, motor->GPIO_AF);

    // 配置定时器时基
    TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
    TIM_TimeBaseStruct.TIM_Prescaler = 0;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_Period = timer_period;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(motor->PWM_TIM, &TIM_TimeBaseStruct);

    // 配置两个通道为PWM模式1（有效电平为高），初始占空比0
    TIM_OCStructInit(&TIM_OCStruct);
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_Pulse = 0;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_High;

    if (motor->PWM_Positive == TIM_Channel_1)
        TIM_OC1Init(motor->PWM_TIM, &TIM_OCStruct);
    else if (motor->PWM_Positive == TIM_Channel_2)
        TIM_OC2Init(motor->PWM_TIM, &TIM_OCStruct);
    else if (motor->PWM_Positive == TIM_Channel_3)
        TIM_OC3Init(motor->PWM_TIM, &TIM_OCStruct);
    else if (motor->PWM_Positive == TIM_Channel_4)
        TIM_OC4Init(motor->PWM_TIM, &TIM_OCStruct);

    if (motor->PWM_Negative == TIM_Channel_1)
        TIM_OC1Init(motor->PWM_TIM, &TIM_OCStruct);
    else if (motor->PWM_Negative == TIM_Channel_2)
        TIM_OC2Init(motor->PWM_TIM, &TIM_OCStruct);
    else if (motor->PWM_Negative == TIM_Channel_3)
        TIM_OC3Init(motor->PWM_TIM, &TIM_OCStruct);
    else if (motor->PWM_Negative == TIM_Channel_4)
        TIM_OC4Init(motor->PWM_TIM, &TIM_OCStruct);

    TIM_Cmd(motor->PWM_TIM, ENABLE);
    if (motor->PWM_TIM == TIM1) {
        TIM_CtrlPWMOutputs(TIM1, ENABLE);   // 高级定时器必须使能主输出
    }

    Motor_SetSpeed(motor, 0);
}

/*
*brief 设置电机转速和方向
*param motor 电机句柄指针
*param duty 占空比（-100 ~ 100），正/负值控制正/反转，0为滑行
*note  两路PWM中有一路始终输出0%，另一路输出占空比，避免H桥直通
*/
void Motor_SetSpeed(Motor_Handle_t *motor, int16_t duty)
{
    uint32_t pulse_pos = 0, pulse_neg = 0;
    
    if (duty > 100) duty = 100;
    if (duty < -100) duty = -100;
    motor->current_duty = duty;

    if (duty > 0) {
        pulse_pos = (uint32_t)((float)duty / 100.0f * MOTOR_PWM_PERIOD);
        pulse_neg = 0;
    } else if (duty < 0) {
        pulse_pos = 0;
        pulse_neg = (uint32_t)((float)(-duty) / 100.0f * MOTOR_PWM_PERIOD);
    } else {
        pulse_pos = 0;
        pulse_neg = 0;
    }

    // 更新正转通道比较值
    if (motor->PWM_Positive == TIM_Channel_1)
        TIM_SetCompare1(motor->PWM_TIM, pulse_pos);
    else if (motor->PWM_Positive == TIM_Channel_2)
        TIM_SetCompare2(motor->PWM_TIM, pulse_pos);
    else if (motor->PWM_Positive == TIM_Channel_3)
        TIM_SetCompare3(motor->PWM_TIM, pulse_pos);
    else if (motor->PWM_Positive == TIM_Channel_4)
        TIM_SetCompare4(motor->PWM_TIM, pulse_pos);

    // 更新反转通道比较值
    if (motor->PWM_Negative == TIM_Channel_1)
        TIM_SetCompare1(motor->PWM_TIM, pulse_neg);
    else if (motor->PWM_Negative == TIM_Channel_2)
        TIM_SetCompare2(motor->PWM_TIM, pulse_neg);
    else if (motor->PWM_Negative == TIM_Channel_3)
        TIM_SetCompare3(motor->PWM_TIM, pulse_neg);
    else if (motor->PWM_Negative == TIM_Channel_4)
        TIM_SetCompare4(motor->PWM_TIM, pulse_neg);
}

/*
*brief 初始化PID参数
*param pid PID句柄指针
*/
void PID_Init(PID_Handle_t *pid)
{
    pid->Kp = 0.0f;
    pid->Ki = 0.0f;
    pid->Kd = 0.0f;
    pid->target = 0.0f;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->output = 0.0f;
    pid->integral_limit = 50.0f;
    pid->output_limit = 100.0f;
}

/*
*brief 位置式PID计算
*param pid PID句柄指针
*param target 目标值（单位：m/s）
*param feedback 反馈值（单位：m/s）
*retval PID输出值（-output_limit ~ +output_limit）
*note  积分和输出均做了限幅，防止积分饱和
*/
float PID_Calculate(PID_Handle_t *pid, float target, float feedback)
{
    float error;
    float output;
    
    pid->target = target;
    error = target - feedback;
    
    pid->integral += error;
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;

    output = pid->Kp * error
           + pid->Ki * pid->integral
           + pid->Kd * (error - pid->last_error);
    pid->last_error = error;
    
    if (output > pid->output_limit)
        output = pid->output_limit;
    else if (output < -pid->output_limit)
        output = -pid->output_limit;
    
    pid->output = output;
    return output;
}

/*
*brief 初始化所有电机和PID
*note  PID参数（Kp/Ki/Kd）需根据实际电机调试确定，当前为初始参考值
*/
void MotorControl_Init(void)
{
    Motor_Init(&motor_LF);
    Motor_Init(&motor_RF);
    Motor_Init(&motor_LR);
    Motor_Init(&motor_RR);

    PID_Init(&pid_LF);
    PID_Init(&pid_RF);
    PID_Init(&pid_LR);
    PID_Init(&pid_RR);

    pid_LF.Kp = 1.0f; pid_LF.Ki = 0.1f; pid_LF.Kd = 0.0f;
    pid_RF.Kp = 1.0f; pid_RF.Ki = 0.1f; pid_RF.Kd = 0.0f;
    pid_LR.Kp = 1.0f; pid_LR.Ki = 0.1f; pid_LR.Kd = 0.0f;
    pid_RR.Kp = 1.0f; pid_RR.Ki = 0.1f; pid_RR.Kd = 0.0f;
}

/*
*brief 速度闭环更新函数（由定时器中断调用）
*note  调用频率必须与 SPEED_SAMPLE_PERIOD（100Hz）保持一致
*       目标速度 0.5m/s 为示例，可根据实际需求修改
*/
void MotorControl_Update(void)
{
    float speed_LF, speed_RF, speed_LR, speed_RR;
    float target;
    int16_t out_LF, out_RF, out_LR, out_RR;
    
    speed_LF = get_speed(ENC_LF);
    speed_RF = get_speed(ENC_RF);
    speed_LR = get_speed(ENC_LR);
    speed_RR = get_speed(ENC_RR);

    target = 0.5f;

    out_LF = (int16_t)PID_Calculate(&pid_LF, target, speed_LF);
    out_RF = (int16_t)PID_Calculate(&pid_RF, target, speed_RF);
    out_LR = (int16_t)PID_Calculate(&pid_LR, target, speed_LR);
    out_RR = (int16_t)PID_Calculate(&pid_RR, target, speed_RR);

    Motor_SetSpeed(&motor_LF, out_LF);
    Motor_SetSpeed(&motor_RF, out_RF);
    Motor_SetSpeed(&motor_LR, out_LR);
    Motor_SetSpeed(&motor_RR, out_RR);
}