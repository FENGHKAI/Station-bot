#include "motor.h"


// 注意：以下方向引脚（IN1/IN2）需根据实际硬件连接修改
Motor_Handle_t motor_LF = {
    .PWM_TIM = TIM1,
    .PWM_CHANNEL = TIM_Channel_1,      // PE9
    .IN1_PORT = GPIOA,                 // 示例，请替换为实际
    .IN1_PIN = GPIO_Pin_0,
    .IN2_PORT = GPIOA,
    .IN2_PIN = GPIO_Pin_1,
    .enc_index = ENC_LF,
    .current_duty = 0
};

Motor_Handle_t motor_RF = {
    .PWM_TIM = TIM1,
    .PWM_CHANNEL = TIM_Channel_3,      // PE13
    .IN1_PORT = GPIOA,
    .IN1_PIN = GPIO_Pin_2,
    .IN2_PORT = GPIOA,
    .IN2_PIN = GPIO_Pin_3,
    .enc_index = ENC_RF,
    .current_duty = 0
};

Motor_Handle_t motor_LR = {
    .PWM_TIM = TIM12,
    .PWM_CHANNEL = TIM_Channel_1,      // PB14
    .IN1_PORT = GPIOA,
    .IN1_PIN = GPIO_Pin_4,
    .IN2_PORT = GPIOA,
    .IN2_PIN = GPIO_Pin_5,
    .enc_index = ENC_LR,
    .current_duty = 0
};

Motor_Handle_t motor_RR = {
    .PWM_TIM = TIM9,
    .PWM_CHANNEL = TIM_Channel_1,      // PE5
    .IN1_PORT = GPIOA,
    .IN1_PIN = GPIO_Pin_6,
    .IN2_PORT = GPIOA,
    .IN2_PIN = GPIO_Pin_7,
    .enc_index = ENC_RR,
    .current_duty = 0
};

// -------------------- PID实例（静态，仅本文件使用） --------------------
static PID_Handle_t pid_LF, pid_RF, pid_LR, pid_RR;

// -------------------- 电机底层驱动函数 --------------------
void Motor_Init(Motor_Handle_t *motor) {
    // 使能相关定时器和GPIO时钟（假设已在外部或这里处理）
    // 配置方向引脚为推挽输出
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_InitStruct.GPIO_Pin = motor->IN1_PIN;
    GPIO_Init(motor->IN1_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = motor->IN2_PIN;
    GPIO_Init(motor->IN2_PORT, &GPIO_InitStruct);

    // 启动PWM输出（假设定时器已配置好，这里仅设置初始占空比0）
    TIM_OCInitTypeDef oc;
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 0;
    // 根据通道选择初始化
    if (motor->PWM_CHANNEL == TIM_Channel_1) TIM_OC1Init(motor->PWM_TIM, &oc);
    else if (motor->PWM_CHANNEL == TIM_Channel_2) TIM_OC2Init(motor->PWM_TIM, &oc);
    else if (motor->PWM_CHANNEL == TIM_Channel_3) TIM_OC3Init(motor->PWM_TIM, &oc);
    else if (motor->PWM_CHANNEL == TIM_Channel_4) TIM_OC4Init(motor->PWM_TIM, &oc);

    TIM_Cmd(motor->PWM_TIM, ENABLE);

    // 初始停止
    Motor_SetSpeed(motor, 0);
}

void Motor_SetSpeed(Motor_Handle_t *motor, int16_t duty) {
    if (duty > 100) duty = 100;
    if (duty < -100) duty = -100;
    motor->current_duty = duty;

    // 控制方向
    if (duty >= 0) {
        GPIO_SetBits(motor->IN1_PORT, motor->IN1_PIN);
        GPIO_ResetBits(motor->IN2_PORT, motor->IN2_PIN);
    } else {
        GPIO_ResetBits(motor->IN1_PORT, motor->IN1_PIN);
        GPIO_SetBits(motor->IN2_PORT, motor->IN2_PIN);
        duty = -duty;
    }

    // 设置PWM占空比，假设ARR=1000（需与定时器配置一致）
    uint32_t pulse = (uint32_t)((float)duty / 100.0f * 1000.0f);
    if (motor->PWM_CHANNEL == TIM_Channel_1) TIM_SetCompare1(motor->PWM_TIM, pulse);
    else if (motor->PWM_CHANNEL == TIM_Channel_2) TIM_SetCompare2(motor->PWM_TIM, pulse);
    else if (motor->PWM_CHANNEL == TIM_Channel_3) TIM_SetCompare3(motor->PWM_TIM, pulse);
    else if (motor->PWM_CHANNEL == TIM_Channel_4) TIM_SetCompare4(motor->PWM_TIM, pulse);
}

// -------------------- PID算法 --------------------
void PID_Init(PID_Handle_t *pid) {
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

float PID_Calculate(PID_Handle_t *pid, float target, float feedback) {
    pid->target = target;
    float error = target - feedback;
    // 积分限幅
    pid->integral += error;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    // 位置式PID输出
    float output = pid->Kp * error
                 + pid->Ki * pid->integral
                 + pid->Kd * (error - pid->last_error);
    pid->last_error = error;
    // 输出限幅
    if (output > pid->output_limit) output = pid->output_limit;
    else if (output < -pid->output_limit) output = -pid->output_limit;
    pid->output = output;
    return output;
}

// -------------------- 统一初始化和更新函数 --------------------
void MotorControl_Init(void) {
    // 初始化四个电机
    Motor_Init(&motor_LF);
    Motor_Init(&motor_RF);
    Motor_Init(&motor_LR);
    Motor_Init(&motor_RR);

    // 初始化四个PID
    PID_Init(&pid_LF);
    PID_Init(&pid_RF);
    PID_Init(&pid_LR);
    PID_Init(&pid_RR);

    // 设置PID参数（需要根据实际调试调整）
    pid_LF.Kp = 1.0f; pid_LF.Ki = 0.1f; pid_LF.Kd = 0.0f;
    pid_RF.Kp = 1.0f; pid_RF.Ki = 0.1f; pid_RF.Kd = 0.0f;
    pid_LR.Kp = 1.0f; pid_LR.Ki = 0.1f; pid_LR.Kd = 0.0f;
    pid_RR.Kp = 1.0f; pid_RR.Ki = 0.1f; pid_RR.Kd = 0.0f;
}

void MotorControl_Update(void) {
    // 获取当前编码器速度（m/s）
    float speed_LF = get_speed(ENC_LF);
    float speed_RF = get_speed(ENC_RF);
    float speed_LR = get_speed(ENC_LR);
    float speed_RR = get_speed(ENC_RR);

    // 目标速度（示例：统一 0.5 m/s 前进，可根据需要单独修改）
    float target = 0.5f;

    // 计算PID输出并应用到电机
    int16_t out_LF = (int16_t)PID_Calculate(&pid_LF, target, speed_LF);
    int16_t out_RF = (int16_t)PID_Calculate(&pid_RF, target, speed_RF);
    int16_t out_LR = (int16_t)PID_Calculate(&pid_LR, target, speed_LR);
    int16_t out_RR = (int16_t)PID_Calculate(&pid_RR, target, speed_RR);

    Motor_SetSpeed(&motor_LF, out_LF);
    Motor_SetSpeed(&motor_RF, out_RF);
    Motor_SetSpeed(&motor_LR, out_LR);
    Motor_SetSpeed(&motor_RR, out_RR);
}