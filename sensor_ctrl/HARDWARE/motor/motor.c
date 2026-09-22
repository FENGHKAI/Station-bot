/*
*file motor.c
*brief 电机驱动与PID控制实现
*note 驱动层自动补偿电机安装方向（reverse标志）
*     计数频率统一为1MHz（TIM1/TIM9: PSC=167, TIM12: PSC=83）
*     增量式PID，输出范围 -1000 ~ 1000
*     目标速度通过 Motor_SetTargetSpeed() 设置
*/
#include "motor.h"

float g_motor_Kp = 100.0f;
float g_motor_Kd = 200.0f;

/* ===== 里程累计 ===== */
static float odometer = 0.0f; /* 整车里程 = 四轮平均速度积分，单位 m */

// ----- 电机实例（reverse根据实际硬件安装方向设置）-----
Motor_Handle_t motor_LF = {
    .PWM_TIM = TIM1,
    .PWM_Positive = TIM_Channel_1,
    .PWM_Negative = TIM_Channel_2,
    .GPIO_Port_Pos = GPIOE,
    .GPIO_Pin_Pos = GPIO_Pin_9,
    .GPIO_PinSource_Pos = GPIO_PinSource9,
    .GPIO_Port_Neg = GPIOE,
    .GPIO_Pin_Neg = GPIO_Pin_11,
    .GPIO_PinSource_Neg = GPIO_PinSource11,
    .GPIO_AF = GPIO_AF_TIM1,
    .enc_index = ENC_LF,
    .current_duty = 0,
    .reverse = 1 // 如果左轮反转，改为1
}; // 左前电机 (LF) : PE9(TIM1_CH1) , PE11(TIM1_CH2)

Motor_Handle_t motor_RF = {
    .PWM_TIM = TIM1,
    .PWM_Positive = TIM_Channel_3,
    .PWM_Negative = TIM_Channel_4,
    .GPIO_Port_Pos = GPIOE,
    .GPIO_Pin_Pos = GPIO_Pin_13,
    .GPIO_PinSource_Pos = GPIO_PinSource13,
    .GPIO_Port_Neg = GPIOE,
    .GPIO_Pin_Neg = GPIO_Pin_14,
    .GPIO_PinSource_Neg = GPIO_PinSource14,
    .GPIO_AF = GPIO_AF_TIM1,
    .enc_index = ENC_RF,
    .current_duty = 0,
    .reverse = 0
}; // 右前电机 (RF) : PE13(TIM1_CH3) , PE14(TIM1_CH4)

Motor_Handle_t motor_LR = {
    .PWM_TIM = TIM12,
    .PWM_Positive = TIM_Channel_1,
    .PWM_Negative = TIM_Channel_2,
    .GPIO_Port_Pos = GPIOB,
    .GPIO_Pin_Pos = GPIO_Pin_14,
    .GPIO_PinSource_Pos = GPIO_PinSource14,
    .GPIO_Port_Neg = GPIOB,
    .GPIO_Pin_Neg = GPIO_Pin_15,
    .GPIO_PinSource_Neg = GPIO_PinSource15,
    .GPIO_AF = GPIO_AF_TIM12,
    .enc_index = ENC_LR,
    .current_duty = 0,
    .reverse = 1
}; // 左后电机 (LR) : PB14(TIM12_CH1) , PB15(TIM12_CH2)

Motor_Handle_t motor_RR = {
    .PWM_TIM = TIM9,
    .PWM_Positive = TIM_Channel_1,
    .PWM_Negative = TIM_Channel_2,
    .GPIO_Port_Pos = GPIOE,
    .GPIO_Pin_Pos = GPIO_Pin_5,
    .GPIO_PinSource_Pos = GPIO_PinSource5,
    .GPIO_Port_Neg = GPIOE,
    .GPIO_Pin_Neg = GPIO_Pin_6,
    .GPIO_PinSource_Neg = GPIO_PinSource6,
    .GPIO_AF = GPIO_AF_TIM9,
    .enc_index = ENC_RR,
    .current_duty = 0,
    .reverse = 0
}; // 右后电机 (RR) : PE5(TIM9_CH1) , PE6(TIM9_CH2)

// ----- PID实例 -----
PID_Handle_t pid_LF, pid_RF, pid_LR, pid_RR;

/*
*brief 初始化单个电机（GPIO + 定时器PWM）
*note 高级定时器 TIM1/TIM9 需额外调用 TIM_CtrlPWMOutputs()
*     计数频率统一为1MHz（TIM1/TIM9: PSC=167, TIM12: PSC=83）
*param motor 电机句柄指针
*/
void Motor_Init(Motor_Handle_t *motor)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_OCInitTypeDef TIM_OCStruct;
    uint16_t timer_period = MOTOR_PWM_PERIOD - 1;
    uint16_t target_psc = 0;

    // 1. 使能定时器时钟
    if (motor->PWM_TIM == TIM1) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    } else if (motor->PWM_TIM == TIM9) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
    } else if (motor->PWM_TIM == TIM12) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
    }

    // 2. 使能GPIO时钟
    if (motor->GPIO_Port_Pos == GPIOE || motor->GPIO_Port_Neg == GPIOE) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    }
    if (motor->GPIO_Port_Pos == GPIOB || motor->GPIO_Port_Neg == GPIOB) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    }

    // 3. 配置GPIO为复用推挽输出
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_InitStruct.GPIO_Pin = motor->GPIO_Pin_Pos;
    GPIO_Init(motor->GPIO_Port_Pos, &GPIO_InitStruct);
    GPIO_PinAFConfig(motor->GPIO_Port_Pos, motor->GPIO_PinSource_Pos, motor->GPIO_AF);

    GPIO_InitStruct.GPIO_Pin = motor->GPIO_Pin_Neg;
    GPIO_Init(motor->GPIO_Port_Neg, &GPIO_InitStruct);
    GPIO_PinAFConfig(motor->GPIO_Port_Neg, motor->GPIO_PinSource_Neg, motor->GPIO_AF);

    // 4. 配置定时器时基（计数频率统一为1MHz）
    TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_Period = timer_period;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;

    if (motor->PWM_TIM == TIM1 || motor->PWM_TIM == TIM9) {
        target_psc = 167; // 168MHz / 168 = 1MHz
    } else if (motor->PWM_TIM == TIM12) {
        target_psc = 83;  // 84MHz / 84 = 1MHz
    }
    TIM_TimeBaseStruct.TIM_Prescaler = target_psc;
    TIM_TimeBaseInit(motor->PWM_TIM, &TIM_TimeBaseStruct);

    // 5. 配置PWM通道
    TIM_OCStructInit(&TIM_OCStruct);
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_Pulse = 0;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_High;

    // 正转通道
    if (motor->PWM_Positive == TIM_Channel_1)      TIM_OC1Init(motor->PWM_TIM, &TIM_OCStruct);
    else if (motor->PWM_Positive == TIM_Channel_2) TIM_OC2Init(motor->PWM_TIM, &TIM_OCStruct);
    else if (motor->PWM_Positive == TIM_Channel_3) TIM_OC3Init(motor->PWM_TIM, &TIM_OCStruct);
    else if (motor->PWM_Positive == TIM_Channel_4) TIM_OC4Init(motor->PWM_TIM, &TIM_OCStruct);

    // 反转通道
    if (motor->PWM_Negative == TIM_Channel_1)      TIM_OC1Init(motor->PWM_TIM, &TIM_OCStruct);
    else if (motor->PWM_Negative == TIM_Channel_2) TIM_OC2Init(motor->PWM_TIM, &TIM_OCStruct);
    else if (motor->PWM_Negative == TIM_Channel_3) TIM_OC3Init(motor->PWM_TIM, &TIM_OCStruct);
    else if (motor->PWM_Negative == TIM_Channel_4) TIM_OC4Init(motor->PWM_TIM, &TIM_OCStruct);

    // 6. 使能定时器和主输出
    TIM_Cmd(motor->PWM_TIM, ENABLE);
    if (motor->PWM_TIM == TIM1 || motor->PWM_TIM == TIM9) {
        TIM_CtrlPWMOutputs(motor->PWM_TIM, ENABLE);
    }

    // 7. 初始停止
    Motor_SetSpeed(motor, 0);
}

/*
*brief 设置电机转速和方向
*param motor 电机句柄指针
*param duty 占空比（-1000 ~ 1000），正/负值控制正/反转，0为滑行
*note 驱动层自动补偿 reverse 标志（取反）
*     占空比绝对值低于600时可能无法驱动电机旋转（取决于实际电压和负载）
*/
void Motor_SetSpeed(Motor_Handle_t *motor, int16_t duty)
{
    uint32_t pulse_pos = 0, pulse_neg = 0;
    int16_t out_duty = duty;

    // 根据 reverse 标志取反（补偿物理安装方向）
    if (motor->reverse) {
        out_duty = -out_duty;
    }

    // 限幅
    if (out_duty > 1000)  out_duty = 1000;
    if (out_duty < -1000) out_duty = -1000;
    motor->current_duty = out_duty;

    // 映射到PWM比较值（MOTOR_PWM_PERIOD = 1000）
    if (out_duty > 0) {
        pulse_pos = (uint32_t)out_duty;
        pulse_neg = 0;
    } else if (out_duty < 0) {
        pulse_pos = 0;
        pulse_neg = (uint32_t)(-out_duty);
    } else {
        pulse_pos = 0;
        pulse_neg = 0;
    }

    // 更新正转通道比较值
    if (motor->PWM_Positive == TIM_Channel_1)      TIM_SetCompare1(motor->PWM_TIM, pulse_pos);
    else if (motor->PWM_Positive == TIM_Channel_2) TIM_SetCompare2(motor->PWM_TIM, pulse_pos);
    else if (motor->PWM_Positive == TIM_Channel_3) TIM_SetCompare3(motor->PWM_TIM, pulse_pos);
    else if (motor->PWM_Positive == TIM_Channel_4) TIM_SetCompare4(motor->PWM_TIM, pulse_pos);

    // 更新反转通道比较值
    if (motor->PWM_Negative == TIM_Channel_1)      TIM_SetCompare1(motor->PWM_TIM, pulse_neg);
    else if (motor->PWM_Negative == TIM_Channel_2) TIM_SetCompare2(motor->PWM_TIM, pulse_neg);
    else if (motor->PWM_Negative == TIM_Channel_3) TIM_SetCompare3(motor->PWM_TIM, pulse_neg);
    else if (motor->PWM_Negative == TIM_Channel_4) TIM_SetCompare4(motor->PWM_TIM, pulse_neg);
}

/*
*brief 初始化PID参数（增量式）
*param pid PID句柄指针
*/
void PID_Init(PID_Handle_t *pid)
{
    pid->Kp = 0.0f;
    pid->Ki = 0.0f;
    pid->Kd = 0.0f;
    pid->target = 0.0f;
    pid->last_error = 0.0f;
    pid->output = 0.0f;
    pid->output_limit = 1000.0f;
}

/*
*brief 增量式PID计算
*param pid PID句柄指针
*param target 目标值（m/s）
*param feedback 反馈值（m/s）
*retval PID输出值（-output_limit ~ +output_limit）
*note 增量式输出累加，Ki一般设为0
*     根据学长代码：out += Kp*bias + Kd*(bias-bias_last)
*/
float PID_Calculate(PID_Handle_t *pid, float target, float feedback)
{
    float error = target - feedback;
    float delta_output;

    pid->target = target;

    /* ★修改：删除了被覆盖的死代码行
       delta_output = pid->Kp * (error - pid->last_error)
                    + pid->Kd * (error - 2 * pid->last_error + 0);
       生效的只有下面这一行 */
    delta_output = pid->Kp * error + pid->Kd * (error - pid->last_error);

    pid->last_error = error;
    pid->output += delta_output;

    // 输出限幅
    if (pid->output > pid->output_limit)       pid->output = pid->output_limit;
    else if (pid->output < -pid->output_limit) pid->output = -pid->output_limit;

    return pid->output;
}

/*
*brief 设置所有轮子的目标速度（统一）
*param speed 目标速度（m/s）
*note 四个轮子速度相同，用于直线行驶
*/
void Motor_SetTargetSpeed(float speed)
{
    pid_LF.target = speed;
    pid_RF.target = speed;
    pid_LR.target = speed;
    pid_RR.target = speed;
}

/*
*brief 分别设置四个轮子的目标速度
*param lf, rf, lr, rr 四个轮子的目标速度（m/s）
*note 用于差速转向
*/
void Motor_SetTargetSpeeds(float lf, float rf, float lr, float rr)
{
    pid_LF.target = lf;
    pid_RF.target = rf;
    pid_LR.target = lr;
    pid_RR.target = rr;
}

/*
*brief 初始化所有电机和PID
*note PID参数需根据实际调试确定
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

    pid_LF.Kp = g_motor_Kp; pid_LF.Ki = 0.0f; pid_LF.Kd = g_motor_Kd;
    pid_RF.Kp = g_motor_Kp; pid_RF.Ki = 0.0f; pid_RF.Kd = g_motor_Kd;
    pid_LR.Kp = g_motor_Kp; pid_LR.Ki = 0.0f; pid_LR.Kd = g_motor_Kd;
    pid_RR.Kp = g_motor_Kp; pid_RR.Ki = 0.0f; pid_RR.Kd = g_motor_Kd;
}

/*
*brief 速度闭环更新和行走距离采集函数（由定时器中断调用）
*note 调用频率必须与 SPEED_SAMPLE_PERIOD（100Hz）保持一致
*     目标速度通过 Motor_SetTargetSpeed() 提前设置
*     编码器反馈已在 get_speed 中统一方向
*     ★修改：速度环的冻结/恢复由 timer.c 关/开 TIM6 实现，
*       本函数被调用时必然处于闭环使能状态，直接写 PWM
*/
void MotorControl_Update(void)
{
    float speed_LF, speed_RF, speed_LR, speed_RR;
    int16_t out_LF, out_RF, out_LR, out_RR;
    float avg_speed;

    speed_LF = get_speed(ENC_LF);
    speed_RF = get_speed(ENC_RF);
    speed_LR = get_speed(ENC_LR);
    speed_RR = get_speed(ENC_RR);

    out_LF = (int16_t)PID_Calculate(&pid_LF, pid_LF.target, speed_LF);
    out_RF = (int16_t)PID_Calculate(&pid_RF, pid_RF.target, speed_RF);
    out_LR = (int16_t)PID_Calculate(&pid_LR, pid_LR.target, speed_LR);
    out_RR = (int16_t)PID_Calculate(&pid_RR, pid_RR.target, speed_RR);

    Motor_SetSpeed(&motor_LF, out_LF);
    Motor_SetSpeed(&motor_RF, out_RF);
    Motor_SetSpeed(&motor_LR, out_LR);
    Motor_SetSpeed(&motor_RR, out_RR);

    avg_speed = (speed_LF + speed_RF + speed_LR + speed_RR) * 0.25f;
    odometer += avg_speed * 0.01f; /* 里程积分保持 */
}

//行走距离对外接口函数 获取和置零
float Motor_GetOdometer(void)
{
    return odometer;
}

void Motor_ResetOdometer(void)
{
    odometer = 0.0f;
}

/**
* @brief 速度环PID状态复位（由 timer.c 的 Motor_ResumeControl 在解冻前调用）
* @note  本结构无积分项，只需清 last_error 与 output。
*        转弯暂停期间 TIM6 停止、编码器 CNT 已被清零，若不在此复位，
*        恢复瞬间 PID 会带着旧 output 和突变误差输出，导致"鬼速"窜车。
* ★新增函数
*/
void Motor_ResetSpeedPID(void)
{
    pid_LF.last_error = 0.0f;  pid_LF.output = 0.0f;
    pid_RF.last_error = 0.0f;  pid_RF.output = 0.0f;
    pid_LR.last_error = 0.0f;  pid_LR.output = 0.0f;
    pid_RR.last_error = 0.0f;  pid_RR.output = 0.0f;
}

/**
* @brief 开环 PWM 直控（四轮独立），导航旋转 / 急停专用
* @param lf/rf/lr/rr: 各轮占空比 [-1000, +1000]，负值反转，0 滑行
* @note 必须在 Motor_PauseControl()（实现在 timer.c，关 TIM6）之后调用，
*       否则写入值会被 TIM6 中断的速度环输出覆盖。
*       复用 Motor_SetSpeed()：自动完成 reverse 方向补偿、
*       ±1000 限幅、正/反转双通道分发，与闭环路径行为一致。
*/
void Motor_SetPWM_Raw(int16_t lf, int16_t rf, int16_t lr, int16_t rr)
{
    Motor_SetSpeed(&motor_LF, lf);
    Motor_SetSpeed(&motor_RF, rf);
    Motor_SetSpeed(&motor_LR, lr);
    Motor_SetSpeed(&motor_RR, rr);
}

/**
* @brief 全轮停止：速度环目标清零 + 开环输出清零
* @note 不解除冻结状态，恢复行驶须调用 Motor_ResumeControl()。
*/
void Motor_AllStop(void)
{
    Motor_SetTargetSpeeds(0.0f, 0.0f, 0.0f, 0.0f); /* 速度环目标清零 */
    Motor_SetPWM_Raw(0, 0, 0, 0);                  /* 开环兜底清零 */
}
