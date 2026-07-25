/*
*file tracking.c
*brief 红外寻迹驱动实现
*note  四路红外从右往左编号 x1~x4，方向 PID 输出转向量
*       速度平滑基于 sys_time 模块的 get_us() 微秒时间戳
*/

#include "tracking.h"

// 外环方向 PID 实例
DirectionPID_Handle_t g_dir_pid;

// 速度平滑状态（静态，仅本文件使用）
static float current_smooth_speed = 0.0f;
static uint32_t last_time_us = 0;


/*
*brief 初始化四路红外 GPIO 和方向 PID
*note  PC4, PC5 在 GPIOC；PB0, PB1 在 GPIOB
*      红外模块假设黑线输出低电平(0)，白线输出高电平(1)
*      sys_time_init() 需在 main 中提前调用
*/
void Tracking_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // 使能 GPIO 时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
    
    // 配置 PB0, PB1 (x3, x4) 为浮空输入
    GPIO_InitStruct.GPIO_Pin = IR_X3_PIN | IR_X4_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 配置 PC4, PC5 (x1, x2) 为浮空输入
    GPIO_InitStruct.GPIO_Pin = IR_X1_PIN | IR_X2_PIN;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    // 初始化方向 PID 参数
    g_dir_pid.Kp = 0.8f;
    g_dir_pid.Ki = 0.05f;
    g_dir_pid.Kd = 0.1f;
    g_dir_pid.target = 0.0f;
    g_dir_pid.integral = 0.0f;
    g_dir_pid.last_error = 0.0f;
    g_dir_pid.output = 0.0f;
    g_dir_pid.integral_limit = 30.0f;
    g_dir_pid.output_limit = 100.0f;
    
    // 重置速度平滑状态
    current_smooth_speed = 0.0f;
    last_time_us = get_us();
}


/*
*brief 获取当前偏差值
*retval 偏差范围 -3 ~ +3，正偏差表示黑线偏右（需左转修正）
*note   从右往左：x1(右) 权重 +3，x4(左) 权重 -3
*       使用 ! 是因为假设红外模块检测到黑线输出 0
*       如果模块相反（黑线为1），去掉 ! 即可
*/
int16_t Tracking_GetDeviation(void)
{
    uint8_t ir1, ir2, ir3, ir4;
    int16_t deviation = 0;
    
    ir1 = GPIO_ReadInputDataBit(GPIOC, IR_X1_PIN);
    ir2 = GPIO_ReadInputDataBit(GPIOC, IR_X2_PIN);
    ir3 = GPIO_ReadInputDataBit(GPIOB, IR_X3_PIN);
    ir4 = GPIO_ReadInputDataBit(GPIOB, IR_X4_PIN);
    
    deviation = (IR_X1_WEIGHT * (!ir1)) +
                (IR_X2_WEIGHT * (!ir2)) +
                (IR_X3_WEIGHT * (!ir3)) +
                (IR_X4_WEIGHT * (!ir4));
    
    return deviation;
}


/*
*brief 方向 PID 计算
*param pid PID 句柄指针
*param deviation 当前偏差（-3 ~ +3）
*retval 转向控制量（-100 ~ +100），正值为左转，负值为右转
*note  位置式 PID，含积分限幅和输出限幅
*/
float DirectionPID_Calculate(DirectionPID_Handle_t *pid, int16_t deviation)
{
    float error;
    
    error = (float)deviation - pid->target;
    
    // 积分限幅，防止积分饱和
    pid->integral += error;
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;
    
    // 位置式 PID
    pid->output = pid->Kp * error +
                  pid->Ki * pid->integral +
                  pid->Kd * (error - pid->last_error);
    pid->last_error = error;
    
    // 输出限幅
    if (pid->output > pid->output_limit)
        pid->output = pid->output_limit;
    else if (pid->output < -pid->output_limit)
        pid->output = -pid->output_limit;
    
    return pid->output;
}


/*
*brief 速度斜坡生成器（内部调用，基于 get_us 微秒时间戳）
*param target_speed 期望的目标速度（m/s）
*retval 经加速度限制后的实际输出速度（m/s）
*note   加速度由 SPEED_ACCEL_LIMIT 宏控制，单位 m/s²
*       基于真实时间差计算步长，控制周期波动不影响加速度
*/
static float apply_speed_ramp(float target_speed)
{
    uint32_t now_us;
    float dt, speed_diff, max_step;
    
    now_us = get_us();
    
    // 计算两次调用的时间差
    if (last_time_us == 0) {
        dt = 0.001f;
    } else {
        dt = (float)(now_us - last_time_us) / 1000000.0f;
        if (dt > 0.1f) dt = 0.1f;   // 防溢出
    }
    last_time_us = now_us;
    
    // 加速度限制，防止速度突变
    speed_diff = target_speed - current_smooth_speed;
    max_step = SPEED_ACCEL_LIMIT * dt;
    
    if (speed_diff > max_step)
        speed_diff = max_step;
    else if (speed_diff < -max_step)
        speed_diff = -max_step;
    
    current_smooth_speed += speed_diff;
    
    // 防止浮点误差导致的微小偏移
    if (fabsf(target_speed - current_smooth_speed) < 0.001f)
        current_smooth_speed = target_speed;
    
    return current_smooth_speed;
}


/*
*brief 红外寻迹核心控制函数（外环方向 + 内环速度）
*param base_speed 期望的基础速度（m/s），外部可直接传入期望值
*note  调用频率建议 50~100Hz，与速度环一致
*       实际加速度由 SPEED_ACCEL_LIMIT 宏控制（默认 0.6 m/s²）
*       转向修正量最大 ±0.3m/s（通过 turn_factor 调节）
*/
void Tracking_Control(float base_speed)
{
    int16_t deviation;
    float turn, speed_offset, smooth_base;
    float turn_factor, speed_LF, speed_RF, speed_LR, speed_RR;
    
    turn_factor = 0.3f;
    
    // 外环方向 PID
    deviation = Tracking_GetDeviation();
    turn = DirectionPID_Calculate(&g_dir_pid, deviation);
    
    // 转向量映射到左右轮速度修正
    speed_offset = (turn / 100.0f) * turn_factor;
    
    // 内环速度基值平滑处理
    smooth_base = apply_speed_ramp(base_speed);
    
    // 计算四轮最终目标速度
    speed_LF = smooth_base + speed_offset;
    speed_RF = smooth_base - speed_offset;
    speed_LR = smooth_base + speed_offset;
    speed_RR = smooth_base - speed_offset;
    
    // 设置目标速度（内环速度 PID 由 TIM6 中断自动执行）
    pid_LF.target = speed_LF;
    pid_RF.target = speed_RF;
    pid_LR.target = speed_LR;
    pid_RR.target = speed_RR;
}
