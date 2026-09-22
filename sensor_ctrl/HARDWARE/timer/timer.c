/*
*file timer.c
*brief 基本定时器tim6 用于计算速度和电机速度控制
*/
#include "timer.h"
#include "debug_usart.h"

void TIM6_init(void)
{
    TIM_TimeBaseInitTypeDef TIM6_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
    TIM6_TimeBaseStructure.TIM_Prescaler=8400-1;
    TIM6_TimeBaseStructure.TIM_Period=100-1;
    TIM6_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
    TIM6_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM6,&TIM6_TimeBaseStructure);
    TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel=TIM6_DAC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; // 抢占优先级 0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;        // 子优先级 1
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM6, ENABLE);
}

/* ============ 暂停/恢复速度环（转弯专用） ============ */
/* 转弯开始时调用：关TIM6中断，速度环和里程累计同时冻结 */
void Motor_PauseControl(void)
{
    TIM_Cmd(TIM6, DISABLE);
}

/* 转弯结束调用：清编码器CNT残留 → 复位PID → 开TIM6，干净起步 */
void Motor_ResumeControl(void)
{
    /* 关键：丢弃转弯期间编码器积累的计数，防止恢复瞬间速度鬼值 */
    Encoder_LF.TIM->CNT = 0;
    Encoder_LR.TIM->CNT = 0;
    Encoder_RF.TIM->CNT = 0;
    Encoder_RR.TIM->CNT = 0;

    Motor_ResetSpeedPID();   /* ★新增：PID输出/误差清零，从静止干净起步，根治鬼速 */

    /* 清掉暂停期间可能挂起的旧中断标志，避免一使能就先跑一次脏中断 */
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    TIM_Cmd(TIM6, ENABLE);
}

void TIM6_DAC_IRQHandler(void)
{
    // 检查 TIM6 更新中断
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        // 清除中断标志（必须！）
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

        // 更新编码器速度（读取 CNT 并计算）
        Update_Encoder_Speeds();

        // 执行电机速度闭环控制
        MotorControl_Update();
    }
}

