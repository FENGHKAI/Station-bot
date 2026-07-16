#include "timer.h"

static void TIM6_init(void)
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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;   // 抢占优先级 0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;   // 子优先级 1

    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM6, ENABLE);
}

void TIM6_DAC_IRQHandler(void)
{
    int16_t cnt;
    float delta_pulses;
    // 检查 TIM6 更新中断
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

        cnt = getCounter(Encoder_LF);          // 自动清零
        delta_pulses = (float)cnt;
        Encoder_Speed_t[ENC_LF]=(float)delta_pulses*WHEEL_SCALE;

        cnt = getCounter(Encoder_LR);
        delta_pulses = (float)cnt;
        Encoder_Speed_t[ENC_LR]=(float)delta_pulses*WHEEL_SCALE;

        cnt = getCounter(Encoder_RF);
        delta_pulses = (float)cnt;
        Encoder_Speed_t[ENC_RF]=(float)delta_pulses*WHEEL_SCALE;
        
        cnt = getCounter(Encoder_RR);
        delta_pulses = (float)cnt;
        Encoder_Speed_t[ENC_RR]=(float)delta_pulses*WHEEL_SCALE;
        
    }
}