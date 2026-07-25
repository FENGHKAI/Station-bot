/*
*file ultrasonic.c
*brief HC-SR04 超声波测距驱动实现（阻塞式轮询，TIM7 按需使能）
*note  超时时间由调用者指定，函数内部使用该值检测超时，确保不会无限阻塞
*/

#include "ultrasonic.h"

#define TIM7_CLK_FREQ  84000000

void HC_SR04_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
    TIM_TimeBaseStruct.TIM_Prescaler = (TIM7_CLK_FREQ / 1000000) - 1;
    TIM_TimeBaseStruct.TIM_Period = 0xFFFF;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStruct);
    
    TIM_Cmd(TIM7, DISABLE);
}

/*
*brief 获取一次测距结果（阻塞式，可指定超时）
*param timeout_us 超时时间（微秒），建议至少 60000（60ms）
*retval 距离（单位：厘米），超时或无效返回 0
*/
float HC_SR04_GetDistance(uint32_t timeout_us)
{
    uint32_t time;
    
    GPIO_SetBits(GPIOA, GPIO_Pin_3);
    delay_us(20);
    GPIO_ResetBits(GPIOA, GPIO_Pin_3);
    
    TIM_Cmd(TIM7,ENABLE);
    while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == 0) {
        if (TIM_GetCounter(TIM7) > timeout_us) {
            TIM_Cmd(TIM7, DISABLE);
            return 0;
        }
    }
    TIM_SetCounter(TIM7,0);
    
    while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == 1) {
        if (TIM_GetCounter(TIM7) > timeout_us) {
            TIM_Cmd(TIM7, DISABLE);
            return 0;
        }
    }
    time = TIM_GetCounter(TIM7);
    
    TIM_Cmd(TIM7, DISABLE);
    
    float distance = (float)time * 0.017f;
    if (distance > 400.0f || distance < 2.0f) return 0;
    
    return distance;
}
