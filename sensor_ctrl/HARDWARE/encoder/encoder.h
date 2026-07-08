/*
*file encoder.h
*brief 编码器驱动声明
*/

#ifndef ENCODER_H
#define ENCODER_H

#include "sys.h"

#define ENCODER_TIM_PERIOD (uint16_t)(65535) //定时器周期，统一采用为16位

typedef struct 
{
    uint32_t RCC_APB1Periph_TIM;
    uint32_t RCC_AHB1Periph_GPIO;
    TIM_TypeDef * TIM;
    GPIO_TypeDef * GPIO_CH1;
    uint16_t GPIO_CH1_Pin;
    uint8_t GPIO_CH1_PinSouce;
    GPIO_TypeDef * GPIO_CH2;
    uint16_t GPIO_CH2_Pin;
    uint8_t GPIO_CH2_PinSouce;
    uint8_t GPIO_AF;
}Encoder_Handle_t;//自定义编码器数据结构体

void encoder_one_init(Encoder_Handle_t Encoder);
void encoder_init(void);

#endif