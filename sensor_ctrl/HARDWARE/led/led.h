/*
*file led.h
*brief LED 驱动声明（PB6，低电平亮）
*/

#ifndef __LED_H
#define __LED_H

#include "sys.h"

void LED_Init(void);

#define LED_On()      GPIO_ResetBits(GPIOB, GPIO_Pin_6)
#define LED_Off()     GPIO_SetBits(GPIOB, GPIO_Pin_6)
#define LED_Toggle()  GPIO_ToggleBits(GPIOB, GPIO_Pin_6)

#endif
