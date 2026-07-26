/*
*file key.h
*brief 按键驱动声明（PD4，外部中断，内部上拉）
*/

#ifndef __KEY_H
#define __KEY_H

#include "sys.h"
#include "delay.h"

typedef void (*KeyCallback_t)(void);

void Key_Init(void);
void Key_SetCallback(KeyCallback_t cb);

#define Key_IsPressed()  (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4) == 0)

#endif
