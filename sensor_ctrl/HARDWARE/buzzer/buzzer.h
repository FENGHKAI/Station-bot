/*
*file buzzer.h
*brief 蜂鸣器驱动声明（PC13，高电平响）
*/

#ifndef __BUZZER_H
#define __BUZZER_H

#include "sys.h"

void Buzzer_Init(void);

#define Buzzer_On()     GPIO_SetBits(GPIOC, GPIO_Pin_13)
#define Buzzer_Off()    GPIO_ResetBits(GPIOC, GPIO_Pin_13)
#define Buzzer_Toggle() GPIO_ToggleBits(GPIOC, GPIO_Pin_13)

#endif
