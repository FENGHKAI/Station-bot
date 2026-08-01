#ifndef __DELAY_H
#define __DELAY_H

#include "sys.h"

// ----- SysTick 中断独立开关（裸机下有效）-----
#ifndef USE_SYSTICK_INTERRUPT
#define USE_SYSTICK_INTERRUPT  0
#endif

// 全局毫秒计数器（裸机下由 SysTick 中断累加，FreeRTOS 下由 tick hook 累加）
extern volatile uint32_t systick_ms;

// ----- 公共接口 -----
void delay_init(uint32_t sysclk_mhz);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);

#endif
