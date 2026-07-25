#ifndef __DELAY_H
#define __DELAY_H

#include "sys.h"

// ----- SysTick 中断独立开关（默认开启）-----
#ifndef USE_SYSTICK_INTERRUPT
#define USE_SYSTICK_INTERRUPT  0
#endif

// 全局毫秒计数器（仅在裸机且中断启用时自动累加）
extern volatile uint32_t systick_ms;

// ----- 公共接口 -----
void delay_init(uint32_t sysclk_mhz);          // 初始化，传入系统时钟（MHz）
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);

#endif
