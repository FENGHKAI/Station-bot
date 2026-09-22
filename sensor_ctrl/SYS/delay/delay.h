#ifndef __DELAY_H
#define __DELAY_H

#include "sys.h"

#if SYSTEM_SUPPORT_OS
    #include "FreeRTOS.h"
    #include "task.h"
#endif

/* ---- 公共接口 ---- */
void delay_init(uint32_t sysclk_mhz);   /* 调度器启动前调用一次 */
void delay_us(uint32_t nus);            /* SysTick轮询（你原来的方式） */
void delay_ms(uint16_t nms);            /* OS下调度器运行中自动转vTaskDelay */

#if SYSTEM_SUPPORT_OS
extern volatile uint32_t systick_ms;    /* 由FreeRTOS tick hook累加，1ms一次 */
#endif

#endif

