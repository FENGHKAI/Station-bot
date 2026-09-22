/* ============================================================
 * sys_time.c —— DWT时间戳模块（裸机/FreeRTOS统一实现）
 *
 * 特点：不依赖OS、不依赖调度器状态、不依赖中断——
 *       DWT是只读硬件计数器，任何时刻读都有效。
 *
 * 回绕说明：32位CYCCNT @168MHz ≈ 25.57s回绕。
 *       只要用差值法 (now - start) 测间隔，完全不受影响；
 *       绝对时间戳需要长时基时，OS下用delay.c的systick_ms。
 * ============================================================ */

#include "sys_time.h"

void sys_time_init( void )
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t get_us( void )
{
    return DWT->CYCCNT / ( SystemCoreClock / 1000000UL );
}

uint32_t get_ms( void )
{
    return get_us() / 1000UL;
}

