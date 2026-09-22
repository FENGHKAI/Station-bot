/* ============================================================
 * delay.c —— 双模式延时（裸机 / FreeRTOS），无DWT、无SysTick中断
 *
 * 关键修复：原代码OS模式的delay_init不配置SysTick，而SysTick
 * 要到vTaskStartScheduler()才被FreeRTOS启动 → 调度器启动前
 * delay_us轮询VAL永远不动 → 死循环（你卡住的地方）。
 * 现在两种模式都在delay_init里把SysTick配成自由计数器。
 * ============================================================ */

#include "delay.h"

static uint32_t fac_us = 0;     /* HCLK/8时钟下，1us对应的SysTick计数 */

/* ============ OS模式：systick_ms由tick hook累加 ============ */
#if SYSTEM_SUPPORT_OS

volatile uint32_t systick_ms = 0;

__attribute__((weak)) void vApplicationTickHook( void )
{
    systick_ms++;               /* 需configUSE_TICK_HOOK=1（你已配好） */
}

#endif /* SYSTEM_SUPPORT_OS */

/* ============ 初始化 ============ */
void delay_init( uint32_t sysclk_mhz )
{
    fac_us = sysclk_mhz / 8;    /* SysTick时钟=HCLK/8=21MHz → 1us=21计数 */

    /* 两种模式统一配置：自由运行计数器，装载最大值0xFFFFFF，
     * 不开中断（不开TICKINT！）。调度器启动后FreeRTOS会重新装载
     * LOAD（变成1ms一个周期），但计数器照样自由跑，
     * 下面的delay_us轮询逻辑前后都有效。 */
    SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK_Div8 );
    SysTick->LOAD  = 0xFFFFFF;
    SysTick->VAL   = 0;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;    /* 注意：没开TICKINT */
}

/* ============ 微秒延时：SysTick轮询（你原来的方式） ============ */
void delay_us( uint32_t nus )
{
    /* 自动识别SysTick当前实际时钟源：
     * CLKSOURCE=1 → 跑HCLK(168M)；=0 → 跑HCLK/8(21M)。
     * 调度器启动后FreeRTOS重配SysTick，时钟源取决于它用哪个值算装载，
     * 这样写前后两种情况都对，不用去改FreeRTOSConfig.h。 */
    uint32_t fac = ( SysTick->CTRL & SysTick_CTRL_CLKSOURCE_Msk )
                 ? ( SystemCoreClock / 1000000UL )
                 : fac_us;

    uint32_t ticks  = nus * fac;
    uint32_t reload = SysTick->LOAD;    /* 每次现读，兼容调度器前后不同装载值 */
    uint32_t told   = SysTick->VAL;
    uint32_t tcnt = 0, tnow;

    while( 1 )
    {
        tnow = SysTick->VAL;
        if( tnow != told )
        {
            if( tnow < told ) tcnt += told - tnow;
            else              tcnt += reload - tnow + told;
            told = tnow;
            if( tcnt >= ticks ) break;
        }
    }
}

/* ============ 毫秒延时 ============ */
void delay_ms( uint16_t nms )
{
#if SYSTEM_SUPPORT_OS
    /* 调度器运行中且不在中断里 → vTaskDelay让出CPU */
    if( xTaskGetSchedulerState() == taskSCHEDULER_RUNNING &&
        xPortIsInsideInterrupt() == pdFALSE )
    {
        uint32_t ticks = ( ( uint32_t ) nms * configTICK_RATE_HZ ) / 1000UL;
        if( ticks > 0 )
        {
            vTaskDelay( ticks );
            return;
        }
    }
#endif
    /* 裸机 / 调度器未启动 / 中断里 → 忙等 */
    while( nms-- )
    {
        delay_us( 1000 );
    }
}

