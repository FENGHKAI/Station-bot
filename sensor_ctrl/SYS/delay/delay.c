#include "delay.h"
#include "sys.h"

// FreeRTOS 支持
#if SYSTEM_SUPPORT_OS
    #include "FreeRTOS.h"
    #include "task.h"
#endif

// ----- 静态变量 -----
static uint32_t fac_us = 0;     // 1us 对应的 SysTick 计数值
static uint32_t fac_ms = 0;     // 1ms 对应的 SysTick 计数值（裸机下）
volatile uint32_t systick_ms = 0;

// ----- 调度锁定/解锁（仅 OS 模式）-----
#if SYSTEM_SUPPORT_OS
    // FreeRTOS 调度锁定/解锁
    static void delay_osschedlock(void)
    {
        vTaskSuspendAll();      // 挂起调度器
    }
    static void delay_osschedunlock(void)
    {
        xTaskResumeAll();       // 恢复调度器
    }
    static void delay_ostimedly(uint32_t ticks)
    {
        vTaskDelay(ticks);
    }
#endif

// ----- SysTick 中断服务（裸机下使用）-----
#if !SYSTEM_SUPPORT_OS
    __weak void SysTick_Handler(void)
    {
        #if USE_SYSTICK_INTERRUPT
            systick_ms++;
        #endif
    }
#endif

// ----- FreeRTOS Tick Hook（用于累加 systick_ms）-----
#if SYSTEM_SUPPORT_OS
    // 若用户未定义自己的 vApplicationTickHook，则使用此弱定义
    __attribute__((weak)) void vApplicationTickHook(void)
    {
        // 需要用户在 FreeRTOSConfig.h 中定义 configUSE_TICK_HOOK = 1
        // 才能调用此钩子函数
        systick_ms++;
    }
#endif

// ----- 初始化函数 -----
void delay_init(uint32_t sysclk_mhz)
{
    fac_us = sysclk_mhz / 8;
    fac_ms = fac_us * 1000;

#if !SYSTEM_SUPPORT_OS
    // 裸机模式下配置 SysTick
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    uint32_t reload = fac_ms;
    if (reload > 0xFFFFFF) reload = 0xFFFFFF;
    SysTick->LOAD = reload;
    SysTick->VAL = 0;
    #if USE_SYSTICK_INTERRUPT
        SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    #else
        SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    #endif
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
#else
    // FreeRTOS 下 SysTick 由 FreeRTOS 管理，此处不做配置
    // 需要确保 FreeRTOS 已正确初始化 SysTick
#endif
}

// ----- 微秒延时 -----
#if SYSTEM_SUPPORT_OS
    // OS 模式：轮询 + 锁定调度
    void delay_us(uint32_t nus)
    {
        uint32_t ticks = nus * fac_us;
        uint32_t reload = SysTick->LOAD;    // FreeRTOS 已经配置好 LOAD
        uint32_t told = SysTick->VAL;
        uint32_t tcnt = 0, tnow;

        delay_osschedlock();    // 挂起调度器，防止任务切换打断延时
        while (1) {
            tnow = SysTick->VAL;
            if (tnow != told) {
                if (tnow < told) tcnt += told - tnow;
                else tcnt += reload - tnow + told;
                told = tnow;
                if (tcnt >= ticks) break;
            }
        }
        delay_osschedunlock();  // 恢复调度器
    }
#else
    // 裸机模式：单次装载轮询
    void delay_us(uint32_t nus)
    {
        uint32_t ticks = nus * fac_us;
        if (ticks > 0xFFFFFF) ticks = 0xFFFFFF;
        SysTick->LOAD = ticks;
        SysTick->VAL = 0;
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        while (!(SysTick->CTRL & (1 << 16)));   // 等待 COUNTFLAG
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        SysTick->VAL = 0;
    }
#endif

// ----- 毫秒延时 -----
void delay_ms(uint16_t nms)
{
#if SYSTEM_SUPPORT_OS
    // OS 模式：如果调度器运行且不在中断中，使用 vTaskDelay
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING &&
        xPortIsInsideInterrupt() == pdFALSE)
    {
        // 计算需要的 tick 数（假设 tick 频率为 configTICK_RATE_HZ）
        uint32_t ticks = (nms * configTICK_RATE_HZ) / 1000;
        if (ticks > 0) {
            delay_ostimedly(ticks);
            return;
        }
    }
    // 否则降级为忙等
    delay_us((uint32_t)nms * 1000);
#else
    // 裸机模式：直接调用微秒延时
    delay_us((uint32_t)nms * 1000);
#endif
}
