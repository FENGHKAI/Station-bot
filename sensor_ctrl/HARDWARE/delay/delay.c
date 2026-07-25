#include "delay.h"
#include "sys.h"

#if SYSTEM_SUPPORT_OS
    #include "includes.h"       // 支持 OS 时，包含 UCOS 头文件
#endif

// 静态变量
static uint32_t fac_us = 0;     // 1us 对应的 SysTick 计数值
static uint32_t fac_ms = 0;     // 1ms 对应的 SysTick 计数值（裸机下）

volatile uint32_t systick_ms = 0;   // 系统毫秒计数器

// ----- OS 相关宏和函数（与原代码一致）-----
#if SYSTEM_SUPPORT_OS
    // 判断 OS 是否运行、节拍频率、中断嵌套等（沿用原代码定义）
    #ifdef OS_CRITICAL_METHOD
        #define delay_osrunning      OSRunning
        #define delay_ostickspersec  OS_TICKS_PER_SEC
        #define delay_osintnesting   OSIntNesting
    #endif
    #ifdef CPU_CFG_CRITICAL_METHOD
        #define delay_osrunning      OSRunning
        #define delay_ostickspersec  OSCfg_TickRate_Hz
        #define delay_osintnesting   OSIntNestingCtr
    #endif

    // 调度锁定/解锁
    void delay_osschedlock(void)
    {
        #ifdef CPU_CFG_CRITICAL_METHOD
            OS_ERR err;
            OSSchedLock(&err);
        #else
            OSSchedLock();
        #endif
    }
    void delay_osschedunlock(void)
    {
        #ifdef CPU_CFG_CRITICAL_METHOD
            OS_ERR err;
            OSSchedUnlock(&err);
        #else
            OSSchedUnlock();
        #endif
    }
    void delay_ostimedly(uint32_t ticks)
    {
        #ifdef CPU_CFG_CRITICAL_METHOD
            OS_ERR err;
            OSTimeDly(ticks, OS_OPT_TIME_PERIODIC, &err);
        #else
            OSTimeDly(ticks);
        #endif
    }
#endif // SYSTEM_SUPPORT_OS

// ----- 弱定义 SysTick 中断服务（裸机下使用）-----
__weak void SysTick_Handler(void)
{
    #if (!SYSTEM_SUPPORT_OS) && (USE_SYSTICK_INTERRUPT)
        systick_ms++;   // 裸机且中断开启时，累加毫秒计数器
    #endif
    // 如果 OS 启用，则由 OS 提供中断服务（会覆盖此弱函数）
}

// ----- 初始化函数 -----
void delay_init(uint32_t sysclk_mhz)
{
    // 配置 SysTick 时钟源为 AHB/8
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    
    fac_us = sysclk_mhz / 8;                // 1us 需要的计数值
    fac_ms = fac_us * 1000;                 // 1ms 需要的计数值（裸机下）

    // 设置重装载值（中断周期 1ms，若开启中断）
    uint32_t reload = fac_ms;               // 1ms 计数值
    if (reload > 0xFFFFFF) reload = 0xFFFFFF;
    SysTick->LOAD = reload;
    SysTick->VAL = 0;
    
    // 中断控制：由宏和函数共同决定
    #if USE_SYSTICK_INTERRUPT
        SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    #else
        SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    #endif
    
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;   // 使能 SysTick
}

// ----- 微秒延时（根据不同模式选择实现）-----
#if SYSTEM_SUPPORT_OS
    // OS 模式下：使用轮询并锁定调度
    void delay_us(uint32_t nus)
    {
        uint32_t ticks = nus * fac_us;
        uint32_t reload = SysTick->LOAD;
        uint32_t told = SysTick->VAL;
        uint32_t tcnt = 0, tnow;
        
        delay_osschedlock();                // 禁止调度
        while (1) {
            tnow = SysTick->VAL;
            if (tnow != told) {
                if (tnow < told) tcnt += told - tnow;
                else tcnt += reload - tnow + told;
                told = tnow;
                if (tcnt >= ticks) break;
            }
        }
        delay_osschedunlock();              // 恢复调度
    }
#else
    // 裸机模式：使用单次装载轮询（不依赖中断）
    void delay_us(uint32_t nus)
    {
        uint32_t ticks = nus * fac_us;
        if (ticks > 0xFFFFFF) ticks = 0xFFFFFF;
        SysTick->LOAD = ticks;
        SysTick->VAL = 0;
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        do {
            // 等待 COUNTFLAG 置位
        } while (!(SysTick->CTRL & (1 << 16)));
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        SysTick->VAL = 0;
    }
#endif

// ----- 毫秒延时（统一调用微秒延时）-----
void delay_ms(uint16_t nms)
{
    #if SYSTEM_SUPPORT_OS
        if (delay_osrunning && delay_osintnesting == 0) {
            // OS 运行且不在中断中，使用 OS 延时
            if (nms >= fac_ms) {
                delay_ostimedly(nms / fac_ms);
            }
            nms %= fac_ms;
        }
        // 剩余部分使用微秒延时
        delay_us((uint32_t)nms * 1000);
    #else
        // 裸机下直接调用微秒延时
        delay_us((uint32_t)nms * 1000);
    #endif
}
