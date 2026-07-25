/*
*file sys_time.c
*brief 系统时间模块实现（基于 DWT 微秒计数器）
*note  依赖系统主频宏 SYSCLK_FREQ_HZ，请确保与实际一致
*/

#include "sys_time.h"

// ----- 系统主频（请根据实际修改）-----
#ifndef SYSCLK_FREQ_HZ
    #define SYSCLK_FREQ_HZ  168000000UL   // 默认 168MHz，请修改为实际值
#endif

/*
*brief 初始化 DWT 微秒计数器
*note  在 main 函数开头调用一次即可
*/
void sys_time_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/*
*brief 获取当前微秒时间戳
*retval 微秒数（32位无符号，约 25 秒后溢出，但差值计算不受影响）
*/
uint32_t get_us(void)
{
    return (uint32_t)((uint64_t)DWT->CYCCNT * 1000000UL / SYSCLK_FREQ_HZ);
}

/*
*brief 获取当前毫秒时间戳
*retval 毫秒数（约 49 天后溢出，但差值计算不受影响）
*/
uint32_t get_ms(void)
{
    return get_us() / 1000UL;
}

/*
*brief 阻塞式微秒延时
*param us 延时微秒数
*/
void udelay(uint32_t us)
{
    uint32_t start = get_us();
    while ((get_us() - start) < us) {
        __NOP();
    }
}

/*
*brief 阻塞式毫秒延时
*param ms 延时毫秒数
*/
void mdelay(uint32_t ms)
{
    udelay(ms * 1000UL);
}
