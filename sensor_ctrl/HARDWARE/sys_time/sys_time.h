/*
*file sys_time.h
*brief 系统时间模块声明（基于 DWT 微秒计数器）
*note  提供微秒级时间戳，函数名不与 delay 模块冲突
*/

#ifndef __SYS_TIME_H
#define __SYS_TIME_H

#include "sys.h"


void sys_time_init(void);          // 初始化 DWT 计数器（在 main 中调用一次）
uint32_t get_us(void);             // 获取当前微秒时间戳
uint32_t get_ms(void);             // 获取当前毫秒时间戳
void udelay(uint32_t us);          // 阻塞式微秒延时（不占用定时器）
void mdelay(uint32_t ms);          // 阻塞式毫秒延时（调用 udelay 实现）

#endif
