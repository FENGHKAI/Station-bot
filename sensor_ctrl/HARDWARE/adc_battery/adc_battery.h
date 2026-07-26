/*
*file adc_battery.h
*brief 电源电量检测驱动声明（ADC1_IN12，PC2，分压1/4）
*note  低压阈值直接修改 BATTERY_LOW_THRESHOLD 宏
*/

#ifndef __ADC_BATTERY_H
#define __ADC_BATTERY_H

#include "sys.h"

#define BATTERY_DIVIDER_RATIO  4.0f      // 分压比
#define ADC_REF_VOLTAGE        3.3f      // ADC 参考电压（VDDA）
#define ADC_FILTER_SAMPLES     10        // 均值滤波采样次数
#define BATTERY_LOW_THRESHOLD  3.0f      // 低压阈值（单位：V，实际电池电压）

typedef void (*LowVoltageCallback_t)(void);

void ADC_Battery_Init(void);
float ADC_GetBatteryVoltage(void);
void ADC_RegisterLowVoltageCallback(LowVoltageCallback_t cb);
void ADC_CheckLowVoltage(void);

#endif
