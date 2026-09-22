/* *file adc_battery.h
 * *brief 电源电量检测驱动声明（ADC1_IN12，PC2，分压1/4）
 * *note 芯片 STM32F407VET6，3S锂电池（12.5V满电/11.1V额定/9.6V放电截止）
 * 两级阈值：报警（提醒收回）+ 停车（强制保护），阈值改宏即可
 */
#ifndef __ADC_BATTERY_H
#define __ADC_BATTERY_H

#include "sys.h"

// ----- 硬件参数 -----
#define BATTERY_DIVIDER_RATIO  4.0f     // 分压比：芯片只测到电池电压的1/4
#define ADC_REF_VOLTAGE        3.3f     // ADC参考电压（VDDA）
#define ADC_FILTER_SAMPLES     10       // 均值滤波采样次数

// ----- 电池阈值（单位：V，均为实际电池组电压，3S锂电）-----
// 满电12.6V / 标称11.1V / 厂家放电截止9.6V
#define BATTERY_WARN_THRESHOLD 10.2f    // 报警阈值：单体3.4V，剩约10~15%电量，尽快收回
#define BATTERY_STOP_THRESHOLD  9.9f    // 停车保护阈值：单体3.3V，距9.6V底线留缓冲
#define BATTERY_LOW_THRESHOLD  BATTERY_WARN_THRESHOLD   // 兼容旧接口

// ----- 电量等级 -----
typedef enum
{
    BATT_LEVEL_OK = 0,      // >10.2V 正常工作
    BATT_LEVEL_WARN,        // 9.9~10.2V 低电量报警，尽快收回充电
    BATT_LEVEL_STOP         // <9.9V 电量耗尽，强制停车保护电池
} BatteryLevel_t;

typedef void (*LowVoltageCallback_t)(void);

void          ADC_Battery_Init(void);
float         ADC_GetBatteryVoltage(void);
BatteryLevel_t ADC_GetBatteryLevel(void);
void          ADC_RegisterLowVoltageCallback(LowVoltageCallback_t cb);
BatteryLevel_t ADC_CheckLowVoltage(void);

#endif

