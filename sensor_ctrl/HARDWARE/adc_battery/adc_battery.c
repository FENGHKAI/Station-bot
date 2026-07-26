/*
*file adc_battery.c
*brief 电源电量检测驱动实现（ADC1_IN12，PC2，均值滤波）
*note  低压阈值检测，回调函数供用户注册响应动作
*/

#include "adc_battery.h"
#include <stddef.h>

static LowVoltageCallback_t low_voltage_callback = NULL;

// 单次 ADC 采样
static uint16_t ADC_ReadSingle(void)
{
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    return ADC_GetConversionValue(ADC1);
}

/*
*brief 初始化 ADC1_IN12（PC2）
*note  使用 ADC1，通道 12，扫描模式关闭，单次转换
*/
void ADC_Battery_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    ADC_InitTypeDef ADC_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    // ADC1 复位（正点原子例程中的做法）
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);

    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;
    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStruct.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStruct);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_56Cycles);

    ADC_Cmd(ADC1, ENABLE);
}

/*
*brief 获取实际电池电压
*retval 电压值（单位：V）
*note  内部进行均值滤波，采样次数由 ADC_FILTER_SAMPLES 宏控制
*/
float ADC_GetBatteryVoltage(void)
{
    uint32_t sum = 0;
    uint8_t i;
    float adc_voltage;

    for (i = 0; i < ADC_FILTER_SAMPLES; i++) {
        sum += ADC_ReadSingle();
    }

    adc_voltage = (float)(sum / ADC_FILTER_SAMPLES) * ADC_REF_VOLTAGE / 4096.0f;
    return adc_voltage * BATTERY_DIVIDER_RATIO;
}

/*
*brief 注册低压回调函数
*param cb 低压触发时调用的函数指针
*/
void ADC_RegisterLowVoltageCallback(LowVoltageCallback_t cb)
{
    low_voltage_callback = cb;
}

/*
*brief 检查低压状态并触发回调
*note  建议放在主循环或定时任务中调用，不要轮询
*       若检测到低压且注册了回调，则执行回调
*/
void ADC_CheckLowVoltage(void)
{
    if (ADC_GetBatteryVoltage() < BATTERY_LOW_THRESHOLD) {
        if (low_voltage_callback != NULL) {
            low_voltage_callback();
        }
    }
}
