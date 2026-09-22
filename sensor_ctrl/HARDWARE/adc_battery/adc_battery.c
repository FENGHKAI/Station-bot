/* *file adc_battery.c
 * *brief 电源电量检测驱动实现（ADC1_IN12，PC2，均值滤波）
 * *note 3S锂电两级电量管理：
 *  >10.2V 正常 | 9.9~10.2V 报警收回 | <9.9V 强制停车保护
 *  检测应在电机带载状态下进行（空载电压偏高，会漏报）
 */
#include "adc_battery.h"
#include <stddef.h>

static LowVoltageCallback_t low_voltage_callback = NULL;
static BatteryLevel_t batt_level = BATT_LEVEL_OK;   // 当前电量等级缓存

// 单次 ADC 采样
static uint16_t ADC_ReadSingle(void)
{
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    return ADC_GetConversionValue(ADC1);
}

/* *brief 初始化 ADC1_IN12（PC2）
 * *note 使用 ADC1，通道 12，扫描模式关闭，单次转换 */
void ADC_Battery_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    ADC_InitTypeDef ADC_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    // ADC1 复位（正点原子例程中的做法）
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);

    ADC_InitStruct.ADC_Resolution          = ADC_Resolution_12b;
    ADC_InitStruct.ADC_ScanConvMode        = DISABLE;
    ADC_InitStruct.ADC_ContinuousConvMode  = DISABLE;
    ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct.ADC_DataAlign           = ADC_DataAlign_Right;
    ADC_InitStruct.ADC_NbrOfConversion     = 1;
    ADC_Init(ADC1, &ADC_InitStruct);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_56Cycles);
    ADC_Cmd(ADC1, ENABLE);
}

/* *brief 获取实际电池电压
 * *retval 电压值（单位：V）
 * *note 内部进行均值滤波，采样次数由 ADC_FILTER_SAMPLES 宏控制 */
float ADC_GetBatteryVoltage(void)
{
    uint32_t sum = 0;
    uint8_t i;
    float adc_voltage;

    for (i = 0; i < ADC_FILTER_SAMPLES; i++)
    {
        sum += ADC_ReadSingle();
    }

    adc_voltage = (float)(sum / ADC_FILTER_SAMPLES) * ADC_REF_VOLTAGE / 4096.0f;

    return adc_voltage * BATTERY_DIVIDER_RATIO;
}

/* *brief 获取当前电量等级（最近一次检测的结果）
 * *retval BATT_LEVEL_OK / BATT_LEVEL_WARN / BATT_LEVEL_STOP */
BatteryLevel_t ADC_GetBatteryLevel(void)
{
    return batt_level;
}

/* *brief 注册低压回调函数
 * *param cb 低压触发时调用的函数指针（报警和停车都会触发，
 *           回调内用 ADC_GetBatteryLevel() 区分等级）*/
void ADC_RegisterLowVoltageCallback(LowVoltageCallback_t cb)
{
    low_voltage_callback = cb;
}

/* *brief 检查低压状态并触发回调
 * *retval 当前电量等级
 * *note 建议放在主循环中周期调用（电机运转时检测更准确）
 *  等级判定：
 *   电压 >= 10.2V        → OK（正常）
 *   9.9V <= 电压 < 10.2V → WARN（报警，尽快收回）
 *   电压 < 9.9V           → STOP（强制停车保护）*/
BatteryLevel_t ADC_CheckLowVoltage(void)
{
    float volt = ADC_GetBatteryVoltage();

    if (volt < BATTERY_STOP_THRESHOLD)
        batt_level = BATT_LEVEL_STOP;
    else if (volt < BATTERY_WARN_THRESHOLD)
        batt_level = BATT_LEVEL_WARN;
    else
        batt_level = BATT_LEVEL_OK;

    // 非正常等级时触发回调（回调内可据等级执行提醒或停车）
    if (batt_level != BATT_LEVEL_OK && low_voltage_callback != NULL)
    {
        low_voltage_callback();
    }

    return batt_level;
}

