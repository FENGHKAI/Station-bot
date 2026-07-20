/*
*file encoder.h
*brief 编码器驱动声明
*/

#ifndef ENCODER_H
#define ENCODER_H

#include "sys.h"

#define ENCODER_TIM_PERIOD (uint16_t)(65535) //定时器周期，统一采用为16位
#define PI 3.14159265358979f
#define SPEED_SAMPLE_PERIOD 100.0f // 速度采样频率 100Hz
#define WHEEL_DIAMETER 0.08f   // 轮子直径 
#define WHEEL_RESOLUTION 1560.0f // 26极磁编码器分辨率,开关霍尔：13*4*30（减速比）= 1560 
#define WHEEL_SCALE (PI*WHEEL_DIAMETER*SPEED_SAMPLE_PERIOD/WHEEL_RESOLUTION) // 速度计算系数 pi*直径*采样频率/编码器分辨率

/* 
*brief 编码器句柄结构体
*/
typedef struct 
{
    uint32_t RCC_APB1Periph_TIM;        // 定时器外设时钟使能掩码
    uint32_t RCC_AHB1Periph_GPIO;       // GPIO端口时钟使能掩码 
    TIM_TypeDef * TIM;                  // 编码器定时器句柄
    GPIO_TypeDef * GPIO_CH1;            // 编码器通道1 GPIO端口
    uint16_t GPIO_CH1_Pin;              // 编码器通道1 引脚号 (GPIO_Pin_x)
    uint8_t GPIO_CH1_PinSouce;          // 编码器通道1 引脚源编号 (GPIO_PinSourcex)
    GPIO_TypeDef * GPIO_CH2;            // 编码器通道2 GPIO端口
    uint16_t GPIO_CH2_Pin;              // 编码器通道2 引脚号 (GPIO_Pin_x)
    uint8_t GPIO_CH2_PinSouce;          // 编码器通道2 引脚源编号 (GPIO_PinSourcex)
    uint8_t GPIO_AF;                    // 复用功能编号 (GPIO_AF_TIMx)
} Encoder_Handle_t;

/* 
*brief 编码器编号共用体
*/
typedef enum {
    ENC_LF,
    ENC_LR,
    ENC_RF,
    ENC_RR,
    ENC_NUM
} Encoder_Index_t;

extern Encoder_Handle_t Encoder_LF;
extern Encoder_Handle_t Encoder_LR ;
extern Encoder_Handle_t Encoder_RF;
extern Encoder_Handle_t Encoder_RR;

void encoder_one_init(Encoder_Handle_t Encoder);
void encoder_init(void);
int32_t getCounter(Encoder_Handle_t Encoder);
float get_speed(Encoder_Index_t index);
void Update_Encoder_Speeds(void);

#endif