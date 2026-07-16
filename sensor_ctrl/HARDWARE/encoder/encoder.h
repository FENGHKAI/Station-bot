/*
*file encoder.h
*brief 编码器驱动声明
*/

#ifndef ENCODER_H
#define ENCODER_H

#include "sys.h"

#define ENCODER_TIM_PERIOD (uint16_t)(65535) //定时器周期，统一采用为16位

#define PI 3.14159265358979f
#define SPEED_SAMPLE_PERIOD 100.0f /*速度采样频率 100Hz*/
#define WHEEL_DIAMETER 0.08f   /**< 轮子直径 */
#define WHEEL_RESOLUTION 1560.0f /*26极磁编码器分辨率,开关霍尔：13*4*30（减速比）= 1560 */
#define WHEEL_SCALE (PI*WHEEL_DIAMETER*SPEED_SAMPLE_PERIOD/WHEEL_RESOLUTION) /*速度计算系数 pi*直径*采样频率/编码器分辨率*/


typedef struct 
{
    uint32_t RCC_APB1Periph_TIM;
    uint32_t RCC_AHB1Periph_GPIO;
    TIM_TypeDef * TIM;
    GPIO_TypeDef * GPIO_CH1;
    uint16_t GPIO_CH1_Pin;
    uint8_t GPIO_CH1_PinSouce;
    GPIO_TypeDef * GPIO_CH2;
    uint16_t GPIO_CH2_Pin;
    uint8_t GPIO_CH2_PinSouce;
    uint8_t GPIO_AF;
}Encoder_Handle_t;//自定义编码器数据结构体

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

extern float Encoder_Speed_t[4];

void encoder_one_init(Encoder_Handle_t Encoder);
static void encoder_init(void);
int16_t getCounter(Encoder_Handle_t Encoder);
float get_speed(Encoder_Index_t index);

#endif