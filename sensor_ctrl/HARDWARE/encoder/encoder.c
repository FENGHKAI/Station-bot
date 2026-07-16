/* 
*file encoder.c
*brief 编码器驱动实现
*/
#include "encoder.h"

Encoder_Handle_t Encoder_LF = {
    RCC_APB1Periph_TIM2,
    RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB,
    TIM2,
    GPIOA,
    GPIO_Pin_15,
    GPIO_PinSource15,
    GPIOB,
    GPIO_Pin_3,
    GPIO_PinSource3,
    GPIO_AF_TIM2
};//左前轮编码器
Encoder_Handle_t Encoder_LR = {
    RCC_APB1Periph_TIM4,
    RCC_AHB1Periph_GPIOD,
    TIM4,
    GPIOD,
    GPIO_Pin_12,
    GPIO_PinSource12,
    GPIOD,
    GPIO_Pin_13,
    GPIO_PinSource13,
    GPIO_AF_TIM4
};//左后轮编码器
Encoder_Handle_t Encoder_RF = {
    RCC_APB1Periph_TIM3,
    RCC_AHB1Periph_GPIOB,
    TIM3,
    GPIOB,
    GPIO_Pin_4,
    GPIO_PinSource4,
    GPIOB,
    GPIO_Pin_5,
    GPIO_PinSource5,
    GPIO_AF_TIM3
};//右前轮编码器
Encoder_Handle_t Encoder_RR = {
    RCC_APB1Periph_TIM5,
    RCC_AHB1Periph_GPIOA,
    TIM5,
    GPIOA,
    GPIO_Pin_0,
    GPIO_PinSource0,
    GPIOA,
    GPIO_Pin_1,
    GPIO_PinSource1,
    GPIO_AF_TIM5
};//右后轮编码器

static float Encoder_Speed_t[4];  //0~3 分别对应 LF, LR, RF, RR
/* 
*brief 初始化一个编码器
*/
void encoder_one_init(Encoder_Handle_t Encoder)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 使能定时器和GPIO端口时钟 */
    RCC_APB1PeriphClockCmd(Encoder.RCC_APB1Periph_TIM,ENABLE);
    RCC_AHB1PeriphClockCmd(Encoder.RCC_AHB1Periph_GPIO,ENABLE);

    /* 配置GPIO端口 */
    GPIO_InitStructure.GPIO_Pin = Encoder.GPIO_CH1_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;          // 复用功能模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        // 推挽（输入模式时无影响）
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;      // 无上下拉（根据外部电路可调整）
    GPIO_Init(Encoder.GPIO_CH1, &GPIO_InitStructure);
    GPIO_PinAFConfig(Encoder.GPIO_CH1,Encoder.GPIO_CH1_PinSouce,Encoder.GPIO_AF );

    GPIO_InitStructure.GPIO_Pin = Encoder.GPIO_CH2_Pin;
    GPIO_Init(Encoder.GPIO_CH2, &GPIO_InitStructure);
    GPIO_PinAFConfig(Encoder.GPIO_CH2,Encoder.GPIO_CH2_PinSouce,Encoder.GPIO_AF );

    /* 配置定时器时基 */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(Encoder.TIM, &TIM_TimeBaseStructure);

    /* 配置编码器接口模式（模式3：双通道双边沿计数，四倍频） */
    TIM_EncoderInterfaceConfig(Encoder.TIM, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 0;
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(Encoder.TIM, &TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(Encoder.TIM, &TIM_ICInitStructure);

    /* 清零计数器，使能定时器 */
    TIM_SetCounter(Encoder.TIM, 0);
    TIM_Cmd(Encoder.TIM, ENABLE);
}

/* 
*breif 初始化所有编码器
*detail 调用ncoder_one_init(void)函数实现
*/
static void encoder_init(void)
{
    encoder_one_init(Encoder_LF);
    encoder_one_init(Encoder_LR);
    encoder_one_init(Encoder_RF);
    encoder_one_init(Encoder_RR);
}

/* 
*brief 获取编码器计数值
*/
int16_t getCounter(Encoder_Handle_t Encoder)
{
    int16_t encoder_cnt;
    encoder_cnt=(short)Encoder.TIM->CNT;
    Encoder.TIM->CNT=0;
    return encoder_cnt;
}

/* 
*brief 获取轮子速度接口
*detail index输入取值ENC_LF,ENC_LR,ENC_RF,ENC_RR,ENC_NUM
*/
float get_speed(Encoder_Index_t index)
{
    if (index >= ENC_NUM) return 0.0f;
    return Encoder_Speed_t[index];
}