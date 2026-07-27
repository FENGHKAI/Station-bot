/*
*file usart.c
*brief 串口驱动实现（USART1 + USART3）
*note  支持 printf 重定向到 USART1
*       支持裸机和 UCOS 模式（通过 SYSTEM_SUPPORT_OS 宏控制）
*/

#include "usart.h"

#if SYSTEM_SUPPORT_OS
#include "includes.h"
#endif

// ----- printf 重定向（只用于 USART1）-----
#if 1
#pragma import(__use_no_semihosting)

struct __FILE { int handle; };
FILE __stdout;

void _sys_exit(int x) { x = x; }

int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0);   // 等待发送缓冲区空
    USART1->DR = (u8)ch;
    return ch;
}
#endif

// ----- USART1 接收缓冲区 -----
u8 USART1_RX_BUF[USART_REC_LEN];
u16 USART1_RX_STA = 0;

// ----- USART3 接收缓冲区 -----
u8 USART3_RX_BUF[USART_REC_LEN];
u16 USART3_RX_STA = 0;

/*
*brief 初始化 USART1（PA9 TX，PA10 RX）
*param bound 波特率
*/
void uart1_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // 使能时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    // 复用功能映射
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    // 配置 PA9, PA10 为复用推挽
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 配置 USART1
    USART_InitStruct.USART_BaudRate = bound;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStruct);

    USART_Cmd(USART1, ENABLE);

#if EN_USART1_RX
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
#endif
}

/*
*brief 初始化 USART3（PD5 TX，PD6 RX）用于 ESP8266
*param bound 波特率
*/
void uart3_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // 使能时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    // 复用功能映射（AF7）
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART3);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    USART_InitStruct.USART_BaudRate = bound;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStruct);

    USART_Cmd(USART3, ENABLE);

#if EN_USART3_RX
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;   // 可调整
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
#endif
}

// ----- USART1 中断服务函数 -----
void USART1_IRQHandler(void)
{
#if SYSTEM_SUPPORT_OS
    OSIntEnter();
#endif
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        u8 res = USART_ReceiveData(USART1);
        // 与原有逻辑一致：以 0x0D 0x0A 为帧结束标志
        if ((USART1_RX_STA & 0x8000) == 0) {
            if (USART1_RX_STA & 0x4000) {
                if (res != 0x0A) USART1_RX_STA = 0;
                else USART1_RX_STA |= 0x8000;
            } else {
                if (res == 0x0D) USART1_RX_STA |= 0x4000;
                else {
                    USART1_RX_BUF[USART1_RX_STA & 0x3FFF] = res;
                    USART1_RX_STA++;
                    if (USART1_RX_STA > (USART_REC_LEN - 1)) USART1_RX_STA = 0;
                }
            }
        }
    }
#if SYSTEM_SUPPORT_OS
    OSIntExit();
#endif
}

// ----- USART3 中断服务函数 -----
void USART3_IRQHandler(void)
{
#if SYSTEM_SUPPORT_OS
    OSIntEnter();
#endif
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        u8 res = USART_ReceiveData(USART3);
        // 同样使用 0x0D 0x0A 帧结束标志
        if ((USART3_RX_STA & 0x8000) == 0) {
            if (USART3_RX_STA & 0x4000) {
                if (res != 0x0A) USART3_RX_STA = 0;
                else USART3_RX_STA |= 0x8000;
            } else {
                if (res == 0x0D) USART3_RX_STA |= 0x4000;
                else {
                    USART3_RX_BUF[USART3_RX_STA & 0x3FFF] = res;
                    USART3_RX_STA++;
                    if (USART3_RX_STA > (USART_REC_LEN - 1)) USART3_RX_STA = 0;
                }
            }
        }
    }
#if SYSTEM_SUPPORT_OS
    OSIntExit();
#endif
}
