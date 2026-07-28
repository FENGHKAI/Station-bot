/*
*file debug_usart.c
*brief 调试串口驱动实现（USART1）
*/

#include "debug_usart.h"

#if SYSTEM_SUPPORT_OS
#include "includes.h"
#endif

#if 1
#pragma import(__use_no_semihosting)

struct __FILE { int handle; };
FILE __stdout;

void _sys_exit(int x) { x = x; }

int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0);
    USART1->DR = (u8)ch;
    return ch;
}
#endif

u8 DBG_RX_BUF[DBG_REC_LEN];
u16 DBG_RX_STA = 0;

void debug_usart_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    USART_InitStruct.USART_BaudRate = bound;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStruct);

    USART_Cmd(USART1, ENABLE);

#if EN_DBG_RX
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
#endif
}

void USART1_IRQHandler(void)
{
#if SYSTEM_SUPPORT_OS
    OSIntEnter();
#endif
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        u8 res = USART_ReceiveData(USART1);

        if ((DBG_RX_STA & 0x8000) == 0) {
            if (DBG_RX_STA & 0x4000) {
                if (res != 0x0A) DBG_RX_STA = 0;
                else DBG_RX_STA |= 0x8000;
            } else {
                if (res == 0x0D) DBG_RX_STA |= 0x4000;
                else {
                    DBG_RX_BUF[DBG_RX_STA & 0x3FFF] = res;
                    DBG_RX_STA++;
                    if (DBG_RX_STA > (DBG_REC_LEN - 1)) DBG_RX_STA = 0;
                }
            }
        }
    }
#if SYSTEM_SUPPORT_OS
    OSIntExit();
#endif
}
