/*
*file debug_usart.c
*brief 调试串口驱动实现（UART4，PC10 TX，PC11 RX）
*note  蓝牙无线调试，printf 重定向到 UART4
*/

#include "debug_usart.h"

// ----- printf 重定向（绑定 UART4）-----
#if 1
#pragma import(__use_no_semihosting)

struct __FILE { int handle; };
FILE __stdout;

void _sys_exit(int x) { x = x; }

int fputc(int ch, FILE *f)
{
    while ((UART4->SR & 0X40) == 0);
    UART4->DR = (u8)ch;
    return ch;
}
#endif

// ----- 接收缓冲区 -----
u8 DBG_RX_BUF[DBG_REC_LEN];
u16 DBG_RX_STA = 0;

/*
*brief 初始化调试串口（UART4，PC10 TX，PC11 RX）
*param bound 波特率（默认 115200）
*/
void debug_usart_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    USART_InitStruct.USART_BaudRate = bound;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &USART_InitStruct);

    USART_Cmd(UART4, ENABLE);

#if EN_DBG_RX
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
#endif
}

/*
*brief UART4 中断服务函数（接收）
*note  以 0x0D 0x0A 为帧结束标志
*/
void UART4_IRQHandler(void)
{

    if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) {
        u8 res = USART_ReceiveData(UART4);

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
}

