/*
*file usart.h
*brief 串口驱动声明（支持 USART1 和 USART3）
*note  调试串口：USART1（PA9 TX，PA10 RX）连接 CH340C
*       ESP8266 串口：USART3（PD5 TX，PD6 RX）
*       接收缓冲区大小 200 字节，以 0x0D 0x0A 作为帧结束标志
*/

#ifndef __USART_H
#define __USART_H

#include "sys.h"

#define USART_REC_LEN  200   // 最大接收字节数

// 使能串口接收中断（1：使能，0：禁止）
#define EN_USART1_RX   1
#define EN_USART3_RX   1

// USART1 接收缓冲区和状态
extern u8  USART1_RX_BUF[USART_REC_LEN];
extern u16 USART1_RX_STA;   // bit15：完成标志，bit14：收到0x0D，bit13~0：有效字节数

// USART3 接收缓冲区和状态
extern u8  USART3_RX_BUF[USART_REC_LEN];
extern u16 USART3_RX_STA;

void uart1_init(u32 bound);
void uart3_init(u32 bound);

#endif
