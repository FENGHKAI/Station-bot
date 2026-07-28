/*
*file debug_usart.h
*brief 调试串口驱动声明（USART1，PA9 TX，PA10 RX）
*/

#ifndef __DEBUG_USART_H
#define __DEBUG_USART_H

#include "sys.h"

#define DBG_REC_LEN  200
#define EN_DBG_RX    1

extern u8  DBG_RX_BUF[DBG_REC_LEN];
extern u16 DBG_RX_STA;

void debug_usart_init(u32 bound);

#endif
