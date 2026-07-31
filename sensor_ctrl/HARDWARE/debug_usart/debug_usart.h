/*
*file debug_usart.h
*brief 调试串口驱动声明（UART4，PC10 TX，PC11 RX，蓝牙无线）
*note  用于无线调试，波特率默认 115200
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
