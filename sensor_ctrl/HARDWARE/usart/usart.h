/*
*file usart.h
*brief 通信串口驱动声明（USART2 + USART6）
*note  只负责收发，不处理协议，接收数据通过回调函数传递给上层
*/

#ifndef __USART_H
#define __USART_H

#include "sys.h"

// 接收回调类型（无返回值，接收单个字节）
typedef void (*UsartRxCallback_t)(uint8_t data);

// ----- 初始化函数 -----
void usart2_init(u32 bound, UsartRxCallback_t callback);
void usart6_init(u32 bound, UsartRxCallback_t callback);

// ----- 发送函数（轮询方式）-----
void usart2_send_byte(uint8_t data);
void usart2_send_bytes(uint8_t *data, uint16_t len);
void usart2_send_string(char *str);

void usart6_send_byte(uint8_t data);
void usart6_send_bytes(uint8_t *data, uint16_t len);
void usart6_send_string(char *str);

#endif
