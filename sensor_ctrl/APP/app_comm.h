/* ================ app_comm.h ================ */
#ifndef __APP_COMM_H
#define __APP_COMM_H

#include "sys.h"

void    app_comm_init(void);                              /* 建队列/互斥锁/事件组，并初始化两路串口 */
void    app_comm_task(void *arg);                         /* FreeRTOS 通信任务 */

/* 带ACK重传的发送，0=成功 1=重传耗尽失败 */
uint8_t comm_send_ack_vision(uint8_t cmd, const uint8_t *data, uint8_t len);

/* 单次发送（HMI 状态上报用） */
void    comm_send_hmi(uint8_t cmd, const uint8_t *data, uint8_t len);

#endif
