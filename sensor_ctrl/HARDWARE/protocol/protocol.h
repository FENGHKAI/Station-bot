/*
 *file protocol.h
 *brief 统一通信协议底层（控制板F板专用）
 *note 版本：V1.3，对应《通信协议架构说明文档》
 *     帧头区分来源：0xA5=视觉模块(H板)，0x5A=人机交互模块(HMI)
 *     帧结构：帧头(1B)+长度(1B)+命令字(1B)+数据体(NB)+校验和(1B)+帧尾(0x0D 0x0A)
 *     数据体采用字节填充（转义），校验和覆盖长度+命令字+转义后数据体
 *     本模块只负责底层：组帧/发送/解析/转义/校验，不含业务逻辑
 *     ACK重传计时、业务状态机由上层实现
 *     双串口(HMI/视觉)各自持有独立解析上下文，互不干扰
 *     串口发送接口通过函数指针注册，协议模块不依赖具体驱动
 */
#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "sys.h"
#include <string.h>

#define PROTOCOL_FRAME_MAX_LEN   128     // 帧最大长度
#define PROTOCOL_DATA_MAX_LEN    64      // 数据体最大长度(转义前)

// ----- 帧头定义（区分通信对象）-----
#define FRAME_HEADER_VISION      0xA5    // 视觉模块(H板/OpenMV)
#define FRAME_HEADER_HMI         0x5A    // 人机交互模块(HMI/ESP8266)

// ----- 帧固定字节 -----
#define FRAME_TAIL1              0x0D
#define FRAME_TAIL2              0x0A
#define FRAME_ESC                0x7D

// ----- 命令字定义 -----
#define CMD_ACK                  0x00    // ACK命令应答（双向通用，收到任何命令后回复）
#define CMD_VISION_HANDOVER      0x01    // 交出控制权（双向通用，谁收到谁接管）
#define CMD_CODE_INPUT           0x10    // HMI→F：下发取件码（3字节ASCII数字）
#define CMD_HMI_START            0x11    // HMI→F：启动指令
#define CMD_HMI_STOP             0x12    // HMI→F：停止指令
#define CMD_HMI_PICKED           0x13    // F→HMI：已取到包裹（HMI需回ACK）
#define CMD_HMI_DELIVERED        0x14    // F→HMI：已送达（HMI需回ACK）
#define CMD_HMI_ERROR_REPORT     0x15    // F→HMI：故障上报（HMI需回ACK）
#define CMD_VISION_MOVE_COORD    0x10    // H→F：移动到指定坐标（6字节，见坐标编码）
#define CMD_VISION_CLAW_CLOSE    0x11    // H→F：机械爪收紧
#define CMD_VISION_CLAW_OPEN     0x12    // H→F：机械爪放开
#define CMD_VISION_GRAB_DONE     0x20    // H→F：抓取动作完成（1字节状态）
#define CMD_VISION_PLACE_DONE    0x21    // H→F：放包裹动作完成（1字节状态）
#define CMD_TEST_LINK            0xFF    // 连接测试（双向通用，仅上电自检）

// ----- 故障码定义（三板统一）-----
#define HMI_ERROR_MOTOR          0x01    // 电机驱动故障
#define HMI_ERROR_SERVO          0x02    // 舵机通信故障
#define HMI_ERROR_VISION_TIMEOUT 0x03    // 视觉搜索超时
#define HMI_ERROR_GRAB_FAIL      0x04    // 抓取失败
#define HMI_ERROR_PLACE_FAIL     0x05    // 放包裹失败
#define HMI_ERROR_PATH_FAIL      0x06    // 路径规划失败
#define HMI_ERROR_UART_TIMEOUT   0x07    // 串口通信超时（命令重发3次无ACK）

// ----- 坐标编码（每轴2字节：符号1B + 幅值1B，上限255mm）-----
#define COORD_POS                0x00    // 符号字节：正
#define COORD_NEG                0x01    // 符号字节：负
#define COORD_MAX_ABS            255     // 坐标幅值上限(mm)

// ----- 动作完成状态 -----
#define ACTION_OK                0x00    // 成功
#define ACTION_FAIL              0x01    // 失败

// ----- 解析结果数据结构 -----
typedef struct {
    uint8_t header;                          // 帧头（来源：0xA5或0x5A）
    uint8_t cmd;                             // 命令字
    uint8_t data_len;                        // 数据体长度（转义前）
    uint8_t data[PROTOCOL_DATA_MAX_LEN];     // 数据体（已转义还原）
    uint8_t valid;                           // 1=解析成功
} ProtocolResult_t;

// ----- 解析状态机上下文（每路串口一个实例，互不干扰）-----
typedef struct {
    uint8_t buf[PROTOCOL_FRAME_MAX_LEN];     // 帧缓冲
    uint8_t len;                             // 当前已收长度
    uint8_t in_frame;                        // 是否正在收帧
} ProtocolParser_t;

// ----- 串口发送接口（函数指针注册）-----
// 发送函数签名：把len字节从对应串口发出
typedef void (*ProtocolSendFunc_t)(uint8_t *buf, uint8_t len);

// ----- 公共接口 -----
void    Protocol_Init(void);                                          // 协议模块初始化

void    Protocol_RegisterSendHMI(ProtocolSendFunc_t func);            // 注册HMI链路发送(USART2)
void    Protocol_RegisterSendVision(ProtocolSendFunc_t func);         // 注册视觉链路发送(USART6)

uint8_t Protocol_Send(uint8_t header, uint8_t cmd,
                      const uint8_t *data, uint8_t data_len);         // 组帧+发送(唯一发送入口)
uint8_t Protocol_SendAck(uint8_t header);                             // ACK快捷发送

uint8_t Protocol_ParseByte(ProtocolParser_t *p, uint8_t data,
                           ProtocolResult_t *result);                 // 逐字节解析(串口回调用)
uint8_t Protocol_ParseFrame(uint8_t *buf, uint8_t len,
                            ProtocolResult_t *result);                // 完整帧解析
void    Protocol_ParserReset(ProtocolParser_t *p);                    // 重置指定路解析状态机

#endif
