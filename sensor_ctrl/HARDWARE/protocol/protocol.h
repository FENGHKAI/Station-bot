/*
*file protocol.h
*brief 统一通信协议（视觉模块 + 人机交互模块）
*note  帧头区分来源：0xA5=视觉模块(H板)，0x5A=人机交互模块(ESP8266)
*       帧结构：帧头(1B) + 长度(1B) + 命令字(1B) + 数据体(NB) + 校验和(1B) + 帧尾(0x0D 0x0A)
*       数据体采用字节填充（转义）防止与帧头帧尾混淆
*       本模块只负责：帧结构定义、封装成帧、解析帧，不涉及数据存储
*/

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "sys.h"

// ----- 缓冲区大小 -----
#define PROTOCOL_FRAME_MAX_LEN     128
#define PROTOCOL_DATA_MAX_LEN      64

// ----- 帧头定义（用于区分通信对象）-----
#define FRAME_HEADER_VISION        0xA5    // 视觉模块（OpenMV/STM32H7）
#define FRAME_HEADER_HMI           0x5A    // 人机交互模块（ESP8266）

// ----- 帧固定字节 -----
#define FRAME_TAIL1                0x0D
#define FRAME_TAIL2                0x0A
#define FRAME_ESC                  0x7D

// ----- 转义规则 -----
//  帧头(0xA5/0x5A) → 0x7D 0x01
//  0x0D → 0x7D 0x02
//  0x0A → 0x7D 0x03
//  0x7D → 0x7D 0x00

// ----- 视觉模块命令（帧头 0xA5）-----
#define CMD_VISION_COORD_REQ       0x01    // F→H：请求坐标
#define CMD_VISION_COORD_RSP       0x02    // H→F：返回坐标
#define CMD_VISION_SERVO_CTRL      0x10    // F→H：控制舵机
#define CMD_VISION_SERVO_CTRL_ACK  0x11    // H→F：控制确认
#define CMD_VISION_SERVO_READ_POS  0x20    // F→H：读取位置
#define CMD_VISION_SERVO_POS_RSP   0x21    // H→F：返回位置
#define CMD_VISION_SERVO_READ_TEMP 0x22    // F→H：读取温度/电压
#define CMD_VISION_SERVO_TEMP_RSP  0x23    // H→F：返回温度/电压

// ----- 人机交互模块命令（帧头 0x5A）-----
#define CMD_HMI_STATUS_REPORT      0x01    // F→HMI：状态上报
#define CMD_HMI_ERROR_REPORT       0x02    // F→HMI：故障上报
#define CMD_HMI_CODE_INPUT         0x10    // HMI→F：下发取件码
#define CMD_HMI_START              0x11    // HMI→F：启动指令

// ----- 状态码（人机交互）-----
#define HMI_STATUS_WAITING         0x00    // 待取件
#define HMI_STATUS_GOING           0x01    // 机器人前往中
#define HMI_STATUS_DELIVERED       0x02    // 已送达

// ----- 数据结构（仅用于解析结果传递）-----
typedef struct {
    uint8_t header;                         // 帧头（来源）
    uint8_t cmd;                            // 命令字
    uint8_t data_len;                       // 数据体长度
    uint8_t data[PROTOCOL_DATA_MAX_LEN];    // 数据体
    uint8_t valid;                          // 1=解析成功
} ProtocolResult_t;

// ----- 公共接口 -----

// 初始化（重置解析状态机）
void Protocol_Init(void);

// ----- 组帧（封装成帧）-----

// 通用组帧
uint8_t Protocol_BuildFrame(uint8_t header, uint8_t cmd, uint8_t *data,
                            uint8_t data_len, uint8_t *out_buf, uint8_t *out_len);

// 视觉模块组帧
uint8_t Protocol_BuildVisionCoordReq(uint8_t *out_buf, uint8_t *out_len);
uint8_t Protocol_BuildVisionServoCtrl(uint8_t id, uint16_t pwm, uint16_t time_ms,
                                       uint8_t *out_buf, uint8_t *out_len);
uint8_t Protocol_BuildVisionServoReadPos(uint8_t id, uint8_t *out_buf, uint8_t *out_len);
uint8_t Protocol_BuildVisionServoReadTemp(uint8_t id, uint8_t *out_buf, uint8_t *out_len);
uint8_t Protocol_BuildVisionAck(uint8_t ack_cmd, uint8_t *out_buf, uint8_t *out_len);

// 人机交互模块组帧
uint8_t Protocol_BuildHMIStatus(uint8_t status, uint8_t *out_buf, uint8_t *out_len);
uint8_t Protocol_BuildHMIError(char *err_str, uint8_t *out_buf, uint8_t *out_len);

// ----- 帧解析 -----

// 解析完整帧（已收到完整一帧时使用）
uint8_t Protocol_ParseFrame(uint8_t *buf, uint8_t len, ProtocolResult_t *result);

// 逐字节解析（流式接收时使用，内部维护状态机）
uint8_t Protocol_ParseByte(uint8_t data, ProtocolResult_t *result);

// 重置解析状态机（发生错误时调用）
void Protocol_ResetParser(void);

#endif
