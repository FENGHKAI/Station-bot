/*
*file protocol.h
*brief 统一通信协议（视觉模块 + 人机交互模块）
*note  帧头区分来源：0xA5=视觉模块(H板)，0x5A=人机交互模块(ESP8266)
*       帧结构：帧头(1B) + 长度(1B) + 命令字(1B) + 数据体(NB) + 校验和(1B) + 帧尾(0x0D 0x0A)
*       数据体采用字节填充（转义）防止与帧头帧尾混淆
*       通信模式：控制板 ↔ 视觉模块 通过 CMD_VISION_HANDOVER 交接控制权
*       持有控制权的一方主动发消息，另一方静默
*       状态由HMI内部维护，控制板通过具体命令触发HMI状态切换
*/

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "sys.h"

#define PROTOCOL_FRAME_MAX_LEN     128
#define PROTOCOL_DATA_MAX_LEN      64

// ----- 帧头定义（区分通信对象）-----
#define FRAME_HEADER_VISION        0xA5    // 视觉模块（OpenMV/STM32H7）
#define FRAME_HEADER_HMI           0x5A    // 人机交互模块（ESP8266）

// ----- 帧固定字节 -----
#define FRAME_TAIL1                0x0D
#define FRAME_TAIL2                0x0A
#define FRAME_ESC                  0x7D

// ----- 权力交接命令（双向通用，帧头 0xA5）-----
#define CMD_VISION_HANDOVER        0x01    // 交出控制权（谁收到谁接管）

// ----- 视觉模块 → 控制板（动作指令，帧头 0xA5）-----
#define CMD_VISION_MOVE_COORD      0x10    // 移动到指定坐标（数据体：x(2B)+y(2B)+z(2B)）
#define CMD_VISION_ARM_RETRACT     0x11    // 机械臂收回（数据体：无）

// ----- 视觉模块 → 控制板（任务完成，帧头 0xA5）-----
#define CMD_VISION_GRAB_DONE       0x20    // 抓取完成（数据体：1字节，0=成功，1=失败）
#define CMD_VISION_PLACE_DONE      0x21    // 放包裹完成（数据体：1字节，0=成功，1=失败）
#define CMD_VISION_BARCODE_DATA    0x22    // 识别到的取件码（数据体：取件码字符串）
#define CMD_VISION_SEARCH_TIMEOUT  0x23    // 搜索超时（数据体：无）

// ----- 人机交互模块命令（帧头 0x5A）-----
#define CMD_HMI_CODE_INPUT         0x10    // HMI→F：下发取件码（数据体：取件码字符串）
#define CMD_HMI_START              0x11    // HMI→F：启动指令（数据体：无）
#define CMD_HMI_STOP               0x12    // HMI→F：停止指令（数据体：无）
#define CMD_HMI_PICKED             0x13    // F→HMI：已取到包裹（数据体：无）
#define CMD_HMI_DELIVERED          0x14    // F→HMI：已送达（数据体：无）
#define CMD_HMI_ERROR_REPORT       0x15    // F→HMI：故障上报（数据体：1字节故障码）

// ----- 故障码（控制板上报具体故障类型）-----
#define HMI_ERROR_MOTOR            0x01    // 电机驱动故障
#define HMI_ERROR_SERVO            0x02    // 舵机通信故障
#define HMI_ERROR_VISION_TIMEOUT   0x03    // 视觉搜索超时
#define HMI_ERROR_CODE_MISMATCH    0x04    // 取件码不匹配
#define HMI_ERROR_GRAB_FAIL        0x05    // 抓取失败
#define HMI_ERROR_PLACE_FAIL       0x06    // 放包裹失败
#define HMI_ERROR_PATH_FAIL        0x07    // 路径规划失败
#define HMI_ERROR_UART_TIMEOUT     0x08    // 串口通信超时

// ----- 数据结构（仅用于解析结果传递）-----
typedef struct {
    uint8_t header;                         // 帧头（来源）
    uint8_t cmd;                            // 命令字
    uint8_t data_len;                       // 数据体长度
    uint8_t data[PROTOCOL_DATA_MAX_LEN];    // 数据体
    uint8_t valid;                          // 1=解析成功
} ProtocolResult_t;

// ----- 公共接口 -----
void Protocol_Init(void);

// ----- 组帧（控制板 ↔ 视觉，双向通用）-----
uint8_t Protocol_BuildVisionHandover(uint8_t *out_buf, uint8_t *out_len);

// ----- 组帧（视觉 → 控制板，H→F）-----
uint8_t Protocol_BuildVisionMoveCoord(int16_t x, int16_t y, int16_t z,
                                       uint8_t *out_buf, uint8_t *out_len);
uint8_t Protocol_BuildVisionArmRetract(uint8_t *out_buf, uint8_t *out_len);
uint8_t Protocol_BuildVisionGrabDone(uint8_t status, uint8_t *out_buf, uint8_t *out_len);
uint8_t Protocol_BuildVisionPlaceDone(uint8_t status, uint8_t *out_buf, uint8_t *out_len);
uint8_t Protocol_BuildVisionBarcodeData(char *code, uint8_t *out_buf, uint8_t *out_len);
uint8_t Protocol_BuildVisionSearchTimeout(uint8_t *out_buf, uint8_t *out_len);

// ----- 组帧（控制板 → HMI，F→HMI）-----
uint8_t Protocol_BuildHMIPicked(uint8_t *out_buf, uint8_t *out_len);
uint8_t Protocol_BuildHMIDelivered(uint8_t *out_buf, uint8_t *out_len);
uint8_t Protocol_BuildHMIError(uint8_t error_code, uint8_t *out_buf, uint8_t *out_len);

// ----- 帧解析 -----
uint8_t Protocol_ParseFrame(uint8_t *buf, uint8_t len, ProtocolResult_t *result);
uint8_t Protocol_ParseByte(uint8_t data, ProtocolResult_t *result);
void Protocol_ResetParser(void);

#endif
