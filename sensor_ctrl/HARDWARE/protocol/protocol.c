/*
*file protocol.c
*brief 统一通信协议实现（视觉模块 + 人机交互模块）
*note  通过帧头 0xA5/0x5A 自动区分来源
*       本模块只负责协议解析与封装，不涉及数据存储
*/

#include "protocol.h"
#include <string.h>

// ----- 逐字节解析状态机（私有）-----
static uint8_t parse_buf[PROTOCOL_FRAME_MAX_LEN];
static uint8_t parse_len = 0;
static uint8_t parse_in_frame = 0;

// ============================================================
//                        转义编码/解码
// ============================================================

/*
*brief 转义编码
*param src 原始数据体
*param src_len 原始长度
*param dst 输出缓冲区（转义后）
*param dst_len 输出长度（指针）
*note  转义规则：0xA5/0x5A→0x7D 0x01, 0x0D→0x7D 0x02,
*       0x0A→0x7D 0x03, 0x7D→0x7D 0x00
*/
static void escape_encode(uint8_t *src, uint8_t src_len, uint8_t *dst, uint8_t *dst_len)
{
    uint8_t i;
    *dst_len = 0;

    for (i = 0; i < src_len; i++) {
        switch (src[i]) {
            case FRAME_HEADER_VISION:
            case FRAME_HEADER_HMI:
                dst[(*dst_len)++] = FRAME_ESC;
                dst[(*dst_len)++] = 0x01;
                break;
            case FRAME_TAIL1:
                dst[(*dst_len)++] = FRAME_ESC;
                dst[(*dst_len)++] = 0x02;
                break;
            case FRAME_TAIL2:
                dst[(*dst_len)++] = FRAME_ESC;
                dst[(*dst_len)++] = 0x03;
                break;
            case FRAME_ESC:
                dst[(*dst_len)++] = FRAME_ESC;
                dst[(*dst_len)++] = 0x00;
                break;
            default:
                dst[(*dst_len)++] = src[i];
                break;
        }
    }
}

/*
*brief 转义解码
*param src 转义后的数据体
*param src_len 转义后长度
*param header 原始帧头（用于恢复0x01对应的真实值）
*param dst 输出缓冲区（原始数据）
*param dst_len 输出长度（指针）
*retval 1=成功，0=失败
*/
static uint8_t escape_decode_with_context(uint8_t *src, uint8_t src_len,
                                           uint8_t header, uint8_t *dst, uint8_t *dst_len)
{
    uint8_t i = 0;
    *dst_len = 0;

    while (i < src_len) {
        if (src[i] == FRAME_ESC) {
            if (i + 1 >= src_len) return 0;
            switch (src[i + 1]) {
                case 0x01: dst[(*dst_len)++] = header; break;
                case 0x02: dst[(*dst_len)++] = FRAME_TAIL1; break;
                case 0x03: dst[(*dst_len)++] = FRAME_TAIL2; break;
                case 0x00: dst[(*dst_len)++] = FRAME_ESC; break;
                default: return 0;
            }
            i += 2;
        } else {
            dst[(*dst_len)++] = src[i++];
        }
    }
    return 1;
}

// ============================================================
//                        校验和
// ============================================================

/*
*brief 计算校验和（所有字节累加，取低8位）
*param buf 待校验数据
*param len 数据长度
*retval 校验和
*/
static uint8_t calc_checksum(uint8_t *buf, uint8_t len)
{
    uint8_t i, sum = 0;
    for (i = 0; i < len; i++) {
        sum += buf[i];
    }
    return sum;
}

// ============================================================
//                        组帧（私有）
// ============================================================

/*
*brief 内部组帧
*param header 帧头（0xA5 或 0x5A）
*param cmd 命令字
*param data 原始数据体（可为NULL）
*param data_len 原始长度
*param out_buf 输出缓冲区
*param out_len 输出长度（指针）
*retval 1=成功，0=失败
*/
static uint8_t build_frame(uint8_t header, uint8_t cmd, uint8_t *data,
                           uint8_t data_len, uint8_t *out_buf, uint8_t *out_len)
{
    uint8_t escaped[PROTOCOL_DATA_MAX_LEN * 2 + 4];
    uint8_t escaped_len;
    uint8_t len = 0;
    uint8_t i;
    uint8_t checksum;

    if (out_buf == NULL || out_len == NULL) return 0;
    if (data_len > PROTOCOL_DATA_MAX_LEN) return 0;

    if (data && data_len > 0) {
        escape_encode(data, data_len, escaped, &escaped_len);
    } else {
        escaped_len = 0;
    }

    if (escaped_len + 6 > PROTOCOL_FRAME_MAX_LEN) return 0;

    out_buf[len++] = header;
    out_buf[len++] = data_len;
    out_buf[len++] = cmd;

    for (i = 0; i < escaped_len; i++) {
        out_buf[len++] = escaped[i];
    }

    checksum = calc_checksum(&out_buf[1], len - 1);
    out_buf[len++] = checksum;
    out_buf[len++] = FRAME_TAIL1;
    out_buf[len++] = FRAME_TAIL2;

    *out_len = len;
    return 1;
}

// ============================================================
//                        帧解析（私有）
// ============================================================

/*
*brief 内部解析帧
*param buf 完整帧（含帧头到帧尾）
*param len 帧长度
*param result 输出解析结果
*retval 1=成功，0=失败
*/
static uint8_t parse_frame_internal(uint8_t *buf, uint8_t len, ProtocolResult_t *result)
{
    uint8_t header, data_len, cmd, checksum_recv, checksum_calc;
    uint8_t escaped_data[PROTOCOL_DATA_MAX_LEN * 2 + 4];
    uint8_t raw_data[PROTOCOL_DATA_MAX_LEN];
    uint8_t raw_len;
    uint8_t i;
    uint8_t payload_start;

    if (buf == NULL || result == NULL) return 0;
    if (len < 6) return 0;

    header = buf[0];
    if (header != FRAME_HEADER_VISION && header != FRAME_HEADER_HMI) return 0;
    if (buf[len - 2] != FRAME_TAIL1 || buf[len - 1] != FRAME_TAIL2) return 0;

    data_len = buf[1];
    cmd = buf[2];

    payload_start = 3;
    uint8_t escaped_len = len - 3 - 1 - 2;
    if (escaped_len > 0) {
        if (escaped_len > sizeof(escaped_data)) return 0;
        for (i = 0; i < escaped_len; i++) {
            escaped_data[i] = buf[payload_start + i];
        }
    }

    checksum_recv = buf[len - 3];
    checksum_calc = calc_checksum(&buf[1], len - 3 - 1);
    if (checksum_recv != checksum_calc) return 0;

    if (data_len > 0) {
        if (!escape_decode_with_context(escaped_data, escaped_len, header,
                                         raw_data, &raw_len)) return 0;
        if (raw_len != data_len) return 0;
    } else {
        raw_len = 0;
    }

    result->header = header;
    result->cmd = cmd;
    result->data_len = raw_len;
    if (raw_len > 0) {
        memcpy(result->data, raw_data, raw_len);
    }
    result->valid = 1;

    return 1;
}

// ============================================================
//                    公共接口：初始化
// ============================================================

/*
*brief 初始化协议模块（重置状态机）
*/
void Protocol_Init(void)
{
    parse_len = 0;
    parse_in_frame = 0;
}

// ============================================================
//                    公共接口：组帧
// ============================================================

/*
*brief 通用组帧
*param header 帧头（0xA5 或 0x5A）
*param cmd 命令字
*param data 原始数据体（可为NULL）
*param data_len 原始长度
*param out_buf 输出缓冲区
*param out_len 输出长度（指针）
*retval 1=成功，0=失败
*/
uint8_t Protocol_BuildFrame(uint8_t header, uint8_t cmd, uint8_t *data,
                            uint8_t data_len, uint8_t *out_buf, uint8_t *out_len)
{
    if (header != FRAME_HEADER_VISION && header != FRAME_HEADER_HMI) return 0;
    return build_frame(header, cmd, data, data_len, out_buf, out_len);
}

/*
*brief 封装坐标请求帧（F→H）
*/
uint8_t Protocol_BuildVisionCoordReq(uint8_t *out_buf, uint8_t *out_len)
{
    return build_frame(FRAME_HEADER_VISION, CMD_VISION_COORD_REQ, NULL, 0, out_buf, out_len);
}

/*
*brief 封装舵机控制帧（F→H）
*/
uint8_t Protocol_BuildVisionServoCtrl(uint8_t id, uint16_t pwm, uint16_t time_ms,
                                       uint8_t *out_buf, uint8_t *out_len)
{
    uint8_t data[5];
    data[0] = id;
    data[1] = (pwm >> 8) & 0xFF;
    data[2] = pwm & 0xFF;
    data[3] = (time_ms >> 8) & 0xFF;
    data[4] = time_ms & 0xFF;
    return build_frame(FRAME_HEADER_VISION, CMD_VISION_SERVO_CTRL, data, 5, out_buf, out_len);
}

/*
*brief 封装读取舵机位置帧（F→H）
*/
uint8_t Protocol_BuildVisionServoReadPos(uint8_t id, uint8_t *out_buf, uint8_t *out_len)
{
    return build_frame(FRAME_HEADER_VISION, CMD_VISION_SERVO_READ_POS, &id, 1, out_buf, out_len);
}

/*
*brief 封装读取舵机温度/电压帧（F→H）
*/
uint8_t Protocol_BuildVisionServoReadTemp(uint8_t id, uint8_t *out_buf, uint8_t *out_len)
{
    return build_frame(FRAME_HEADER_VISION, CMD_VISION_SERVO_READ_TEMP, &id, 1, out_buf, out_len);
}

/*
*brief 封装ACK确认帧（F→H）
*/
uint8_t Protocol_BuildVisionAck(uint8_t ack_cmd, uint8_t *out_buf, uint8_t *out_len)
{
    return build_frame(FRAME_HEADER_VISION, CMD_VISION_COORD_REQ + 0x10, &ack_cmd, 1, out_buf, out_len);
}

/*
*brief 封装状态上报帧（F→HMI）
*/
uint8_t Protocol_BuildHMIStatus(uint8_t status, uint8_t *out_buf, uint8_t *out_len)
{
    return build_frame(FRAME_HEADER_HMI, CMD_HMI_STATUS_REPORT, &status, 1, out_buf, out_len);
}

/*
*brief 封装故障上报帧（F→HMI）
*/
uint8_t Protocol_BuildHMIError(char *err_str, uint8_t *out_buf, uint8_t *out_len)
{
    uint8_t len;
    if (err_str == NULL) return 0;
    len = strlen(err_str);
    if (len > PROTOCOL_DATA_MAX_LEN) len = PROTOCOL_DATA_MAX_LEN;
    return build_frame(FRAME_HEADER_HMI, CMD_HMI_ERROR_REPORT,
                       (uint8_t*)err_str, len, out_buf, out_len);
}

// ============================================================
//                    公共接口：帧解析
// ============================================================

/*
*brief 解析完整帧
*param buf 完整帧
*param len 帧长度
*param result 输出解析结果
*retval 1=成功，0=失败
*/
uint8_t Protocol_ParseFrame(uint8_t *buf, uint8_t len, ProtocolResult_t *result)
{
    return parse_frame_internal(buf, len, result);
}

/*
*brief 逐字节解析（状态机）
*param data 输入字节
*param result 输出解析结果（解析完成时填充）
*retval 1=解析到完整帧，0=尚未完成
*/
uint8_t Protocol_ParseByte(uint8_t data, ProtocolResult_t *result)
{
    if (result == NULL) return 0;

    if (!parse_in_frame) {
        if (data == FRAME_HEADER_VISION || data == FRAME_HEADER_HMI) {
            parse_buf[0] = data;
            parse_len = 1;
            parse_in_frame = 1;
        }
        return 0;
    }

    parse_buf[parse_len++] = data;

    if (parse_len >= 2 &&
        parse_buf[parse_len - 2] == FRAME_TAIL1 &&
        parse_buf[parse_len - 1] == FRAME_TAIL2) {
        uint8_t ret = parse_frame_internal(parse_buf, parse_len, result);
        parse_len = 0;
        parse_in_frame = 0;
        return ret;
    }

    if (parse_len >= PROTOCOL_FRAME_MAX_LEN) {
        parse_len = 0;
        parse_in_frame = 0;
    }

    return 0;
}

/*
*brief 重置解析状态机
*/
void Protocol_ResetParser(void)
{
    parse_len = 0;
    parse_in_frame = 0;
}
