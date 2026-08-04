/*
*file protocol.c
*brief 统一通信协议实现（视觉模块 + 人机交互模块）
*note  权力交接制：CMD_VISION_HANDOVER 交出控制权
*       持有控制权的一方主动发消息
*/

#include "protocol.h"

// ----- 逐字节解析状态机（私有）-----
static uint8_t parse_buf[PROTOCOL_FRAME_MAX_LEN];
static uint8_t parse_len = 0;
static uint8_t parse_in_frame = 0;

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
//                    公共接口
// ============================================================

/*
*brief 初始化协议模块（重置状态机）
*/
void Protocol_Init(void)
{
    parse_len = 0;
    parse_in_frame = 0;
}

// ----- 控制板 ↔ 视觉（权力交接，双向通用）-----

/*
*brief 封装权力交接帧
*param out_buf 输出缓冲区
*param out_len 输出长度（指针）
*retval 1=成功，0=失败
*note  谁收到此命令谁接管控制权，另一方进入静默
*/
uint8_t Protocol_BuildVisionHandover(uint8_t *out_buf, uint8_t *out_len)
{
    return build_frame(FRAME_HEADER_VISION, CMD_VISION_HANDOVER, NULL, 0, out_buf, out_len);
}

// ----- 视觉 → 控制板（动作指令）-----

/*
*brief 封装移动到指定坐标指令（H→F）
*param x, y, z 目标坐标（mm）
*param out_buf 输出缓冲区
*param out_len 输出长度（指针）
*retval 1=成功，0=失败
*note  控制板收到后执行逆运动学，驱动舵机到达目标位置
*/
uint8_t Protocol_BuildVisionMoveCoord(int16_t x, int16_t y, int16_t z,
                                       uint8_t *out_buf, uint8_t *out_len)
{
    uint8_t data[6];
    data[0] = (x >> 8) & 0xFF;
    data[1] = x & 0xFF;
    data[2] = (y >> 8) & 0xFF;
    data[3] = y & 0xFF;
    data[4] = (z >> 8) & 0xFF;
    data[5] = z & 0xFF;
    return build_frame(FRAME_HEADER_VISION, CMD_VISION_MOVE_COORD, data, 6, out_buf, out_len);
}

/*
*brief 封装机械臂收回指令（H→F）
*param out_buf 输出缓冲区
*param out_len 输出长度（指针）
*retval 1=成功，0=失败
*note  控制板收到后将机械臂回到初始姿态（收紧状态）
*/
uint8_t Protocol_BuildVisionArmRetract(uint8_t *out_buf, uint8_t *out_len)
{
    return build_frame(FRAME_HEADER_VISION, CMD_VISION_ARM_RETRACT, NULL, 0, out_buf, out_len);
}

// ----- 视觉 → 控制板（任务完成）-----

/*
*brief 封装抓取完成信号（H→F）
*param status 0=成功，1=失败
*param out_buf 输出缓冲区
*param out_len 输出长度（指针）
*retval 1=成功，0=失败
*/
uint8_t Protocol_BuildVisionGrabDone(uint8_t status, uint8_t *out_buf, uint8_t *out_len)
{
    return build_frame(FRAME_HEADER_VISION, CMD_VISION_GRAB_DONE, &status, 1, out_buf, out_len);
}

/*
*brief 封装放包裹完成信号（H→F）
*param status 0=成功，1=失败
*param out_buf 输出缓冲区
*param out_len 输出长度（指针）
*retval 1=成功，0=失败
*/
uint8_t Protocol_BuildVisionPlaceDone(uint8_t status, uint8_t *out_buf, uint8_t *out_len)
{
    return build_frame(FRAME_HEADER_VISION, CMD_VISION_PLACE_DONE, &status, 1, out_buf, out_len);
}

/*
*brief 封装识别到的取件码（H→F）
*param code 取件码字符串
*param out_buf 输出缓冲区
*param out_len 输出长度（指针）
*retval 1=成功，0=失败
*/
uint8_t Protocol_BuildVisionBarcodeData(char *code, uint8_t *out_buf, uint8_t *out_len)
{
    uint8_t len = 0;
    if (code != NULL) {
        len = strlen(code);
        if (len > PROTOCOL_DATA_MAX_LEN) len = PROTOCOL_DATA_MAX_LEN;
    }
    return build_frame(FRAME_HEADER_VISION, CMD_VISION_BARCODE_DATA, (uint8_t*)code, len, out_buf, out_len);
}

/*
*brief 封装视觉搜索超时信号（H→F）
*param out_buf 输出缓冲区
*param out_len 输出长度（指针）
*retval 1=成功，0=失败
*note  触发控制板故障上报机制
*/
uint8_t Protocol_BuildVisionSearchTimeout(uint8_t *out_buf, uint8_t *out_len)
{
    return build_frame(FRAME_HEADER_VISION, CMD_VISION_SEARCH_TIMEOUT, NULL, 0, out_buf, out_len);
}

// ----- 控制板 → HMI（状态事件）-----

/*
*brief 封装"已取到包裹"信号（F→HMI）
*param out_buf 输出缓冲区
*param out_len 输出长度（指针）
*retval 1=成功，0=失败
*note  HMI收到后自行切换状态为"已取到包裹"
*/
uint8_t Protocol_BuildHMIPicked(uint8_t *out_buf, uint8_t *out_len)
{
    return build_frame(FRAME_HEADER_HMI, CMD_HMI_PICKED, NULL, 0, out_buf, out_len);
}

/*
*brief 封装"已送达"信号（F→HMI）
*param out_buf 输出缓冲区
*param out_len 输出长度（指针）
*retval 1=成功，0=失败
*note  HMI收到后自行切换状态为"已送达"
*/
uint8_t Protocol_BuildHMIDelivered(uint8_t *out_buf, uint8_t *out_len)
{
    return build_frame(FRAME_HEADER_HMI, CMD_HMI_DELIVERED, NULL, 0, out_buf, out_len);
}

/*
*brief 封装故障上报帧（F→HMI）
*param error_code 故障码
*param out_buf 输出缓冲区
*param out_len 输出长度（指针）
*retval 1=成功，0=失败
*/
uint8_t Protocol_BuildHMIError(uint8_t error_code, uint8_t *out_buf, uint8_t *out_len)
{
    return build_frame(FRAME_HEADER_HMI, CMD_HMI_ERROR_REPORT, &error_code, 1, out_buf, out_len);
}

// ----- 帧解析 -----

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
*note  内部维护状态机，调用者需持续喂入数据
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
*note  在开始新一帧接收前或发生错误时调用
*/
void Protocol_ResetParser(void)
{
    parse_len = 0;
    parse_in_frame = 0;
}
