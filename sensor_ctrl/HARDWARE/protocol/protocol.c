/*
 *file protocol.c
 *brief 统一通信协议底层实现（控制板F板专用）
 *note 版本：V1.3
 *     组帧：转义编码 → 帧头+长度+命令字+数据体 → 校验和(转义后) → 帧尾 → 串口发出
 *     解析：状态机收帧 → 校验和验证 → 转义解码
 *     双串口各持独立解析上下文；串口发送通过注册的函数指针调用；
 *     ACK重传/业务状态机由上层实现
 */
#include "protocol.h"

// ----- 发送接口保存（static，仅本文件使用）-----
static ProtocolSendFunc_t send_hmi    = NULL;
static ProtocolSendFunc_t send_vision = NULL;

// ============================================================
// 转义编码（V1.3：0xA5与0x5A独立转义码，无损还原）
// ============================================================

/*
 *brief 转义编码
 *param src 原始数据体  src_len 原始长度
 *param dst 输出缓冲区(转义后)  dst_len 输出长度(指针)
 *note 规则：0xA5→0x7D 0x01, 0x5A→0x7D 0x04,
 *          0x0D→0x7D 0x02, 0x0A→0x7D 0x03, 0x7D→0x7D 0x00
 */
static void escape_encode(uint8_t *src, uint8_t src_len, uint8_t *dst, uint8_t *dst_len)
{
    uint8_t i;

    *dst_len = 0;
    for (i = 0; i < src_len; i++) {
        switch (src[i]) {
            case FRAME_HEADER_VISION:                   /* 0xA5 */
                dst[(*dst_len)++] = FRAME_ESC;
                dst[(*dst_len)++] = 0x01;
                break;
            case FRAME_HEADER_HMI:                      /* 0x5A */
                dst[(*dst_len)++] = FRAME_ESC;
                dst[(*dst_len)++] = 0x04;
                break;
            case FRAME_TAIL1:                           /* 0x0D */
                dst[(*dst_len)++] = FRAME_ESC;
                dst[(*dst_len)++] = 0x02;
                break;
            case FRAME_TAIL2:                           /* 0x0A */
                dst[(*dst_len)++] = FRAME_ESC;
                dst[(*dst_len)++] = 0x03;
                break;
            case FRAME_ESC:                             /* 0x7D */
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
 *param src 转义后数据体  src_len 长度
 *param dst 输出缓冲区(原始数据)  dst_len 输出长度(指针)
 *retval 1=成功，0=失败(非法转义序列)
 *note V1.3转义表与帧头无关，解码无需上下文
 */
static uint8_t escape_decode(uint8_t *src, uint8_t src_len, uint8_t *dst, uint8_t *dst_len)
{
    uint8_t i = 0;

    *dst_len = 0;
    while (i < src_len) {
        if (src[i] == FRAME_ESC) {
            if (i + 1 >= src_len) return 0;             /* 转义符在末尾，残帧 */
            switch (src[i + 1]) {
                case 0x01: dst[(*dst_len)++] = FRAME_HEADER_VISION; break;
                case 0x04: dst[(*dst_len)++] = FRAME_HEADER_HMI;    break;
                case 0x02: dst[(*dst_len)++] = FRAME_TAIL1;         break;
                case 0x03: dst[(*dst_len)++] = FRAME_TAIL2;         break;
                case 0x00: dst[(*dst_len)++] = FRAME_ESC;           break;
                default:   return 0;                        /* 非法转义码 */
            }
            i += 2;
        } else {
            dst[(*dst_len)++] = src[i++];
        }
    }
    return 1;
}

// ============================================================
// 校验和
// ============================================================

/*
 *brief 计算校验和（所有字节累加，取低8位）
 *note 覆盖范围：长度字节+命令字+转义后数据体（不含帧头/校验/帧尾）
 */
static uint8_t calc_checksum(uint8_t *buf, uint8_t len)
{
    uint8_t i;
    uint8_t sum = 0;

    for (i = 0; i < len; i++) {
        sum += buf[i];
    }
    return sum;
}

// ============================================================
// 组帧核心（static，对外只暴露Send）
// ============================================================

/*
 *brief 内部组帧
 *param header 帧头(0xA5或0x5A)  cmd 命令字
 *param data 原始数据体(可为NULL)  data_len 原始长度
 *param out_buf 输出缓冲区  out_len 输出长度(指针)
 *retval 1=成功，0=失败
 */
static uint8_t build_frame(uint8_t header, uint8_t cmd, const uint8_t *data,
                           uint8_t data_len, uint8_t *out_buf, uint8_t *out_len)
{
    uint8_t escaped[PROTOCOL_DATA_MAX_LEN * 2 + 4];
    uint8_t escaped_len = 0;
    uint8_t len = 0;
    uint8_t i;
    uint8_t checksum;

    if (out_buf == NULL || out_len == NULL) return 0;
    if (data_len > PROTOCOL_DATA_MAX_LEN)   return 0;
    if (data == NULL && data_len > 0)       return 0;

    if (data_len > 0) {
        escape_encode((uint8_t *)data, data_len, escaped, &escaped_len);
    }

    if (escaped_len + 6 > PROTOCOL_FRAME_MAX_LEN) return 0;

    out_buf[len++] = header;
    out_buf[len++] = data_len;              /* 长度字段=转义前数据体长度 */
    out_buf[len++] = cmd;
    for (i = 0; i < escaped_len; i++) {
        out_buf[len++] = escaped[i];
    }
    /* 校验和覆盖：长度+命令字+转义后数据体 */
    checksum = calc_checksum(&out_buf[1], len - 1);
    out_buf[len++] = checksum;
    out_buf[len++] = FRAME_TAIL1;
    out_buf[len++] = FRAME_TAIL2;

    *out_len = len;
    return 1;
}

// ============================================================
// 帧解析核心
// ============================================================

/*
 *brief 内部解析完整帧
 *param buf 完整帧(帧头到帧尾)  len 帧长  result 输出解析结果
 *retval 1=成功，0=失败(格式/校验/解码错误)
 */
static uint8_t parse_frame_internal(uint8_t *buf, uint8_t len, ProtocolResult_t *result)
{
    uint8_t header, data_len, cmd;
    uint8_t checksum_recv, checksum_calc;
    uint8_t raw_data[PROTOCOL_DATA_MAX_LEN];
    uint8_t raw_len = 0;
    uint8_t escaped_len;

    if (buf == NULL || result == NULL) return 0;
    if (len < 6)                       return 0;            /* 最小帧长6B */

    /* 帧头校验 */
    header = buf[0];
    if (header != FRAME_HEADER_VISION && header != FRAME_HEADER_HMI) return 0;

    /* 帧尾校验 */
    if (buf[len - 2] != FRAME_TAIL1 || buf[len - 1] != FRAME_TAIL2)  return 0;

    data_len = buf[1];
    cmd      = buf[2];
    escaped_len = len - 6;                                  /* 总长-头3-校验1-尾2 */

    /* 转义后数据体长度合法性 */
    if (escaped_len > PROTOCOL_DATA_MAX_LEN * 2 + 4) return 0;

    /* 校验和验证（覆盖长度+命令字+转义后数据体） */
    checksum_recv = buf[len - 3];
    checksum_calc = calc_checksum(&buf[1], len - 4);
    if (checksum_recv != checksum_calc) return 0;

    /* 转义解码 */
    if (escaped_len > 0) {
        if (!escape_decode(&buf[3], escaped_len, raw_data, &raw_len)) return 0;
        /* 解码后长度必须与长度字段一致（抗干扰二次校验） */
        if (raw_len != data_len) return 0;
    } else {
        if (data_len != 0) return 0;                        /* 无数据体但长度字段非0 */
    }

    /* 填充结果 */
    result->header   = header;
    result->cmd      = cmd;
    result->data_len = raw_len;
    if (raw_len > 0) {
        memcpy(result->data, raw_data, raw_len);
    }
    result->valid = 1;
    return 1;
}

// ============================================================
// 公共接口
// ============================================================

/*
 *brief 协议模块初始化
 *note 解析上下文由调用者为每路串口各自声明并用Protocol_ParserReset初始化；
 *     发送接口需在首次发送前通过Register函数注册
 */
void Protocol_Init(void)
{
    send_hmi    = NULL;
    send_vision = NULL;
}

/*
 *brief 注册HMI链路发送接口（USART2）
 *param func 串口驱动提供的发送函数，如usart2_send_bytes
 *note 若驱动函数第二参数为uint16_t，请包一层wrapper后注册
 */
void Protocol_RegisterSendHMI(ProtocolSendFunc_t func)
{
    send_hmi = func;
}

/*
 *brief 注册视觉链路发送接口（USART6）
 *param func 串口驱动提供的发送函数，如usart6_send_bytes
 */
void Protocol_RegisterSendVision(ProtocolSendFunc_t func)
{
    send_vision = func;
}

/*
 *brief 组帧并发送（对外唯一发送入口）
 *param header 帧头(0xA5=发视觉，0x5A=发HMI)  cmd 命令字
 *param data 原始数据体(转义前，可为NULL)  data_len 原始长度
 *retval 1=发送成功，0=失败(未注册接口/组帧失败/帧头非法)
 *note 转义和校验和由本函数自动处理，按header自动路由到对应串口；
 *     发送为阻塞式(依赖底层串口发送函数)，ACK重传由上层实现
 */
uint8_t Protocol_Send(uint8_t header, uint8_t cmd,
                      const uint8_t *data, uint8_t data_len)
{
    uint8_t buf[PROTOCOL_FRAME_MAX_LEN];
    uint8_t len = 0;

    if (!build_frame(header, cmd, data, data_len, buf, &len)) {
        return 0;
    }

    if (header == FRAME_HEADER_HMI) {
        if (send_hmi == NULL) return 0;          /* 未注册，拒绝发送 */
        send_hmi(buf, len);
    } else if (header == FRAME_HEADER_VISION) {
        if (send_vision == NULL) return 0;
        send_vision(buf, len);
    } else {
        return 0;
    }
    return 1;
}

/*
 *brief ACK快捷发送（最高频操作，独立封装）
 *param header 对方帧头：回HMI用FRAME_HEADER_HMI，回视觉用FRAME_HEADER_VISION
 *note 等价于 Protocol_Send(header, CMD_ACK, NULL, 0)
 */
uint8_t Protocol_SendAck(uint8_t header)
{
    return Protocol_Send(header, CMD_ACK, NULL, 0);
}

/*
 *brief 逐字节解析（串口接收回调中逐字节喂入）
 *param p 该路串口的解析上下文（USART2/USART6各一个实例）
 *param data 输入字节  result 输出解析结果(解析到完整帧时填充)
 *retval 1=解析到完整帧，0=尚未完成
 *note 用法示例：
 *     ProtocolParser_t hmi_parser;                  // 全局定义
 *     Protocol_ParserReset(&hmi_parser);            // 初始化时调用
 *     // USART2回调中：
 *     if (Protocol_ParseByte(&hmi_parser, data, &result)) { 处理result; }
 */
uint8_t Protocol_ParseByte(ProtocolParser_t *p, uint8_t data, ProtocolResult_t *result)
{
    if (p == NULL || result == NULL) return 0;

    if (!p->in_frame) {
        /* 等待帧头 */
        if (data == FRAME_HEADER_VISION || data == FRAME_HEADER_HMI) {
            p->buf[0]   = data;
            p->len      = 1;
            p->in_frame = 1;
        }
        return 0;
    }

    p->buf[p->len++] = data;

    /* 扫描帧尾 0x0D 0x0A（数据体已转义，不会出现假帧尾） */
    if (p->len >= 2 &&
        p->buf[p->len - 2] == FRAME_TAIL1 &&
        p->buf[p->len - 1] == FRAME_TAIL2) {
        uint8_t ret = parse_frame_internal(p->buf, p->len, result);
        p->len      = 0;
        p->in_frame = 0;
        return ret;
    }

    /* 超长保护：残帧丢弃 */
    if (p->len >= PROTOCOL_FRAME_MAX_LEN) {
        p->len      = 0;
        p->in_frame = 0;
    }
    return 0;
}

/*
 *brief 解析完整帧（非中断场景直接解析一块缓冲）
 *retval 1=成功，0=失败
 */
uint8_t Protocol_ParseFrame(uint8_t *buf, uint8_t len, ProtocolResult_t *result)
{
    return parse_frame_internal(buf, len, result);
}

/*
 *brief 重置指定路的解析状态机（残帧/错误后调用）
 */
void Protocol_ParserReset(ProtocolParser_t *p)
{
    if (p == NULL) return;
    p->len      = 0;
    p->in_frame = 0;
}
