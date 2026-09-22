/* ================ app_comm.c ================
 * 通信任务：字节队列 → 帧解析 → 状态门控分发
 * 帧格式（V1.3，与 protocol.c 严格一致）：
 * HEAD(0xA5/0x5A) | LEN | CMD | DATA(n) | SUM | 0x0D 0x0A
 * LEN = 转义前 DATA 体长度（不含 CMD）★V1.3口径，已向 protocol.c 对齐
 * SUM = LEN + CMD + 转义后DATA 的累加和（8bit）
 * 转义 = 仅作用于 DATA 体，0x7D 前缀
 * ============================================================ */
#include "app_comm.h"
#include "app_config.h"
#include "app_state.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include <string.h>

/* ---------------- 事件位（ControlTask 消费） ---------------- */
#define EV_ACK        (1 << 0)
#define EV_START      (1 << 1)
#define EV_VISION_OK  (1 << 2)   /* 收到 0x20/0x21 */
#define EV_VISION_FAIL (1 << 3)  /* 收到视觉故障 0x03 */
#define EV_RESUME     (1 << 4)

EventGroupHandle_t g_comm_events = NULL;

/* 启动任务的货架号（ControlTask 读取） */
volatile uint8_t g_shelf_id = 1;

/* ---------------- 转义表（V1.3，与 protocol.c 逐字节一致）----------------
 * 0xA5→0x7D 0x01   0x5A→0x7D 0x04   0x0D→0x7D 0x02
 * 0x0A→0x7D 0x03   0x7D→0x7D 0x00
 * 三端口径：本表 = protocol.c = uart_comm.py，必须逐字节一致。
 * ★修改：原占位表 {0xA5,0x86}... 全部作废；
 * ★修改：0x7D 的转义后缀是 0x00，不能再用"返回值是否为0"判断是否转义，
 *   改为 esc_need() 显式判断 + esc_suffix() 取后缀。 */
static uint8_t esc_need(uint8_t b)
{
    return (b == 0xA5 || b == 0x5A || b == 0x0D || b == 0x0A || b == 0x7D);
}

static uint8_t esc_suffix(uint8_t b)
{
    switch (b) {
    case 0xA5: return 0x01;
    case 0x5A: return 0x04;
    case 0x0D: return 0x02;
    case 0x0A: return 0x03;
    case 0x7D: return 0x00;
    default:   return 0;
    }
}

static uint8_t esc_decode(uint8_t b)
{
    switch (b) {
    case 0x00: return 0x7D;
    case 0x01: return 0xA5;
    case 0x02: return 0x0D;
    case 0x03: return 0x0A;
    case 0x04: return 0x5A;
    default:   return b; /* 非法转义码容错 */
    }
}

#define FRAME_DATA_MAX 32

typedef struct {
    uint8_t state, head, len, cmd, esc;
    uint8_t data[FRAME_DATA_MAX];
    uint8_t dec_cnt, need;
    uint8_t sum;
} FrameParser_t;

enum { ST_HEAD, ST_LEN, ST_CMD, ST_DATA, ST_SUM, ST_CR, ST_LF };

static QueueHandle_t q_u2, q_u6, q_set;
static SemaphoreHandle_t tx_mutex;
static FrameParser_t parser_hmi, parser_vis;
static volatile uint8_t pending_ack_cmd = 0;

/* ============ ISR 回调：只做入队，解析放任务里 ============ */
static void rx2_cb(uint8_t d)
{
    BaseType_t hpw = pdFALSE;
    xQueueSendFromISR(q_u2, &d, &hpw);
    portYIELD_FROM_ISR(hpw);
}

static void rx6_cb(uint8_t d)
{
    BaseType_t hpw = pdFALSE;
    xQueueSendFromISR(q_u6, &d, &hpw);
    portYIELD_FROM_ISR(hpw);
}

/* ============ 帧解析（每字节驱动） ============ */
static uint8_t parse_byte(FrameParser_t *p, uint8_t b, uint8_t *cmd, uint8_t *data, uint8_t *dlen)
{
    switch (p->state) {
    case ST_HEAD:
        if (b == 0xA5 || b == 0x5A) { p->head = b; p->state = ST_LEN; }
        break;

    case ST_LEN:
        /* ★修改：V1.3 LEN = 转义前DATA体长度（不含CMD），可为0 */
        if (b > FRAME_DATA_MAX) { p->state = ST_HEAD; break; }
        p->len = b;
        p->sum = b;
        p->state = ST_CMD;
        break;

    case ST_CMD:
        p->cmd = b;
        p->sum += b;
        p->need = p->len;      /* ★修改：需要接收的就是 LEN 个字节 */
        p->dec_cnt = 0;
        p->esc = 0;
        p->state = p->need ? ST_DATA : ST_SUM;
        break;

    case ST_DATA:
        /* 转义感知解码，SUM 累加线上原始字节（转义后）——与V1.3校验口径一致 */
        p->sum += b;
        if (p->esc) {
            p->data[p->dec_cnt++] = esc_decode(b);
            p->esc = 0;
        } else if (b == 0x7D) {
            p->esc = 1;
        } else {
            p->data[p->dec_cnt++] = b;
        }
        if (p->dec_cnt >= p->need) p->state = ST_SUM;
        break;

    case ST_SUM:
        if (b == p->sum) p->state = ST_CR;
        else             p->state = ST_HEAD;
        break;

    case ST_CR:
        p->state = (b == 0x0D) ? ST_LF : ST_HEAD;
        break;

    case ST_LF:
        p->state = ST_HEAD;
        if (b == 0x0A) {
            *cmd = p->cmd;
            memcpy(data, p->data, p->dec_cnt);
            *dlen = p->dec_cnt;
            return 1;
        }
        break;
    }
    return 0;
}

/* ============ 发送 ============ */
static void link_send_bytes(uint8_t is_hmi, const uint8_t *buf, uint16_t len)
{
    if (is_hmi) usart2_send_bytes((uint8_t *)buf, len);
    else        usart6_send_bytes((uint8_t *)buf, len);
}

/* 组帧并发送：HEAD|LEN|CMD|DATA(转义)|SUM|0D0A */
static void send_frame(uint8_t is_hmi, uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t buf[8 + FRAME_DATA_MAX * 2];
    uint8_t esc[FRAME_DATA_MAX * 2];
    uint16_t esc_len = 0, pos = 0, i;
    uint8_t sum;

    for (i = 0; i < len; i++) {  /* 先转义 */
        if (esc_need(data[i])) {                       /* ★修改：显式判断 */
            esc[esc_len++] = 0x7D;
            esc[esc_len++] = esc_suffix(data[i]);
        } else {
            esc[esc_len++] = data[i];
        }
    }

    sum = (uint8_t)len + cmd;    /* ★修改：V1.3 = LEN(仅DATA) + CMD */
    for (i = 0; i < esc_len; i++) sum += esc[i];  /* + 转义后 DATA */

    buf[pos++] = is_hmi ? 0x5A : 0xA5;
    buf[pos++] = len;            /* ★修改：LEN = 转义前DATA体长度 */
    buf[pos++] = cmd;
    memcpy(&buf[pos], esc, esc_len);
    pos += esc_len;
    buf[pos++] = sum;
    buf[pos++] = 0x0D;
    buf[pos++] = 0x0A;

    xSemaphoreTake(tx_mutex, portMAX_DELAY);  /* 多任务发送互斥 */
    link_send_bytes(is_hmi, buf, pos);
    xSemaphoreGive(tx_mutex);
}

uint8_t comm_send_ack_vision(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t r;
    for (r = 0; r < ACK_RETRY_TIMES; r++) {
        xEventGroupClearBits(g_comm_events, EV_ACK);
        pending_ack_cmd = cmd;  /* 分发线程据此匹配 ACK */
        send_frame(0, cmd, data, len);
        if (xEventGroupWaitBits(g_comm_events, EV_ACK, pdTRUE, pdFALSE,
                                ACK_TIMEOUT_VISION_MS / portTICK_PERIOD_MS) & EV_ACK)
            return 0;
    }
    return 1;  /* 3 次无 ACK → 上层置通信故障 */
}

void comm_send_hmi(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    send_frame(1, cmd, data, len);
}

/* ============ 状态门控分发 ============ */
static void dispatch_frame(uint8_t is_hmi, uint8_t cmd, uint8_t *data, uint8_t dlen)
{
    /* ---- 全局命令（任何状态）---- */
    if (cmd == CMD_ACK) {
        /* ★修改：V1.3 的 ACK（Protocol_SendAck）是空数据体；
           同时兼容"带被确认命令字"的格式。pending_ack_cmd==0 表示本方未在等待 */
        if (pending_ack_cmd != 0 &&
            (dlen == 0 || (dlen >= 1 && data[0] == pending_ack_cmd))) {
            xEventGroupSetBits(g_comm_events, EV_ACK);
            pending_ack_cmd = 0;
        }
        return;
    }

    if (cmd == CMD_TEST) {  /* 连接测试：原样回显 */
        send_frame(is_hmi, cmd, data, dlen);
        return;
    }

    if (cmd == CMD_FAULT) {  /* 视觉上报故障（0x03 搜索失败） */
        if (!is_hmi && dlen >= 1 && data[0] == 0x03) {
            xEventGroupSetBits(g_comm_events, EV_VISION_FAIL);
            send_frame(0, CMD_ACK, NULL, 0);  /* ★新增：V1.3 收到命令回ACK */
        }
        return;
    }

    if (!is_hmi) {  /* ---- 视觉链路（F板为持权方）---- */
        if (cmd == CMD_GRAB_DONE || cmd == CMD_PLACE_DONE) {
            xEventGroupSetBits(g_comm_events, EV_VISION_OK);
            send_frame(0, CMD_ACK, NULL, 0);  /* ★新增：V1.3 收到命令回ACK */
        }
        /* 门控：GRAB/DROP 之外收到 OpenMV 任何其他命令一律丢弃（非持权态） */
        return;
    }

    /* ---- HMI 链路：F 板为被指挥方，只接受持权命令 ---- */
    if (cmd == CMD_START_TASK && dlen >= 1) {
        if (app_state() == APP_STANDBY) {  /* 门控：仅待命态可启动 */
            g_shelf_id = data[0];
            xEventGroupSetBits(g_comm_events, EV_START);
            send_frame(1, CMD_ACK, NULL, 0);  /* ★新增：V1.3 收到命令回ACK */
        }
        /* 非待命态收到启动指令：忽略（可扩展为回 NAK） */
        return;
    }

    if (cmd == CMD_RESUME) {
        xEventGroupSetBits(g_comm_events, EV_RESUME);
        send_frame(1, CMD_ACK, NULL, 0);      /* ★新增：V1.3 收到命令回ACK */
        return;
    }

    /* 其余一律丢弃 */
}

/* ============ 通信任务 ============ */
void app_comm_task(void *arg)
{
    (void)arg;
    uint8_t byte, cmd, dlen, data[FRAME_DATA_MAX];

    for (;;) {
        QueueSetMemberHandle_t m = xQueueSelectFromSet(q_set, portMAX_DELAY);
        if (m == NULL) continue;

        while (xQueueReceive(m, &byte, 0) == pdTRUE) {
            FrameParser_t *p = (m == q_u2) ? &parser_hmi : &parser_vis;
            if (parse_byte(p, byte, &cmd, data, &dlen))
                dispatch_frame(m == q_u2, cmd, data, dlen);
        }
    }
}

void app_comm_init(void)
{
    q_u2 = xQueueCreate(128, sizeof(uint8_t));
    q_u6 = xQueueCreate(128, sizeof(uint8_t));
    q_set = xQueueCreateSet(256);
    xQueueAddToSet(q_u2, q_set);
    xQueueAddToSet(q_u6, q_set);

    tx_mutex = xSemaphoreCreateMutex();
    g_comm_events = xEventGroupCreate();

    usart2_init(UART_HMI_BAUD, rx2_cb);     /* ESP8266 → ESP32-S3 */
    usart6_init(UART_VISION_BAUD, rx6_cb);  /* OpenMV 直连 */
}
