/*
*file servo_bus.c
*brief 总线舵机驱动实现（USART3，PB10，只发送）
*note  状态机：只有 STOP 状态才执行指令
*/

#include "servo_bus.h"

uint8_t servo_state = SERVO_STATE_STOP;

/*
*brief 初始化 USART3（PB10 TX）
*/
void servo_bus_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    USART_InitStruct.USART_BaudRate = bound;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStruct);

    USART_Cmd(USART3, ENABLE);
}

static void servo_send_string(char *str)
{
    while (*str) {
        while ((USART3->SR & 0X40) == 0);
        USART3->DR = (uint8_t)*str++;
    }
}

void servo_bus_set_state(uint8_t state)
{
    servo_state = state;
}

// ============================================================
//                      单舵机控制
// ============================================================

/*
*brief 控制单个舵机（PWM 方式）
*note  只有 STOP 状态才执行
*/
void servo_control(uint8_t id, uint16_t pwm, uint16_t time_ms)
{
    char cmd[32];
    if (servo_state != SERVO_STATE_STOP) return;
    if (pwm < SERVO_PWM_MIN) pwm = SERVO_PWM_MIN;
    if (pwm > SERVO_PWM_MAX) pwm = SERVO_PWM_MAX;
    if (time_ms > 9999) time_ms = 9999;
    sprintf(cmd, "#%03dP%04dT%04d!\r\n", id, pwm, time_ms);
    servo_send_string(cmd);
}

/*
*brief 控制单个舵机（角度方式，270° 模式）
*/
void servo_control_angle(uint8_t id, float angle_deg, uint16_t time_ms)
{
    uint16_t pwm;
    if (servo_state != SERVO_STATE_STOP) return;
    if (angle_deg < -135.0f) angle_deg = -135.0f;
    if (angle_deg > 135.0f) angle_deg = 135.0f;
    pwm = (uint16_t)((angle_deg + 135.0f) / 270.0f * 2000.0f + 500.0f + 0.5f);
    servo_control(id, pwm, time_ms);
}

// ============================================================
//                      多舵机同步控制
// ============================================================

/*
*brief 多舵机同步控制（PWM 方式）
*note  一次性发送多条指令，舵机同时开始动作
*/
void servo_control_multi(uint8_t *ids, uint16_t *pwms, uint16_t *times, uint8_t count)
{
    char cmd[128];
    char *ptr = cmd;
    uint8_t i;
    if (servo_state != SERVO_STATE_STOP) return;
    ptr += sprintf(ptr, "{");
    for (i = 0; i < count; i++) {
        if (pwms[i] < SERVO_PWM_MIN) pwms[i] = SERVO_PWM_MIN;
        if (pwms[i] > SERVO_PWM_MAX) pwms[i] = SERVO_PWM_MAX;
        if (times[i] > 9999) times[i] = 9999;
        ptr += sprintf(ptr, "#%03dP%04dT%04d!", ids[i], pwms[i], times[i]);
    }
    ptr += sprintf(ptr, "}\r\n");
    servo_send_string(cmd);
}

/*
*brief 多舵机同步控制（角度方式）
*/
void servo_control_multi_angle(uint8_t *ids, float *angles, uint16_t *times, uint8_t count)
{
    uint16_t pwms[8];
    uint8_t i;
    if (count > 8) count = 8;
    for (i = 0; i < count; i++) {
        if (angles[i] < -135.0f) angles[i] = -135.0f;
        if (angles[i] > 135.0f) angles[i] = 135.0f;
        pwms[i] = (uint16_t)((angles[i] + 135.0f) / 270.0f * 2000.0f + 500.0f + 0.5f);
    }
    servo_control_multi(ids, pwms, times, count);
}

// ============================================================
//                      扭力控制
// ============================================================

/*
*brief 释放单个舵机扭力（舵机松劲，可手动掰动）
*/
void servo_release_torque(uint8_t id)
{
    char cmd[16];
    sprintf(cmd, "#%03dPULK!\r\n", id);
    servo_send_string(cmd);
}

/*
*brief 恢复单个舵机扭力（舵机上锁，保持当前位置）
*/
void servo_enable_torque(uint8_t id)
{
    char cmd[16];
    sprintf(cmd, "#%03dPULR!\r\n", id);
    servo_send_string(cmd);
}

/*
*brief 释放多个舵机扭力（批量松劲）
*/
void servo_release_torque_multi(uint8_t *ids, uint8_t count)
{
    char cmd[128];
    char *ptr = cmd;
    uint8_t i;
    if (count == 0) return;
    ptr += sprintf(ptr, "{");
    for (i = 0; i < count; i++) {
        ptr += sprintf(ptr, "#%03dPULK!", ids[i]);
    }
    ptr += sprintf(ptr, "}\r\n");
    servo_send_string(cmd);
}

/*
*brief 恢复多个舵机扭力（批量上锁）
*/
void servo_enable_torque_multi(uint8_t *ids, uint8_t count)
{
    char cmd[128];
    char *ptr = cmd;
    uint8_t i;
    if (count == 0) return;
    ptr += sprintf(ptr, "{");
    for (i = 0; i < count; i++) {
        ptr += sprintf(ptr, "#%03dPULR!", ids[i]);
    }
    ptr += sprintf(ptr, "}\r\n");
    servo_send_string(cmd);
}

// ============================================================
//                      工作模式（广播）
// ============================================================

/*
*brief 广播设置所有舵机工作模式
*param mode 模式编号（1~8）
*       1：270° 顺时针   2：270° 逆时针
*       3：180° 顺时针   4：180° 逆时针
*       5：360° 定圈顺时针  6：360° 定圈逆时针
*       7：360° 定时顺时针  8：360° 定时逆时针
*note  广播模式，所有在线舵机同时生效
*/
void servo_set_mode_broadcast(uint8_t mode)
{
    char cmd[16];
    if (mode < 1) mode = 1;
    if (mode > 8) mode = 8;
    sprintf(cmd, "#255PMOD%d!\r\n", mode);
    servo_send_string(cmd);
}

// ============================================================
//                      ID 操作（广播）
// ============================================================

/*
*brief 强制修改舵机 ID（广播方式）
*note  多个舵机不能同时修改，只能一个一个单独接上修改
*/
void servo_force_id(uint8_t new_id)
{
    char cmd[20];
    if (new_id > 254) new_id = 254;
    sprintf(cmd, "#255PID%03d!\r\n", new_id);
    servo_send_string(cmd);
}
