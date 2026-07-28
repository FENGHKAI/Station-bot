/*
*file servo_bus.h
*brief 总线舵机驱动声明（USART3，PB10，只发送）
*note  状态机控制：只有 STOP 状态才能执行舵机指令
*/

#ifndef __SERVO_BUS_H
#define __SERVO_BUS_H

#include "sys.h"

// 舵机状态
extern uint8_t servo_state;
#define SERVO_STATE_MOVING   0
#define SERVO_STATE_STOP     1

// PWM 范围（270° 舵机）
#define SERVO_PWM_MIN        500
#define SERVO_PWM_MID        1500
#define SERVO_PWM_MAX        2500

// 工作模式（用于 servo_set_mode_broadcast）
#define SERVO_MODE_270_CW    1   // 270° 顺时针
#define SERVO_MODE_270_CCW   2   // 270° 逆时针
#define SERVO_MODE_180_CW    3   // 180° 顺时针
#define SERVO_MODE_180_CCW   4   // 180° 逆时针
#define SERVO_MODE_360_CW    5   // 360° 定圈顺时针
#define SERVO_MODE_360_CCW   6   // 360° 定圈逆时针
#define SERVO_MODE_TIMER_CW  7   // 360° 定时顺时针
#define SERVO_MODE_TIMER_CCW 8   // 360° 定时逆时针

// ----- 初始化和状态 -----
void servo_bus_init(u32 bound);
void servo_bus_set_state(uint8_t state);

// ----- 单舵机控制 -----
void servo_control(uint8_t id, uint16_t pwm, uint16_t time_ms);
void servo_control_angle(uint8_t id, float angle_deg, uint16_t time_ms);

// ----- 多舵机同步控制 -----
void servo_control_multi(uint8_t *ids, uint16_t *pwms, uint16_t *times, uint8_t count);
void servo_control_multi_angle(uint8_t *ids, float *angles, uint16_t *times, uint8_t count);

// ----- 扭力控制 -----
void servo_release_torque(uint8_t id);
void servo_enable_torque(uint8_t id);
void servo_release_torque_multi(uint8_t *ids, uint8_t count);
void servo_enable_torque_multi(uint8_t *ids, uint8_t count);

// ----- 工作模式（广播）-----
void servo_set_mode_broadcast(uint8_t mode);

// ----- ID 操作（广播）-----
void servo_force_id(uint8_t new_id);

#endif
