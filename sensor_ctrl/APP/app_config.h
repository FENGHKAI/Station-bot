#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include "sys.h"
#include "protocol.h" /* ★新增：命令字唯一真值来源（V1.3），别名全部引用它 */

/* ============================================================
 * 命令字别名——★已按 protocol.h V1.3 逐条对齐，不再手写码值。
 * 原默认值（CMD_ACK=0x01 / CMD_TEST=0x00 / CMD_HANDOVER=0x10 /
 * CMD_TASK_DONE=0x12 / CMD_FAULT=0x13）与协议冲突，全部作废。
 * ============================================================ */
#ifndef CMD_TEST
#define CMD_TEST        CMD_TEST_LINK         /* 0xFF 连接测试，原样回显 */
#endif

/* CMD_ACK：protocol.h 已定义 = 0x00（空数据体ACK），此处不再重复定义 */

#ifndef CMD_HANDOVER
#define CMD_HANDOVER    CMD_VISION_HANDOVER   /* 0x01 F板→OpenMV 交权 */
#endif

#ifndef CMD_START_TASK
#define CMD_START_TASK  CMD_HMI_START         /* 0x11 HMI→F 启动，data[0]=货架号 */
#endif

#ifndef CMD_TASK_DONE
#define CMD_TASK_DONE   CMD_HMI_DELIVERED     /* 0x14 F→HMI 已送达（HMI需回ACK） */
#endif

#ifndef CMD_FAULT
#define CMD_FAULT       CMD_HMI_ERROR_REPORT  /* 0x15 F→HMI 故障上报（HMI需回ACK） */
#endif

#ifndef CMD_RESUME
#define CMD_RESUME      0x16 /* ★protocol.h 未定义（0x11~0x15已用满，0x16空闲），
                                  需与 ESP32-S3 端书面约定后固化进 protocol.h */
#endif

#ifndef CMD_GRAB_DONE
#define CMD_GRAB_DONE   CMD_VISION_GRAB_DONE  /* 0x20 OpenMV→F 抓取完成 */
#endif

#ifndef CMD_PLACE_DONE
#define CMD_PLACE_DONE  CMD_VISION_PLACE_DONE /* 0x21 OpenMV→F 放置完成 */
#endif

/* ---- 故障码（数值本体用 protocol.h 的 HMI_ERROR_xxx）----
 * FAULT_BLOCKED / 电池 / 看门狗 protocol.h 只定义到 0x07：
 * 0x08 前方受阻、0x08之后需与 ESP32-S3 端同步添加后固化进 protocol.h */
#ifndef FAULT_BLOCKED
#define FAULT_BLOCKED 0x08
#endif

/* ================= 任务与周期 ================= */
#define CTRL_TASK_PERIOD_MS 20   /* 控制任务周期（50Hz） */
#define OBST_TASK_PERIOD_MS 200  /* 避障检测周期 */
#define BATT_TASK_PERIOD_MS 500
#define MISSION_WATCHDOG_MS 60000UL /* 任务总看门狗 */
#define GRAB_TIMEOUT_MS     60000UL /* 等视觉完成的超时 */

/* ================= 链路参数 ================= */
#define ACK_TIMEOUT_VISION_MS 300   /* USART6 有线 */
#define ACK_TIMEOUT_HMI_MS    500   /* USART2 经 WiFi */
#define ACK_RETRY_TIMES 3
#define UART_VISION_BAUD 115200
#define UART_HMI_BAUD    115200

/* ================= 导航参数 ================= */
#define TRACK_BASE_SPEED 0.25f   /* 循迹基速 m/s */
#define TURN_TOL_DEG     3.0f    /* IMU 转向容差 */
#define TURN_OK_SAMPLES  5       /* 容差带内连续次数 */
#define TURN_TIMEOUT_CYC 150     /* 3s 转向超时 */
#define SPIN_PWM         400     /* 开环旋转占空比（周期1000） */
#define FINDLINE_SPEED   0.08f   /* 转弯后找线速度 */
#define FINDLINE_MAX_MM  150.0f  /* 找线最大距离，超距报故障 */

/* graph_config.h 边权 80~95 按 cm 计 → ×10 得 mm；
 * 若边权本身是 mm，改成 1.0f */
#define EDGE_WEIGHT_TO_MM 10.0f

/* Motor_GetOdometer() 返回单位为 m（motor.c 已核实：odometer += m/s*0.01s） */
#define ODOM_TO_MM 1000.0f

/* ================= 避障参数 ================= */
#define OBSTACLE_TH_CM     30.0f
#define OBSTACLE_CONFIRM_CNT 3  /* 连续3次<阈值判受阻 */
#define OBSTACLE_RECOVER_CNT 5  /* 恢复：连续5次 */
#define OBSTACLE_RECOVER_INT_MS OBST_TASK_PERIOD_MS

/* ================= 电池阈值（3S 锂电，与 adc_battery.h 一致）================= */
#define BATT_WARN_V     10.2f   /* 蜂鸣提醒 = BATTERY_WARN_THRESHOLD */
#define BATT_CRITICAL_V 9.9f    /* 强制停车 = BATTERY_STOP_THRESHOLD */

#endif

