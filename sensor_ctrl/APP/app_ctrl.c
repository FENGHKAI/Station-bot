/* ================ app_ctrl.c ================ */
#include "app_config.h"
#include "app_state.h"
#include "app_comm.h"
#include "app_nav.h"
#include "path_planner.h"
#include "ultrasonic.h"
#include "motor.h"
#include "protocol.h"     /* ★新增：HMI_ERROR_xxx 故障码 */
#include "buzzer.h"       /* ★新增：Buzzer_On/Off */
#include "adc_battery.h"  /* ★新增：ADC_GetBatteryVoltage */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include <stdio.h>

#define EV_START      (1 << 1)
#define EV_VISION_OK  (1 << 2)
#define EV_VISION_FAIL (1 << 3)
#define EV_RESUME     (1 << 4)

/* protocol.h 未定义的故障码（★需与HMI端同步约定后写入protocol.h） */
#define FAULT_BATTERY_LOW 0x08  /* 电池电量耗尽停车 */
#define FAULT_WATCHDOG    0x09  /* 任务看门狗：60s无进展 */

extern EventGroupHandle_t g_comm_events;
extern volatile uint8_t g_shelf_id;   /* ★补回：定义在app_comm.c，清理假声明时误删 */

static Path_t m_path;
static uint8_t m_shelf_node_idx;
static uint8_t m_blocked_reported = 0;
static uint8_t m_batt_critical = 0;

/* ---- 故障上报 → HMI，并复位待命 ---- */
static void report_fault_and_reset(uint8_t code)
{
    comm_send_hmi(CMD_FAULT, &code, 1);
    app_enter(APP_STANDBY);
}

/* ---- 按键 PD4（demo：待命态按键=直接取货架1）---- */
static void key_scan(void)
{
    static uint8_t cnt = 0;
    if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4) == 0) {
        if (++cnt == 10 && app_state() == APP_STANDBY) {  /* 200ms 消抖 */
            g_shelf_id = 1;
            xEventGroupSetBits(g_comm_events, EV_START);
        }
    } else cnt = 0;
}

/* ---- LED PB6 心跳 ---- */
static void led_heartbeat(void)
{
    static uint16_t div = 0;
    if (++div >= 25) {
        div = 0;
        GPIO_WriteBit(GPIOB, GPIO_Pin_6,
                      (BitAction)!GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_6));
    }
}

/* ================= 控制任务（20ms） ================= */
void app_ctrl_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    EventBits_t ev;
    uint8_t i;

    for (;;) {
        vTaskDelayUntil(&last, CTRL_TASK_PERIOD_MS / portTICK_PERIOD_MS);
        key_scan();
        led_heartbeat();

        ev = xEventGroupGetBits(g_comm_events);

        switch (app_state()) {
        case APP_STANDBY:
            m_blocked_reported = 0;
            if (ev & EV_START) {
                xEventGroupClearBits(g_comm_events, EV_START);
                app_enter(APP_PLAN);
            }
            break;

        case APP_PLAN: {
            /* 规划 0→货架→0 的往返路径 */
            if (!PathPlanner_PlanToShelf(g_shelf_id, &m_path)) {
                report_fault_and_reset(HMI_ERROR_PATH_FAIL);  /* ★0x06，原0x05与协议不符 */
                break;
            }

            m_shelf_node_idx = 0xFF;
            {   /* 正向找第一个匹配货架节点的位置 */
                uint8_t target;
                PathPlanner_FindShelf(g_shelf_id, &target);
                for (i = 0; i < m_path.len; i++)
                    if (m_path.nodes[i] == target) { m_shelf_node_idx = i; break; }
            }

            if (m_shelf_node_idx == 0xFF || !nav_start(&m_path, 0, m_shelf_node_idx)) {
                report_fault_and_reset(HMI_ERROR_PATH_FAIL);
                break;
            }
            app_enter(APP_RUN_OUT);
            break;
        }

        case APP_RUN_OUT:
        case APP_RUN_BACK: {
            NavResult_t r = nav_step();
            /* ★修改：删除了原空 if 占位段（只有注释、无实际作用） */

            if (r == NAV_ARRIVED) {
                app_enter(app_state() == APP_RUN_OUT ? APP_GRAB : APP_DROP);
            } else if (r == NAV_FAIL) {
                report_fault_and_reset(HMI_ERROR_PATH_FAIL);  /* ★导航执行失败归入路径类 */
            }

            if (ev & EV_RESUME) {  /* 移障完成 → 原地续跑 */
                xEventGroupClearBits(g_comm_events, EV_RESUME);
                nav_resume();
                m_blocked_reported = 0;
            }
            break;
        }

        case APP_GRAB:
        case APP_DROP: {
            static uint8_t sent = 0;
            if (!sent) {  /* 进入态单次交权 */
                if (comm_send_ack_vision(CMD_HANDOVER, NULL, 0)) {
                    sent = 0;
                    report_fault_and_reset(HMI_ERROR_UART_TIMEOUT);  /* 3次无ACK=通信故障 */
                    break;
                }
                sent = 1;
            }

            if (ev & EV_VISION_FAIL) {  /* 视觉搜索失败 */
                xEventGroupClearBits(g_comm_events, EV_VISION_FAIL);
                sent = 0;
                report_fault_and_reset(HMI_ERROR_VISION_TIMEOUT);
                break;
            }

            if (ev & EV_VISION_OK) {  /* 0x20 抓取完 / 0x21 放置完 */
                xEventGroupClearBits(g_comm_events, EV_VISION_OK);
                sent = 0;
                if (app_state() == APP_GRAB) {
                    nav_start(&m_path, m_shelf_node_idx, m_path.len - 1);
                    app_enter(APP_RUN_BACK);
                } else {
                    comm_send_hmi(CMD_TASK_DONE, NULL, 0);  /* 任务完成 */
                    app_enter(APP_STANDBY);
                }
                break;
            }

            if (app_state_elapsed_ms() > GRAB_TIMEOUT_MS) {
                sent = 0;
                report_fault_and_reset(HMI_ERROR_UART_TIMEOUT);
            }
            break;
        }

        case APP_PLAN_BACK:  /* 已并入 RUN_BACK 入口，防御性保留 */
            app_enter(APP_RUN_BACK);
            break;
        }

        /* 任务级看门狗：任何非待命态 60s 无进展即复位 */
        if (app_state() != APP_STANDBY && app_state_elapsed_ms() > MISSION_WATCHDOG_MS) {
            report_fault_and_reset(FAULT_WATCHDOG);
        }

        /* 电池耗尽保护 */
        if (m_batt_critical && app_state() != APP_STANDBY) {
            Motor_AllStop();  /* ★修改：目标清零+开环清零，兼容转弯raw态 */
            report_fault_and_reset(FAULT_BATTERY_LOW);
        }
    }
}

/* ================= 避障任务（200ms，独立低优先级） =================
 * HC-SR04 阻塞式测距最长 60ms，放独立任务不干扰 20ms 控制环——
 * 这正是 RTOS 化的收益之一。 */
void app_obstacle_task(void *arg)
{
    (void)arg;
    uint8_t low_cnt = 0, ok_cnt = 0, blocked = 0;

    for (;;) {
        vTaskDelay(OBST_TASK_PERIOD_MS / portTICK_PERIOD_MS);

        float d = HC_SR04_GetDistance(60000);  /* 阻塞≤60ms，带超时保护 */

        if (!blocked) {
            if (d > 0 && d < OBSTACLE_TH_CM) {
                if (++low_cnt >= OBSTACLE_CONFIRM_CNT) {
                    blocked = 1;
                    low_cnt = 0;
                    nav_pause();  /* 立即停车、状态保持 */
                    if (app_state() == APP_RUN_OUT || app_state() == APP_RUN_BACK) {
                        comm_send_hmi(CMD_FAULT, (uint8_t[]){FAULT_BLOCKED}, 1);
                        m_blocked_reported = 1;
                    }
                }
            } else low_cnt = 0;
        } else {
            /* 恢复判据：连续N次、间隔200ms 均≥阈值，防"挪一半又放回" */
            if (d >= OBSTACLE_TH_CM || d == 0) {  /* 0=超时按畅通处理 ★可按需改 */
                if (++ok_cnt >= OBSTACLE_RECOVER_CNT) {
                    blocked = 0;
                    ok_cnt = 0;
                    xEventGroupSetBits(g_comm_events, EV_RESUME);
                }
            } else ok_cnt = 0;
        }
    }
}

/* ================= 电池任务（500ms） =================
 * ★修改：接口名已按 adc_battery.h/buzzer.h 实际定义替换：
 *   battery_get_voltage() → ADC_GetBatteryVoltage()
 *   buzzer_set(x)         → Buzzer_On()/Buzzer_Off()（宏，PC13高电平响）
 * 删除了原文件底部的 extern 假声明 */
void app_battery_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(BATT_TASK_PERIOD_MS / portTICK_PERIOD_MS);

        float v = ADC_GetBatteryVoltage();

        if (v > 0 && v < BATTERY_STOP_THRESHOLD) {       /* <9.9V 强制停车等级 */
            m_batt_critical = 1;
            Buzzer_On();                                 /* 常鸣 */
        } else if (v > 0 && v < BATTERY_WARN_THRESHOLD) { /* 9.9~10.2V 报警 */
            if ((xTaskGetTickCount() / portTICK_PERIOD_MS / 250) & 1)
                Buzzer_On();                             /* 1Hz 断续 */
            else
                Buzzer_Off();
        } else {
            Buzzer_Off();
        }
    }
}

