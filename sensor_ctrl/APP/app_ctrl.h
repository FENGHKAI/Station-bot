/* ================ app_ctrl.h ================
 * 控制层三个 FreeRTOS 任务声明
 *   ctrl : 20ms 主控制（状态机/按键/LED/看门狗/电池临界停车）
 *   obst : 200ms 超声波避障（独立任务，不阻塞控制环）
 *   batt : 500ms 电池监测 + 蜂鸣报警
 * ============================================ */
#ifndef __APP_CTRL_H
#define __APP_CTRL_H

void app_ctrl_task(void *arg);
void app_obstacle_task(void *arg);
void app_battery_task(void *arg);

#endif
