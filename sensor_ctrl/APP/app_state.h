/* ================ app_state.h ================ */
#ifndef __APP_STATE_H
#define __APP_STATE_H

#include <stdint.h>

typedef enum {
    APP_STANDBY = 0,   /* 待命，等启动指令 */
    APP_PLAN,          /* 规划路径 */
    APP_RUN_OUT,       /* 去程循迹 */
    APP_GRAB,          /* 交权给视觉，等抓取完成 0x20 */
    APP_PLAN_BACK,     /* 装载回程路径 */
    APP_RUN_BACK,      /* 回程循迹 */
    APP_DROP,          /* 交权给视觉，等放置完成 0x21 */
} AppState_t;

void     app_state_init(void);
void     app_enter(AppState_t s);           /* 切状态并记录进入时刻 */
AppState_t app_state(void);
uint32_t app_state_elapsed_ms(void);        /* 当前状态已持续 ms */
void     app_force_standby(void);           /* 故障复位入口 */

#endif
