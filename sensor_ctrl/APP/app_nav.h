/* ================ app_nav.h ================ */
#ifndef __APP_NAV_H
#define __APP_NAV_H

#include "path_planner.h"

typedef enum { NAV_BUSY = 0, NAV_ARRIVED, NAV_FAIL } NavResult_t;

/* 装载路径：从 path 的 start_idx 段开始走，到 stop_idx 号节点停 */
uint8_t nav_start(const Path_t *path, uint8_t start_idx, uint8_t stop_idx);
NavResult_t nav_step(void);      /* 20ms 调用一次 */
void nav_pause(void);            /* 受阻停车（保持相位） */
void nav_resume(void);           /* 移障后续跑 */

#endif
