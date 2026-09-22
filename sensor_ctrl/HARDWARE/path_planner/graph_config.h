/*
 * file   graph_config.h
 * brief  图结构的静态配置（直接定义所有节点和边）
 * note   修改此文件即可重新配置地图拓扑
 *        节点编号从0开始，0为原点（出入口）
 *        边权重为距离（uint16_t）
 *        所有边都是无向的（需要成对出现）
 *        货架与节点关联（包裹在节点旁边）
 *
 *        【v0.5 变更】删除 blocked 字段初始化（结构体成员已移除）
 *        修正示意图为 H 结构（原树状示意与实际拓扑不符）
 */
#ifndef __GRAPH_CONFIG_H
#define __GRAPH_CONFIG_H

#include "path_planner.h"

/* ============================================================
 * 实际场地地图（H 结构，6个节点）
 *
 *   4 ─────┐        ┌─ 3
 *          │        │
 *          2 ─────── 1        ← 中间横杠 1-2
 *          │        │
 *   5 ─────┘        └─ 0 (原点)
 *
 *   节点角色：
 *     0 原点（出入口，右下端点）
 *     1 纯路口（H 右竖线与横杠交点，不停留）
 *     2 纯路口（H 左竖线与横杠交点，不停留）
 *     3 取件点1（右上端点，货架1）
 *     4 取件点2（左上端点，货架2）
 *     5 取件点3（左下端点，货架3）
 *
 *   行进方向（去程）：
 *     0→1 直行；1 直行去3；1 左转去2；
 *     2 右转去4；2 左转去5
 * ============================================================ */

static const Graph_t g_graph_default = {
    .node_cnt = 6,
    .nodes = {
        // ====== 节点0（原点，右下端点）======
        { .edge_cnt = 1,
          .edges = {
              {.target = 1, .weight = 80}
          }},

        // ====== 节点1（H 右交点：连0竖线/横杠/3竖线）======
        { .edge_cnt = 3,
          .edges = {
              {.target = 0, .weight = 80},   /* 向下 → 原点 */
              {.target = 2, .weight = 70},   /* 左转 → 横杠 */
              {.target = 3, .weight = 90}    /* 直行 → 取件点1 */
          }},

        // ====== 节点2（H 左交点：连1横杠/4竖线/5竖线）======
        { .edge_cnt = 3,
          .edges = {
              {.target = 1, .weight = 70},   /* 向右 → 横杠 */
              {.target = 4, .weight = 90},   /* 右转 → 取件点2 */
              {.target = 5, .weight = 95}    /* 左转 → 取件点3 */
          }},

        // ====== 节点3（取件点1：货架1，右上端点）======
        { .edge_cnt = 1,
          .edges = {
              {.target = 1, .weight = 90}
          }},

        // ====== 节点4（取件点2：货架2，左上端点）======
        { .edge_cnt = 1,
          .edges = {
              {.target = 2, .weight = 90}
          }},

        // ====== 节点5（取件点3：货架3，左下端点）======
        { .edge_cnt = 1,
          .edges = {
              {.target = 2, .weight = 95}
          }}
    }
};

/* 货架映射表（shelf_id -> node_id） */
static const ShelfMap_t g_shelf_map_default[] = {
    {.node_id = 3, .shelf_id = 1},
    {.node_id = 4, .shelf_id = 2},
    {.node_id = 5, .shelf_id = 3}
};

static const uint8_t g_shelf_count_default = 3;

#endif

