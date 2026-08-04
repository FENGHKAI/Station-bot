/*
*file graph_config.h
*brief 图结构的静态配置（直接定义所有节点和边）
*note  修改此文件即可重新配置地图拓扑
*       节点编号从0开始，0为原点（出入口）
*       边权重为距离（uint16_t）
*       所有边都是无向的（需要成对出现）
*       货架与节点关联（包裹在节点旁边）
*/

#ifndef __GRAPH_CONFIG_H
#define __GRAPH_CONFIG_H

#include "path_planner.h"

// ============================================================
// 示例图结构（可根据实际地图修改）
// ============================================================
//        (10)        (15)
//    0 ────── 1 ────── 2
//               │        │
//             (20)      (18)
//               │        │
//               3 ────── 4
//               (12)
//
// 货架1在节点2旁边，货架2在节点4旁边
// ============================================================

static const Graph_t g_graph_default = {
    .node_cnt = 5,
    .nodes = {
        // ====== 节点0（原点，唯一出入口）======
        {
            .edge_cnt = 1,
            .edges = {
                {.target = 1, .weight = 10, .blocked = 0}
            }
        },
        // ====== 节点1 ======
        {
            .edge_cnt = 3,
            .edges = {
                {.target = 0, .weight = 10, .blocked = 0},
                {.target = 2, .weight = 15, .blocked = 0},
                {.target = 3, .weight = 20, .blocked = 0}
            }
        },
        // ====== 节点2（货架1旁）======
        {
            .edge_cnt = 3,
            .edges = {
                {.target = 1, .weight = 15, .blocked = 0},
                {.target = 3, .weight = 20, .blocked = 0},
                {.target = 4, .weight = 18, .blocked = 0}
            }
        },
        // ====== 节点3 ======
        {
            .edge_cnt = 3,
            .edges = {
                {.target = 1, .weight = 20, .blocked = 0},
                {.target = 2, .weight = 20, .blocked = 0},
                {.target = 4, .weight = 12, .blocked = 0}
            }
        },
        // ====== 节点4（货架2旁）======
        {
            .edge_cnt = 2,
            .edges = {
                {.target = 2, .weight = 18, .blocked = 0},
                {.target = 3, .weight = 12, .blocked = 0}
            }
        }
    }
};

// 货架映射表（shelf_id -> node_id）
static const ShelfMap_t g_shelf_map_default[] = {
    {.node_id = 2, .shelf_id = 1},
    {.node_id = 4, .shelf_id = 2}
};

static const uint8_t g_shelf_count_default = 2;

#endif
