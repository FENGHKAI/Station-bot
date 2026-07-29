/*
*file path_planner.h
*brief 最短路径规划模块（Dijkstra + 障碍物避让）
*note  无向图，邻接表，节点≤15，原点0为出入口
*       图数据通过 graph_config.h 静态配置
*/

#ifndef __PATH_PLANNER_H
#define __PATH_PLANNER_H

#include "sys.h"

#define MAX_NODES            15
#define MAX_EDGES_PER_NODE   8
#define MAX_PATH_LEN         20
#define MAX_SHELF_NUM        20
#define INF                  UINT16_MAX

#define NO_SHELF             0

// 边结构（邻接表节点）
typedef struct {
    uint8_t  target;        // 目标节点编号
    uint16_t weight;        // 边权重（距离）
    uint8_t  shelf_id;      // 货架编号（0=无货架）
    uint8_t  blocked;       // 1=被阻塞
} AdjEdge_t;

// 节点结构
typedef struct {
    uint8_t edge_cnt;
    AdjEdge_t edges[MAX_EDGES_PER_NODE];
} AdjNode_t;

// 图结构
typedef struct {
    uint8_t node_cnt;
    AdjNode_t nodes[MAX_NODES];
} Graph_t;

// 路径结构
typedef struct {
    uint8_t len;
    uint8_t nodes[MAX_PATH_LEN];
    uint16_t total_cost;
} Path_t;

// 货架映射表
typedef struct {
    uint8_t node_a;
    uint8_t node_b;
    uint8_t shelf_id;
} ShelfMap_t;

// ----- 全局变量（外部可访问）-----
extern Graph_t g_graph;
extern ShelfMap_t g_shelf_map[MAX_SHELF_NUM];
extern uint8_t g_shelf_count;

// ----- 公共接口 -----
void PathPlanner_Init(void);
void PathPlanner_ClearGraph(void);

// 路径规划
uint8_t PathPlanner_PlanPath(uint8_t start, uint8_t target, Path_t *path);
uint8_t PathPlanner_PlanRoundTrip(uint8_t start, uint8_t target, Path_t *path);
uint8_t PathPlanner_PlanToShelf(uint8_t shelf_id, Path_t *path);

// 障碍物控制
void PathPlanner_BlockEdge(uint8_t node_a, uint8_t node_b);
void PathPlanner_UnblockEdge(uint8_t node_a, uint8_t node_b);
uint8_t PathPlanner_ReplanFromCurrent(uint8_t current, uint8_t target, Path_t *path);

// 货架查询
uint8_t PathPlanner_FindShelf(uint8_t shelf_id, uint8_t *node_a, uint8_t *node_b);

// 边查找（供 graph_storage 调用）
AdjEdge_t* PathPlanner_FindEdge(uint8_t from, uint8_t to);

// 调试
void PathPlanner_PrintPath(Path_t *path);
void PathPlanner_PrintGraph(void);

#endif
