/*
 * file   path_planner.h
 * brief  最短路径规划模块（Dijkstra）
 * note   无向图，邻接表，节点≤15，原点0为出入口
 *        图数据通过 graph_config.h 静态配置，编译期确定，无持久化
 *        货架与节点关联（包裹在节点旁边）
 *
 *        【v0.5 变更】运行时避障接口已删除（BlockEdge/Unblock/Replan*）：
 *        障碍处理策略改为"停车上报人工移障"（见工作流程文档 5.3），
 *        车辆障碍时不动，图状态无需运行时修改。
 *        本模块仅负责出发前的路径规划。
 */
 
#ifndef __PATH_PLANNER_H
#define __PATH_PLANNER_H

#include "sys.h"

#define MAX_NODES          15     /* 最大节点数 */
#define MAX_EDGES_PER_NODE 8      /* 每个节点最大邻接边数 */
#define MAX_PATH_LEN       20     /* 路径最大节点数 */
#define MAX_SHELF_NUM      20     /* 最大货架编号 */
#define INF                UINT16_MAX
#define NO_SHELF           0

typedef struct {
    uint8_t  target;             /* 目标节点编号 */
    uint16_t weight;             /* 边权重（距离） */
} AdjEdge_t;

typedef struct {
    uint8_t   edge_cnt;          /* 邻接边数量 */
    AdjEdge_t edges[MAX_EDGES_PER_NODE];
} AdjNode_t;

typedef struct {
    uint8_t   node_cnt;          /* 节点总数 */
    AdjNode_t nodes[MAX_NODES];
} Graph_t;

typedef struct {
    uint8_t  len;                /* 路径长度（节点数） */
    uint8_t  nodes[MAX_PATH_LEN];
    uint16_t total_cost;         /* 总代价 */
} Path_t;

/* 货架映射表（货架 → 节点） */
typedef struct {
    uint8_t node_id;             /* 货架所在的节点编号 */
    uint8_t shelf_id;            /* 货架编号 */
} ShelfMap_t;

/* ----- 全局变量（外部可访问）----- */
extern Graph_t    g_graph;
extern ShelfMap_t g_shelf_map[MAX_SHELF_NUM];
extern uint8_t    g_shelf_count;

/* ----- 公共接口 ----- */
void PathPlanner_Init(void);

/* 路径规划 */
uint8_t PathPlanner_PlanPath(uint8_t start, uint8_t target, Path_t *path);
uint8_t PathPlanner_PlanRoundTrip(uint8_t start, uint8_t target, Path_t *path);
uint8_t PathPlanner_PlanToShelf(uint8_t shelf_id, Path_t *path);

/* 货架查询 */
uint8_t PathPlanner_FindShelf(uint8_t shelf_id, uint8_t *node_id);

/* 边查找（内部/调试可用） */
AdjEdge_t* PathPlanner_FindEdge(uint8_t from, uint8_t to);

/* 调试 */
void PathPlanner_PrintPath(Path_t *path);
void PathPlanner_PrintGraph(void);

#endif
