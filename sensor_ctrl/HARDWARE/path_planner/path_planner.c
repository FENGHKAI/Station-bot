/*
*file path_planner.c
*brief 最短路径规划实现（Dijkstra + 阻塞边避让）
*note  图数据通过 graph_config.h 静态配置
*       货架与节点关联（包裹在节点旁边）
*/

#include "path_planner.h"
#include "graph_config.h"

// ----- 全局变量 -----
Graph_t g_graph;
ShelfMap_t g_shelf_map[MAX_SHELF_NUM];
uint8_t g_shelf_count = 0;

// Dijkstra 内部状态
static uint16_t g_dist[MAX_NODES];
static uint8_t g_visited[MAX_NODES];
static uint8_t g_parent[MAX_NODES];

// ----- 内部辅助函数 -----

/*
*brief 在邻接表中查找边
*param from 起始节点
*param to   目标节点
*retval 指向边的指针，未找到返回 NULL
*/
AdjEdge_t* PathPlanner_FindEdge(uint8_t from, uint8_t to)
{
    uint8_t i;
    AdjNode_t *node = &g_graph.nodes[from];
    for (i = 0; i < node->edge_cnt; i++) {
        if (node->edges[i].target == to) {
            return &node->edges[i];
        }
    }
    return NULL;
}

/*
*brief 根据货架号查找对应的节点
*param shelf_id 货架号
*param node_id 输出节点编号
*retval 1=找到，0=未找到
*/
static uint8_t find_shelf_node(uint8_t shelf_id, uint8_t *node_id)
{
    uint8_t i;
    for (i = 0; i < g_shelf_count; i++) {
        if (g_shelf_map[i].shelf_id == shelf_id) {
            *node_id = g_shelf_map[i].node_id;
            return 1;
        }
    }
    return 0;
}

// ----- Dijkstra 核心 -----

/*
*brief Dijkstra 最短路径算法
*param start 起点
*param target 终点
*param path 输出路径
*retval 1=找到路径，0=无路径
*/
static uint8_t dijkstra(uint8_t start, uint8_t target, Path_t *path)
{
    uint8_t i, u, v, cnt;
    uint16_t min_dist;
    uint8_t path_rev[MAX_PATH_LEN];
    uint8_t path_len = 0;
    AdjNode_t *node;
    AdjEdge_t *edge;

    for (i = 0; i < g_graph.node_cnt; i++) {
        g_dist[i] = INF;
        g_visited[i] = 0;
        g_parent[i] = 0xFF;
    }

    g_dist[start] = 0;
    g_parent[start] = start;

    for (cnt = 0; cnt < g_graph.node_cnt; cnt++) {
        u = 0xFF;
        min_dist = INF;
        for (i = 0; i < g_graph.node_cnt; i++) {
            if (!g_visited[i] && g_dist[i] < min_dist) {
                min_dist = g_dist[i];
                u = i;
            }
        }
        if (u == 0xFF || u == target) break;
        g_visited[u] = 1;

        node = &g_graph.nodes[u];
        for (i = 0; i < node->edge_cnt; i++) {
            edge = &node->edges[i];
            v = edge->target;
            if (g_visited[v]) continue;
            if (edge->blocked) continue;
            if (g_dist[u] + edge->weight < g_dist[v]) {
                g_dist[v] = g_dist[u] + edge->weight;
                g_parent[v] = u;
            }
        }
    }

    if (g_dist[target] >= INF) return 0;

    // 回溯路径
    u = target;
    while (u != start) {
        path_rev[path_len++] = u;
        u = g_parent[u];
        if (u == 0xFF) return 0;
    }
    path_rev[path_len++] = start;

    path->len = path_len;
    path->total_cost = g_dist[target];
    for (i = 0; i < path_len; i++) {
        path->nodes[i] = path_rev[path_len - 1 - i];
    }
    return 1;
}

// ============================================================
//                    公共接口
// ============================================================

/*
*brief 初始化图结构（加载 graph_config.h 中的静态配置）
*note  复制默认图结构和货架映射表，清除所有 blocked 标志
*/
void PathPlanner_Init(void)
{
    uint8_t i, j;

    g_graph = g_graph_default;

    g_shelf_count = g_shelf_count_default;
    for (i = 0; i < g_shelf_count; i++) {
        g_shelf_map[i] = g_shelf_map_default[i];
    }

    for (i = 0; i < g_graph.node_cnt; i++) {
        for (j = 0; j < g_graph.nodes[i].edge_cnt; j++) {
            g_graph.nodes[i].edges[j].blocked = 0;
        }
    }
}

/*
*brief 重置图结构为默认配置
*note  等同于 PathPlanner_Init()
*/
void PathPlanner_ClearGraph(void)
{
    PathPlanner_Init();
}

/*
*brief 阻塞一条边（双向标记）
*param node_a, node_b 边的两个端点
*/
void PathPlanner_BlockEdge(uint8_t node_a, uint8_t node_b)
{
    AdjEdge_t *edge;

    edge = PathPlanner_FindEdge(node_a, node_b);
    if (edge) edge->blocked = 1;

    edge = PathPlanner_FindEdge(node_b, node_a);
    if (edge) edge->blocked = 1;
}

/*
*brief 解除一条边的阻塞状态（双向解除）
*param node_a, node_b 边的两个端点
*/
void PathPlanner_UnblockEdge(uint8_t node_a, uint8_t node_b)
{
    AdjEdge_t *edge;

    edge = PathPlanner_FindEdge(node_a, node_b);
    if (edge) edge->blocked = 0;

    edge = PathPlanner_FindEdge(node_b, node_a);
    if (edge) edge->blocked = 0;
}

// ----- 路径规划 -----

/*
*brief 规划从起点到终点的最短路径
*param start 起点
*param target 终点
*param path 输出路径
*retval 1=找到路径，0=无路径
*/
uint8_t PathPlanner_PlanPath(uint8_t start, uint8_t target, Path_t *path)
{
    return dijkstra(start, target, path);
}

/*
*brief 规划往返路径（起点→目标→起点）
*param start 起点
*param target 目标点
*param path 输出路径
*retval 1=找到路径，0=无路径
*/
uint8_t PathPlanner_PlanRoundTrip(uint8_t start, uint8_t target, Path_t *path)
{
    Path_t go_path, back_path;
    uint8_t i;

    if (!dijkstra(start, target, &go_path)) return 0;
    if (!dijkstra(target, start, &back_path)) return 0;

    path->len = 0;
    path->total_cost = go_path.total_cost + back_path.total_cost;

    for (i = 0; i < go_path.len; i++) {
        path->nodes[path->len++] = go_path.nodes[i];
    }
    for (i = 1; i < back_path.len; i++) {
        if (back_path.nodes[i] == path->nodes[path->len - 1]) continue;
        path->nodes[path->len++] = back_path.nodes[i];
    }
    return 1;
}

/*
*brief 规划到货架的往返路径（从原点出发，到达货架所在节点后返回原点）
*param shelf_id 货架编号
*param path 输出路径
*retval 1=找到路径，0=货架不存在或无路径
*/
uint8_t PathPlanner_PlanToShelf(uint8_t shelf_id, Path_t *path)
{
    uint8_t target_node;

    if (!find_shelf_node(shelf_id, &target_node)) return 0;

    return PathPlanner_PlanRoundTrip(0, target_node, path);
}

/*
*brief 从当前位置重新规划到目标
*param current 当前位置
*param target 目标点
*param path 输出路径
*retval 1=找到路径，0=无路径
*note  用于障碍物避让后的重新规划
*/
uint8_t PathPlanner_ReplanFromCurrent(uint8_t current, uint8_t target, Path_t *path)
{
    return dijkstra(current, target, path);
}

/*
*brief 根据货架号查找对应的节点
*param shelf_id 货架号
*param node_id 输出节点编号
*retval 1=找到，0=未找到
*/
uint8_t PathPlanner_FindShelf(uint8_t shelf_id, uint8_t *node_id)
{
    return find_shelf_node(shelf_id, node_id);
}

// ----- 调试 -----

/*
*brief 打印路径
*param path 路径
*/
void PathPlanner_PrintPath(Path_t *path)
{
    uint8_t i;
    printf("Path: ");
    for (i = 0; i < path->len; i++) {
        printf("%d", path->nodes[i]);
        if (i < path->len - 1) printf(" -> ");
    }
    printf(" (cost: %d)\r\n", path->total_cost);
}

/*
*brief 打印图结构（调试用）
*/
void PathPlanner_PrintGraph(void)
{
    uint8_t i, j;
    AdjNode_t *node;
    AdjEdge_t *edge;

    printf("Graph: %d nodes\r\n", g_graph.node_cnt);
    for (i = 0; i < g_graph.node_cnt; i++) {
        node = &g_graph.nodes[i];
        printf("Node %d: ", i);
        for (j = 0; j < node->edge_cnt; j++) {
            edge = &node->edges[j];
            printf("->%d(w:%d", edge->target, edge->weight);
            if (edge->blocked) printf(",BLOCKED");
            printf(") ");
        }
        printf("\r\n");
    }

    printf("Shelf count: %d\r\n", g_shelf_count);
    for (i = 0; i < g_shelf_count; i++) {
        printf("Shelf %d -> Node %d\r\n", g_shelf_map[i].shelf_id, g_shelf_map[i].node_id);
    }
}
