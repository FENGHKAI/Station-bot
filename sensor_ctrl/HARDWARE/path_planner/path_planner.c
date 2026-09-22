/*
 * file   path_planner.c
 * brief  最短路径规划实现（Dijkstra）
 * note   图数据通过 graph_config.h 静态配置，编译期确定
 *
 *        【v0.5 变更】
 *        - 删除运行时避障：BlockEdge/UnblockEdge/UnblockNodeEdges/ReplanFromCurrent
 *        - 删除 AdjEdge_t.blocked 字段及 Dijkstra 中的跳过分支
 *          （障碍策略已改为停车上报人工移障，图运行时不可变）
 *        - 删除 ClearGraph（与 Init 等价）
 *        本模块仅负责出发前的路径规划，运行中不再修改图数据。
 */
#include "path_planner.h"
#include "graph_config.h"

/* ----- 全局变量 ----- */
Graph_t    g_graph;
ShelfMap_t g_shelf_map[MAX_SHELF_NUM];
uint8_t    g_shelf_count = 0;

/* Dijkstra 内部状态 */
static uint16_t g_dist[MAX_NODES];
static uint8_t  g_visited[MAX_NODES];
static uint8_t  g_parent[MAX_NODES];

/* ============================================================
 * 内部辅助函数
 * ============================================================ */

/*
 * brief  根据货架号查找对应的节点
 * param  shelf_id  货架号
 * param  node_id   输出节点编号
 * retval 1=找到，0=未找到
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

/* ============================================================
 * Dijkstra 核心
 * ============================================================ */

/*
 * brief  Dijkstra 最短路径算法
 * param  start  起点
 * param  target 终点
 * param  path   输出路径
 * retval 1=找到路径，0=无路径
 */
static uint8_t dijkstra(uint8_t start, uint8_t target, Path_t *path)
{
    uint8_t  i, u, v, cnt;
    uint16_t min_dist;
    uint8_t  path_rev[MAX_PATH_LEN];
    uint8_t  path_len = 0;
    AdjNode_t *node;
    AdjEdge_t *edge;

    /* 初始化 */
    for (i = 0; i < g_graph.node_cnt; i++) {
        g_dist[i]    = INF;
        g_visited[i] = 0;
        g_parent[i]  = 0xFF;
    }
    g_dist[start]   = 0;
    g_parent[start] = start;

    /* 主循环 */
    for (cnt = 0; cnt < g_graph.node_cnt; cnt++) {
        /* 找未访问节点中距离最小的 */
        u = 0xFF;
        min_dist = INF;
        for (i = 0; i < g_graph.node_cnt; i++) {
            if (!g_visited[i] && g_dist[i] < min_dist) {
                min_dist = g_dist[i];
                u = i;
            }
        }

        if (u == 0xFF || u == target) {
            break;    /* 无可达节点或已到终点 */
        }
        g_visited[u] = 1;

        /* 松弛 u 的所有邻接边 */
        node = &g_graph.nodes[u];
        for (i = 0; i < node->edge_cnt; i++) {
            edge = &node->edges[i];
            v = edge->target;

            if (g_visited[v]) {
                continue;
            }
            if (g_dist[u] + edge->weight < g_dist[v]) {
                g_dist[v]   = g_dist[u] + edge->weight;
                g_parent[v] = u;
            }
        }
    }

    if (g_dist[target] >= INF) {
        return 0;    /* 无路径 */
    }

    /* 回溯路径（反向存入再翻转） */
    u = target;
    while (u != start) {
        path_rev[path_len++] = u;
        u = g_parent[u];
        if (u == 0xFF) {
            return 0;    /* 异常保护 */
        }
    }
    path_rev[path_len++] = start;

    path->len = path_len;
    path->total_cost = g_dist[target];
    for (i = 0; i < path_len; i++) {
        path->nodes[i] = path_rev[path_len - 1 - i];
    }

    return 1;
}

/* ============================================================
 * 初始化
 * ============================================================ */

/*
 * brief  初始化图结构（加载 graph_config.h 中的静态配置）
 * note   复制默认图结构和货架映射表
 *        整个系统唯一的地图来源
 */
void PathPlanner_Init(void)
{
    uint8_t i;

    g_graph       = g_graph_default;
    g_shelf_count = g_shelf_count_default;

    for (i = 0; i < g_shelf_count; i++) {
        g_shelf_map[i] = g_shelf_map_default[i];
    }
}

/* ============================================================
 * 路径规划
 * ============================================================ */

/*
 * brief  规划从起点到终点的最短路径
 * param  start  起点
 * param  target 终点
 * param  path   输出路径
 * retval 1=找到路径，0=无路径
 */
uint8_t PathPlanner_PlanPath(uint8_t start, uint8_t target, Path_t *path)
{
    return dijkstra(start, target, path);
}

/*
 * brief  规划往返路径（起点→目标→起点）
 * param  start  起点
 * param  target 目标点
 * param  path   输出路径
 * retval 1=找到路径，0=无路径
 */
uint8_t PathPlanner_PlanRoundTrip(uint8_t start, uint8_t target, Path_t *path)
{
    Path_t  go_path, back_path;
    uint8_t i;

    if (!dijkstra(start, target, &go_path)) {
        return 0;
    }
    if (!dijkstra(target, start, &back_path)) {
        return 0;
    }

    path->len = 0;
    path->total_cost = go_path.total_cost + back_path.total_cost;

    for (i = 0; i < go_path.len; i++) {
        path->nodes[path->len++] = go_path.nodes[i];
    }

    /* 回程从第二个节点开始拼接，避免重复节点 */
    for (i = 1; i < back_path.len; i++) {
        if (back_path.nodes[i] == path->nodes[path->len - 1]) {
            continue;
        }
        path->nodes[path->len++] = back_path.nodes[i];
    }

    return 1;
}

/*
 * brief  规划到货架的往返路径（从原点出发，到达货架所在节点后返回原点）
 * param  shelf_id 货架编号
 * param  path     输出路径
 * retval 1=找到路径，0=货架不存在或无路径
 */
uint8_t PathPlanner_PlanToShelf(uint8_t shelf_id, Path_t *path)
{
    uint8_t target_node;

    if (!find_shelf_node(shelf_id, &target_node)) {
        return 0;
    }

    return PathPlanner_PlanRoundTrip(0, target_node, path);
}

/* ============================================================
 * 货架查询
 * ============================================================ */

/*
 * brief  根据货架号查找对应的节点（外部接口）
 * param  shelf_id 货架号
 * param  node_id  输出节点编号
 * retval 1=找到，0=未找到
 */
uint8_t PathPlanner_FindShelf(uint8_t shelf_id, uint8_t *node_id)
{
    return find_shelf_node(shelf_id, node_id);
}

/* ============================================================
 * 调试
 * ============================================================ */

/*
 * brief  打印路径
 * param  path 路径
 */
void PathPlanner_PrintPath(Path_t *path)
{
    uint8_t i;

    printf("Path: ");
    for (i = 0; i < path->len; i++) {
        printf("%d", path->nodes[i]);
        if (i < path->len - 1) {
            printf(" -> ");
        }
    }
    printf(" (cost: %d)\r\n", path->total_cost);
}

/*
 * brief  打印图结构（调试用）
 */
void PathPlanner_PrintGraph(void)
{
    uint8_t    i, j;
    AdjNode_t *node;
    AdjEdge_t *edge;

    printf("Graph: %d nodes\r\n", g_graph.node_cnt);
    for (i = 0; i < g_graph.node_cnt; i++) {
        node = &g_graph.nodes[i];
        printf("Node %d: ", i);
        for (j = 0; j < node->edge_cnt; j++) {
            edge = &node->edges[j];
            printf("->%d(w:%d) ", edge->target, edge->weight);
        }
        printf("\r\n");
    }

    printf("Shelf count: %d\r\n", g_shelf_count);
    for (i = 0; i < g_shelf_count; i++) {
        printf("Shelf %d -> Node %d\r\n",
               g_shelf_map[i].shelf_id, g_shelf_map[i].node_id);
    }
}


