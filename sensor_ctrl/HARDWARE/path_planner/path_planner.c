/*
*file path_planner.c
*brief 最短路径规划实现（Dijkstra + 阻塞边避让）
*note  图数据通过 graph_config.h 静态配置，PathPlanner_Init 加载
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

static uint8_t find_shelf_edge(uint8_t shelf_id, uint8_t *node_a, uint8_t *node_b)
{
    uint8_t i;
    for (i = 0; i < g_shelf_count; i++) {
        if (g_shelf_map[i].shelf_id == shelf_id) {
            *node_a = g_shelf_map[i].node_a;
            *node_b = g_shelf_map[i].node_b;
            return 1;
        }
    }
    return 0;
}

// ----- Dijkstra 核心 -----
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

// ----- 公共接口 -----

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

void PathPlanner_ClearGraph(void)
{
    PathPlanner_Init();
}

void PathPlanner_BlockEdge(uint8_t node_a, uint8_t node_b)
{
    AdjEdge_t *edge;

    edge = PathPlanner_FindEdge(node_a, node_b);
    if (edge) edge->blocked = 1;

    edge = PathPlanner_FindEdge(node_b, node_a);
    if (edge) edge->blocked = 1;
}

void PathPlanner_UnblockEdge(uint8_t node_a, uint8_t node_b)
{
    AdjEdge_t *edge;

    edge = PathPlanner_FindEdge(node_a, node_b);
    if (edge) edge->blocked = 0;

    edge = PathPlanner_FindEdge(node_b, node_a);
    if (edge) edge->blocked = 0;
}

// ----- 路径规划 -----

uint8_t PathPlanner_PlanPath(uint8_t start, uint8_t target, Path_t *path)
{
    return dijkstra(start, target, path);
}

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

uint8_t PathPlanner_PlanToShelf(uint8_t shelf_id, Path_t *path)
{
    uint8_t node_a, node_b;
    Path_t go_a, go_b, back_a, back_b;
    Path_t candidate1, candidate2;
    uint8_t i;
    AdjEdge_t *edge;

    if (!find_shelf_edge(shelf_id, &node_a, &node_b)) return 0;

    candidate1.len = 0;
    candidate2.len = 0;

    // 候选1：0 -> node_a -> (过边) -> node_b -> 0
    if (dijkstra(0, node_a, &go_a) && dijkstra(node_b, 0, &back_a)) {
        edge = PathPlanner_FindEdge(node_a, node_b);
        candidate1.len = 0;
        candidate1.total_cost = go_a.total_cost + (edge ? edge->weight : 0) + back_a.total_cost;
        for (i = 0; i < go_a.len; i++) candidate1.nodes[candidate1.len++] = go_a.nodes[i];
        candidate1.nodes[candidate1.len++] = node_b;
        for (i = 1; i < back_a.len; i++) {
            if (back_a.nodes[i] == candidate1.nodes[candidate1.len - 1]) continue;
            candidate1.nodes[candidate1.len++] = back_a.nodes[i];
        }
    }

    // 候选2：0 -> node_b -> (过边) -> node_a -> 0
    if (dijkstra(0, node_b, &go_b) && dijkstra(node_a, 0, &back_b)) {
        edge = PathPlanner_FindEdge(node_a, node_b);
        candidate2.len = 0;
        candidate2.total_cost = go_b.total_cost + (edge ? edge->weight : 0) + back_b.total_cost;
        for (i = 0; i < go_b.len; i++) candidate2.nodes[candidate2.len++] = go_b.nodes[i];
        candidate2.nodes[candidate2.len++] = node_a;
        for (i = 1; i < back_b.len; i++) {
            if (back_b.nodes[i] == candidate2.nodes[candidate2.len - 1]) continue;
            candidate2.nodes[candidate2.len++] = back_b.nodes[i];
        }
    }

    if (candidate1.len > 0 && candidate2.len > 0) {
        *path = (candidate1.total_cost <= candidate2.total_cost) ? candidate1 : candidate2;
    } else if (candidate1.len > 0) {
        *path = candidate1;
    } else if (candidate2.len > 0) {
        *path = candidate2;
    } else {
        return 0;
    }
    return 1;
}

uint8_t PathPlanner_ReplanFromCurrent(uint8_t current, uint8_t target, Path_t *path)
{
    return dijkstra(current, target, path);
}

uint8_t PathPlanner_FindShelf(uint8_t shelf_id, uint8_t *node_a, uint8_t *node_b)
{
    return find_shelf_edge(shelf_id, node_a, node_b);
}

// ----- 调试 -----
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
            if (edge->shelf_id) printf(",shelf:%d", edge->shelf_id);
            if (edge->blocked) printf(",BLOCKED");
            printf(") ");
        }
        printf("\r\n");
    }
}
