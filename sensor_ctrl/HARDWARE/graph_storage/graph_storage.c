/*
*file graph_storage.c
*brief 图结构编码/解码与存储实现
*note  读取时从Flash加载并解码到 g_graph，写入时编码 g_graph 并写入Flash
*       内部 add_edge_storage 仅用于加载时重建图，不对外暴露
*/

#include "graph_storage.h"

#define STORAGE_BUFFER_SIZE  256

// 静态内部函数：添加边（仅用于 decode_graph）
static uint8_t add_edge_storage(uint8_t from, uint8_t to, uint16_t weight, uint8_t shelf_id)
{
    AdjEdge_t *edge;
    AdjNode_t *node;

    if (from >= MAX_NODES || to >= MAX_NODES) return 0;
    if (weight == 0) return 0;
    if (from == to) return 0;

    if (from + 1 > g_graph.node_cnt) g_graph.node_cnt = from + 1;
    if (to + 1 > g_graph.node_cnt) g_graph.node_cnt = to + 1;

    // 检查是否已存在，若存在则更新
    edge = PathPlanner_FindEdge(from, to);
    if (edge) {
        edge->weight = weight;
        edge->shelf_id = shelf_id;
        edge = PathPlanner_FindEdge(to, from);
        if (edge) {
            edge->weight = weight;
            edge->shelf_id = shelf_id;
        }
        return 1;
    }

    // 添加 from -> to
    if (g_graph.nodes[from].edge_cnt >= MAX_EDGES_PER_NODE) return 0;
    node = &g_graph.nodes[from];
    edge = &node->edges[node->edge_cnt++];
    edge->target = to;
    edge->weight = weight;
    edge->shelf_id = shelf_id;
    edge->blocked = 0;

    // 添加 to -> from（无向图）
    if (g_graph.nodes[to].edge_cnt >= MAX_EDGES_PER_NODE) return 0;
    node = &g_graph.nodes[to];
    edge = &node->edges[node->edge_cnt++];
    edge->target = from;
    edge->weight = weight;
    edge->shelf_id = shelf_id;
    edge->blocked = 0;

    // 更新货架映射
    if (shelf_id != NO_SHELF) {
        uint8_t i;
        for (i = 0; i < g_shelf_count; i++) {
            if (g_shelf_map[i].shelf_id == shelf_id) {
                g_shelf_map[i].node_a = from;
                g_shelf_map[i].node_b = to;
                return 1;
            }
        }
        if (g_shelf_count < MAX_SHELF_NUM) {
            g_shelf_map[g_shelf_count].node_a = from;
            g_shelf_map[g_shelf_count].node_b = to;
            g_shelf_map[g_shelf_count].shelf_id = shelf_id;
            g_shelf_count++;
        }
    }

    return 1;
}

// 编码函数（图结构 -> 字节流）
static uint16_t encode_graph(uint8_t *buf)
{
    uint8_t from, to, edge_cnt = 0;
    uint8_t *ptr = buf;
    uint8_t saved[MAX_NODES][MAX_NODES] = {{0}};
    AdjEdge_t *edge;

    *ptr++ = GRAPH_STORAGE_VERSION;
    *ptr++ = g_graph.node_cnt;

    for (from = 0; from < g_graph.node_cnt; from++) {
        for (to = 0; to < g_graph.nodes[from].edge_cnt; to++) {
            edge = &g_graph.nodes[from].edges[to];
            if (from < edge->target) {
                saved[from][edge->target] = 1;
                edge_cnt++;
            }
        }
    }

    *ptr++ = edge_cnt;

    for (from = 0; from < g_graph.node_cnt; from++) {
        for (to = from + 1; to < g_graph.node_cnt; to++) {
            if (saved[from][to]) {
                edge = PathPlanner_FindEdge(from, to);
                if (edge) {
                    *ptr++ = from;
                    *ptr++ = to;
                    *ptr++ = (uint8_t)(edge->weight & 0xFF);
                    *ptr++ = edge->shelf_id;
                }
            }
        }
    }

    return (uint16_t)(ptr - buf);
}

// 解码函数（字节流 -> 图结构）
static uint8_t decode_graph(uint8_t *buf, uint16_t len)
{
    uint8_t *ptr = buf;
    uint8_t version, node_cnt, edge_cnt, i;
    uint8_t from, to, weight, shelf_id;

    if (len < 3) return 0;

    version = *ptr++;
    if (version != GRAPH_STORAGE_VERSION) return 0;

    node_cnt = *ptr++;
    edge_cnt = *ptr++;

    if (node_cnt == 0 || node_cnt > MAX_NODES) return 0;
    if (edge_cnt > 60) return 0;

    PathPlanner_ClearGraph();

    for (i = 0; i < edge_cnt; i++) {
        if (ptr + 4 > buf + len) return 0;
        from = *ptr++;
        to = *ptr++;
        weight = *ptr++;
        shelf_id = *ptr++;
        if (from >= MAX_NODES || to >= MAX_NODES) return 0;
        if (weight == 0) continue;
        add_edge_storage(from, to, weight, shelf_id);
    }

    return 1;
}

uint8_t GraphStorage_Save(void)
{
    uint8_t buf[STORAGE_BUFFER_SIZE];
    uint16_t len = encode_graph(buf);
    if (len == 0 || len > STORAGE_BUFFER_SIZE) return 0;

    W25Q64_SectorErase(GRAPH_STORAGE_SECTOR);
    W25Q64_WaitBusy();

    W25Q64_PageProgram(GRAPH_STORAGE_SECTOR, buf, len);
    W25Q64_WaitBusy();

    return 1;
}

uint8_t GraphStorage_Load(void)
{
    uint8_t buf[STORAGE_BUFFER_SIZE];
    W25Q64_ReadData(GRAPH_STORAGE_SECTOR, buf, STORAGE_BUFFER_SIZE);
    return decode_graph(buf, STORAGE_BUFFER_SIZE);
}
