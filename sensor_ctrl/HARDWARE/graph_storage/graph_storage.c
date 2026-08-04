/*
*file graph_storage.c
*brief 图结构编码/解码与存储实现
*note  读取时从Flash加载并解码到 g_graph，写入时编码 g_graph 并写入Flash
*       内部 add_edge_storage 仅用于加载时重建图，不对外暴露
*       存储格式：版本号(1B) + 节点数(1B) + 边数(1B) + 边列表(N×3B) + 货架数(1B) + 货架映射表(N×2B)
*/

#include "graph_storage.h"

#define STORAGE_BUFFER_SIZE  256

/*
*brief 添加一条边（仅用于从Flash加载时重建图）
*param from 边的起点
*param to 边的终点
*param weight 边权重
*retval 1=成功，0=失败
*note  无向图，自动添加反向边
*       此函数不对外暴露，仅在 decode_graph 中调用
*/
static uint8_t add_edge_storage(uint8_t from, uint8_t to, uint16_t weight)
{
    AdjEdge_t *edge;
    AdjNode_t *node;

    if (from >= MAX_NODES || to >= MAX_NODES) return 0;
    if (weight == 0) return 0;
    if (from == to) return 0;

    if (from + 1 > g_graph.node_cnt) g_graph.node_cnt = from + 1;
    if (to + 1 > g_graph.node_cnt) g_graph.node_cnt = to + 1;

    // 若边已存在则更新
    edge = PathPlanner_FindEdge(from, to);
    if (edge) {
        edge->weight = weight;
        edge = PathPlanner_FindEdge(to, from);
        if (edge) {
            edge->weight = weight;
        }
        return 1;
    }

    // 添加 from -> to
    if (g_graph.nodes[from].edge_cnt >= MAX_EDGES_PER_NODE) return 0;
    node = &g_graph.nodes[from];
    edge = &node->edges[node->edge_cnt++];
    edge->target = to;
    edge->weight = weight;
    edge->blocked = 0;

    // 添加 to -> from（无向图必须双向）
    if (g_graph.nodes[to].edge_cnt >= MAX_EDGES_PER_NODE) return 0;
    node = &g_graph.nodes[to];
    edge = &node->edges[node->edge_cnt++];
    edge->target = from;
    edge->weight = weight;
    edge->blocked = 0;

    return 1;
}

/*
*brief 将图结构编码为字节流
*param buf 输出缓冲区
*retval 编码后的字节数
*note  只存储无向图的单向边（from < to），避免重复
*       weight 只存低 8 位（实际距离需 ≤ 255）
*/
static uint16_t encode_graph(uint8_t *buf)
{
    uint8_t from, to, edge_cnt = 0;
    uint8_t *ptr = buf;
    uint8_t saved[MAX_NODES][MAX_NODES] = {{0}};
    AdjEdge_t *edge;
    uint8_t i;

    // 版本号
    *ptr++ = GRAPH_STORAGE_VERSION;
    // 节点数
    *ptr++ = g_graph.node_cnt;

    // 统计边数（去重：只统计 from < to）
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

    // 写入边列表
    for (from = 0; from < g_graph.node_cnt; from++) {
        for (to = from + 1; to < g_graph.node_cnt; to++) {
            if (saved[from][to]) {
                edge = PathPlanner_FindEdge(from, to);
                if (edge) {
                    *ptr++ = from;
                    *ptr++ = to;
                    *ptr++ = (uint8_t)(edge->weight & 0xFF);
                }
            }
        }
    }

    // 写入货架映射表
    *ptr++ = g_shelf_count;
    for (i = 0; i < g_shelf_count; i++) {
        *ptr++ = g_shelf_map[i].shelf_id;
        *ptr++ = g_shelf_map[i].node_id;
    }

    return (uint16_t)(ptr - buf);
}

/*
*brief 从字节流解码图结构
*param buf 输入缓冲区
*param len 缓冲区长度
*retval 1=成功，0=失败（版本不匹配或数据异常）
*note  先清空当前图，再重建
*       会调用 add_edge_storage 逐条添加边
*/
static uint8_t decode_graph(uint8_t *buf, uint16_t len)
{
    uint8_t *ptr = buf;
    uint8_t version, node_cnt, edge_cnt, i;
    uint8_t from, to, weight;
    uint8_t shelf_count;

    if (len < 3) return 0;

    // 检查版本号
    version = *ptr++;
    if (version != GRAPH_STORAGE_VERSION) return 0;

    node_cnt = *ptr++;
    edge_cnt = *ptr++;

    if (node_cnt == 0 || node_cnt > MAX_NODES) return 0;
    if (edge_cnt > 60) return 0;

    // 清空当前图，准备加载新数据
    PathPlanner_ClearGraph();

    // 逐条添加边
    for (i = 0; i < edge_cnt; i++) {
        if (ptr + 3 > buf + len) return 0;
        from = *ptr++;
        to = *ptr++;
        weight = *ptr++;
        if (from >= MAX_NODES || to >= MAX_NODES) return 0;
        if (weight == 0) continue;
        add_edge_storage(from, to, weight);
    }

    // 解码货架映射表
    shelf_count = *ptr++;
    if (shelf_count > MAX_SHELF_NUM) return 0;

    g_shelf_count = shelf_count;
    for (i = 0; i < shelf_count; i++) {
        if (ptr + 2 > buf + len) return 0;
        g_shelf_map[i].shelf_id = *ptr++;
        g_shelf_map[i].node_id = *ptr++;
    }

    return 1;
}

/*
*brief 将当前图结构保存到 W25Q64
*retval 1=成功，0=失败
*note  会擦除整个扇区，然后写入编码后的数据
*/
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

/*
*brief 从 W25Q64 加载图结构
*retval 1=成功，0=失败
*note  读取后自动解码并填充到 g_graph
*/
uint8_t GraphStorage_Load(void)
{
    uint8_t buf[STORAGE_BUFFER_SIZE];
    W25Q64_ReadData(GRAPH_STORAGE_SECTOR, buf, STORAGE_BUFFER_SIZE);
    return decode_graph(buf, STORAGE_BUFFER_SIZE);
}
