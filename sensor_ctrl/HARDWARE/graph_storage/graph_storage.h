/*
*file graph_storage.h
*brief 图结构编码/解码与存储（W25Q64）
*note  直接操作全局变量 g_graph 和 g_shelf_map
*       存储格式：版本号(1B) + 节点数(1B) + 边数(1B) + 边列表(每边4B)
*       weight 只存低 8 位（实际距离需 ≤ 255，或缩放后存入）
*/

#ifndef __GRAPH_STORAGE_H
#define __GRAPH_STORAGE_H

#include "sys.h"
#include "path_planner.h"
#include "w25q64.h"

#define GRAPH_STORAGE_VERSION  0x01
#define GRAPH_STORAGE_SECTOR   0x000000   // W25Q64 扇区地址

uint8_t GraphStorage_Save(void);
uint8_t GraphStorage_Load(void);

#endif
