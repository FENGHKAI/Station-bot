/*
*file w25q64.h
*brief W25Q64 SPI Flash 驱动声明
*note  SPI1，NSS=PA4（软件控制），SCK=PA5，MISO=PA6，MOSI=PA7
*      W25Q64 容量 8MB（64Mbit），页大小 256 字节，扇区 4KB，块 64KB
*/

#ifndef __W25Q64_H
#define __W25Q64_H

#include "sys.h"

// SPI 引脚宏定义
#define W25Q_NSS_HIGH()  GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define W25Q_NSS_LOW()   GPIO_ResetBits(GPIOA, GPIO_Pin_4)

// 指令集
#define W25Q_CMD_WREN      0x06   // 写使能
#define W25Q_CMD_WRDIS     0x04   // 写禁止
#define W25Q_CMD_RDID      0x9F   // 读 ID
#define W25Q_CMD_RDSR      0x05   // 读状态寄存器
#define W25Q_CMD_WRSR      0x01   // 写状态寄存器
#define W25Q_CMD_READ      0x03   // 读数据
#define W25Q_CMD_FAST_READ 0x0B   // 快速读
#define W25Q_CMD_PAGE_PROG 0x02   // 页编程
#define W25Q_CMD_SECTOR_ER 0x20   // 扇区擦除（4KB）
#define W25Q_CMD_BLOCK_ER  0xD8   // 块擦除（64KB）
#define W25Q_CMD_CHIP_ER   0xC7   // 全片擦除
#define W25Q_CMD_POWER_DOWN 0xB9  // 掉电
#define W25Q_CMD_RELEASE   0xAB   // 释放掉电

// W25Q64 ID 应为 0xEF4017
#define W25Q64_ID          0xEF4017

// 大小定义
#define W25Q_SECTOR_SIZE   4096   // 扇区 4KB
#define W25Q_PAGE_SIZE     256    // 页 256 字节
#define W25Q_BLOCK_SIZE    65536  // 块 64KB

void W25Q64_Init(void);
uint32_t W25Q64_ReadID(void);
void W25Q64_ReadData(uint32_t addr, uint8_t *buf, uint32_t len);
void W25Q64_SectorErase(uint32_t addr);
void W25Q64_PageProgram(uint32_t addr, uint8_t *data, uint16_t len);
void W25Q64_WaitBusy(void);
void W25Q64_WriteEnable(void);
void W25Q64_WriteDisable(void);

#endif
