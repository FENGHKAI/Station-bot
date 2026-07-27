/*
*file w25q64.c
*brief W25Q64 SPI Flash 驱动实现
*note  SPI1，软件 NSS，读写操作前需确保片选正确
*/

#include "w25q64.h"

/*
*brief SPI 读写一个字节
*param tx 发送字节
*retval 接收字节
*/
static uint8_t W25Q_SPI_ReadWrite(uint8_t tx)
{
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI1, tx);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    return SPI_I2S_ReceiveData(SPI1);
}

/*
*brief 初始化 SPI1 和 GPIO
*note  PA5 SCK，PA6 MISO，PA7 MOSI，PA4 NSS（软件控制）
*      SPI 模式 0（CPOL=0，CPHA=0），8 位数据，时钟 21MHz（168M/8）
*/
void W25Q64_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    SPI_InitTypeDef SPI_InitStruct;

    // 使能 GPIOA 和 SPI1 时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    // 配置 PA4（NSS）为推挽输出，默认高电平
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    W25Q_NSS_HIGH();

    // 配置 PA5（SCK），PA6（MISO），PA7（MOSI）为复用功能
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 复用功能映射到 SPI1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

    // 配置 SPI1：主机，模式0，8位，时钟 168M/8 = 21MHz
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;   // 21MHz
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStruct);
    SPI_Cmd(SPI1, ENABLE);
}

/*
*brief 读取 W25Q64 的 ID
*retval 24 位 ID，W25Q64 应为 0xEF4017
*/
uint32_t W25Q64_ReadID(void)
{
    uint32_t id = 0;
    uint8_t tmp;

    W25Q_NSS_LOW();
    W25Q_SPI_ReadWrite(W25Q_CMD_RDID);   // 发送读 ID 指令
    tmp = W25Q_SPI_ReadWrite(0x00);
    id = (uint32_t)tmp << 16;
    tmp = W25Q_SPI_ReadWrite(0x00);
    id |= (uint32_t)tmp << 8;
    tmp = W25Q_SPI_ReadWrite(0x00);
    id |= tmp;
    W25Q_NSS_HIGH();

    return id;
}

/*
*brief 写使能
*/
void W25Q64_WriteEnable(void)
{
    W25Q_NSS_LOW();
    W25Q_SPI_ReadWrite(W25Q_CMD_WREN);
    W25Q_NSS_HIGH();
}

/*
*brief 写禁止
*/
void W25Q64_WriteDisable(void)
{
    W25Q_NSS_LOW();
    W25Q_SPI_ReadWrite(W25Q_CMD_WRDIS);
    W25Q_NSS_HIGH();
}

/*
*brief 等待 Flash 忙状态结束（读状态寄存器 bit0）
*/
void W25Q64_WaitBusy(void)
{
    uint8_t sr;
    do {
        W25Q_NSS_LOW();
        W25Q_SPI_ReadWrite(W25Q_CMD_RDSR);
        sr = W25Q_SPI_ReadWrite(0x00);
        W25Q_NSS_HIGH();
    } while (sr & 0x01);   // 当 bit0（BUSY）为 1 时继续等待
}

/*
*brief 扇区擦除（4KB）
*param addr 扇区起始地址（必须是 4096 的整数倍）
*note  擦除后该扇区所有字节变为 0xFF
*/
void W25Q64_SectorErase(uint32_t addr)
{
    W25Q64_WriteEnable();
    W25Q64_WaitBusy();

    W25Q_NSS_LOW();
    W25Q_SPI_ReadWrite(W25Q_CMD_SECTOR_ER);
    W25Q_SPI_ReadWrite((addr >> 16) & 0xFF);
    W25Q_SPI_ReadWrite((addr >> 8) & 0xFF);
    W25Q_SPI_ReadWrite(addr & 0xFF);
    W25Q_NSS_HIGH();

    W25Q64_WaitBusy();
}

/*
*brief 页编程（最大 256 字节）
*param addr 起始地址（必须是页内偏移 0~255，即低 8 位任意）
*param data 数据指针
*param len 要写入的字节数（最大 256）
*note  写入前必须先擦除对应扇区，且不能跨页（单次最大 256 字节）
*      如果 len 超过 256，只写入前 256 字节
*/
void W25Q64_PageProgram(uint32_t addr, uint8_t *data, uint16_t len)
{
    uint16_t i;

    if (len > W25Q_PAGE_SIZE) len = W25Q_PAGE_SIZE;

    W25Q64_WriteEnable();
    W25Q64_WaitBusy();

    W25Q_NSS_LOW();
    W25Q_SPI_ReadWrite(W25Q_CMD_PAGE_PROG);
    W25Q_SPI_ReadWrite((addr >> 16) & 0xFF);
    W25Q_SPI_ReadWrite((addr >> 8) & 0xFF);
    W25Q_SPI_ReadWrite(addr & 0xFF);
    for (i = 0; i < len; i++) {
        W25Q_SPI_ReadWrite(data[i]);
    }
    W25Q_NSS_HIGH();

    W25Q64_WaitBusy();
}

/*
*brief 读取数据
*param addr 起始地址
*param buf 接收缓冲区
*param len 要读取的字节数
*/
void W25Q64_ReadData(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint32_t i;

    W25Q_NSS_LOW();
    W25Q_SPI_ReadWrite(W25Q_CMD_READ);
    W25Q_SPI_ReadWrite((addr >> 16) & 0xFF);
    W25Q_SPI_ReadWrite((addr >> 8) & 0xFF);
    W25Q_SPI_ReadWrite(addr & 0xFF);
    for (i = 0; i < len; i++) {
        buf[i] = W25Q_SPI_ReadWrite(0x00);
    }
    W25Q_NSS_HIGH();
}
