/*
*file mpu6050.h
*brief MPU6050 驱动声明（软件 I2C，偏航角积分）
*note  SCL=PB8, SDA=PB9, I2C 地址 0x68
*/

#ifndef __MPU6050_H
#define __MPU6050_H

#include "sys.h"
#include "sys_time.h"

// SCL 操作（PB8）
#define MPU6050_SCL_HIGH()  GPIO_SetBits(GPIOB, GPIO_Pin_8)
#define MPU6050_SCL_LOW()   GPIO_ResetBits(GPIOB, GPIO_Pin_8)

// SDA 操作（PB9）
#define MPU6050_SDA_HIGH()  GPIO_SetBits(GPIOB, GPIO_Pin_9)
#define MPU6050_SDA_LOW()   GPIO_ResetBits(GPIOB, GPIO_Pin_9)
#define MPU6050_SDA_READ()  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)

// 设备地址（AD0 接地）
#define MPU6050_ADDR        0x68

// 寄存器地址
#define MPU6050_WHO_AM_I      0x75   // 只读，固定 0x68，用于验证通信
#define MPU6050_PWR_MGMT_1    0x6B   // 电源管理：复位、休眠、时钟源
#define MPU6050_GYRO_CONFIG   0x1B   // 陀螺仪量程
#define MPU6050_ACCEL_CONFIG  0x1C   // 加速度计量程
#define MPU6050_SMPLRT_DIV    0x19   // 采样率分频
#define MPU6050_CONFIG        0x1A   // 低通滤波器带宽
#define MPU6050_GYRO_ZOUT_H   0x47   // Z轴陀螺仪高字节
#define MPU6050_GYRO_ZOUT_L   0x48   // Z轴陀螺仪低字节

void MPU6050_Init(void);
void MPU6050_CalibrateGyro(uint32_t samples);
float MPU6050_GetYaw(void);
void MPU6050_ResetYaw(void);

#endif
