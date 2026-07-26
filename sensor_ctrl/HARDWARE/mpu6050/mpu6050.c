/*
*file mpu6050.c
*brief MPU6050 驱动实现（软件 I2C，陀螺仪 Z 轴积分得到偏航角）
*note  依赖 sys_time.h 提供 udelay() 和 get_us()
*/

#include "mpu6050.h"

// 软件 I2C 底层
static void I2C_Delay(void)
{
    udelay(2);
}

static void I2C_Start(void)
{
    MPU6050_SDA_HIGH();
    MPU6050_SCL_HIGH();
    I2C_Delay();
    MPU6050_SDA_LOW();
    I2C_Delay();
    MPU6050_SCL_LOW();
    I2C_Delay();
}

static void I2C_Stop(void)
{
    MPU6050_SDA_LOW();
    MPU6050_SCL_HIGH();
    I2C_Delay();
    MPU6050_SDA_HIGH();
    I2C_Delay();
}

static uint8_t I2C_WriteByte(uint8_t byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (byte & 0x80)
            MPU6050_SDA_HIGH();
        else
            MPU6050_SDA_LOW();
        byte <<= 1;
        MPU6050_SCL_HIGH();
        I2C_Delay();
        MPU6050_SCL_LOW();
        I2C_Delay();
    }
    MPU6050_SDA_HIGH();
    MPU6050_SCL_HIGH();
    I2C_Delay();
    uint8_t ack = MPU6050_SDA_READ();
    MPU6050_SCL_LOW();
    I2C_Delay();
    return ack;
}

static uint8_t I2C_ReadByte(uint8_t ack)
{
    uint8_t byte = 0;
    uint8_t i;
    MPU6050_SDA_HIGH();
    for (i = 0; i < 8; i++) {
        byte <<= 1;
        MPU6050_SCL_HIGH();
        I2C_Delay();
        if (MPU6050_SDA_READ())
            byte |= 0x01;
        MPU6050_SCL_LOW();
        I2C_Delay();
    }
    if (ack)
        MPU6050_SDA_LOW();
    else
        MPU6050_SDA_HIGH();
    MPU6050_SCL_HIGH();
    I2C_Delay();
    MPU6050_SCL_LOW();
    I2C_Delay();
    MPU6050_SDA_HIGH();
    return byte;
}

static uint8_t MPU6050_ReadReg(uint8_t reg)
{
    uint8_t data;
    I2C_Start();
    I2C_WriteByte((MPU6050_ADDR << 1) | 0);
    I2C_WriteByte(reg);
    I2C_Start();
    I2C_WriteByte((MPU6050_ADDR << 1) | 1);
    data = I2C_ReadByte(0);
    I2C_Stop();
    return data;
}

static void MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    I2C_Start();
    I2C_WriteByte((MPU6050_ADDR << 1) | 0);
    I2C_WriteByte(reg);
    I2C_WriteByte(data);
    I2C_Stop();
}

static int16_t MPU6050_ReadInt16(uint8_t reg)
{
    uint8_t h, l;
    h = MPU6050_ReadReg(reg);
    l = MPU6050_ReadReg(reg + 1);
    return (int16_t)((h << 8) | l);
}

static float gyro_z_offset = 0.0f;
static float yaw_angle = 0.0f;
static uint32_t last_time_us = 0;

/*
*brief 初始化 GPIO 和 MPU6050
*note  唤醒芯片，量程 ±2000dps，采样率 100Hz，滤波带宽 42Hz
*/
void MPU6050_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    MPU6050_SCL_HIGH();
    MPU6050_SDA_HIGH();

    udelay(10000);

    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x80);
    udelay(10000);

    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 9);
    MPU6050_WriteReg(MPU6050_CONFIG, 0x03);

    MPU6050_ResetYaw();
}

/*
*brief 校准陀螺仪零偏（需静止放置）
*param samples 采样次数，建议 100~200
*note  会重置偏航角，校准后零偏值用于后续积分
*/
void MPU6050_CalibrateGyro(uint32_t samples)
{
    int32_t sum = 0;
    uint32_t i;

    MPU6050_ResetYaw();

    for (i = 0; i < samples; i++) {
        sum += MPU6050_ReadInt16(MPU6050_GYRO_ZOUT_H);
        udelay(1000);
    }
    gyro_z_offset = (float)sum / samples / 16.4f;

    last_time_us = get_us();
}

/*
*brief 重置偏航角为 0
*/
void MPU6050_ResetYaw(void)
{
    yaw_angle = 0.0f;
    last_time_us = get_us();
}

/*
*brief 获取当前偏航角（度）
*retval 累计角度（顺时针为正，逆时针为负）
*note  每次读取会积分 Z 轴角速度，长时间会有漂移
*/
float MPU6050_GetYaw(void)
{
    int16_t raw;
    float gyro_z_dps;
    uint32_t now_us;
    float dt;

    raw = MPU6050_ReadInt16(MPU6050_GYRO_ZOUT_H);
    gyro_z_dps = (float)raw / 16.4f - gyro_z_offset;

    now_us = get_us();
    if (last_time_us == 0) {
        dt = 0.01f;
    } else {
        dt = (float)(now_us - last_time_us) / 1000000.0f;
        if (dt > 0.1f) dt = 0.1f;
    }
    last_time_us = now_us;

    yaw_angle += gyro_z_dps * dt;
    return yaw_angle;
}
