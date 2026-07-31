/*
*file ik_solver.h
*brief 四自由度机械臂运动学求解器（正解+逆解）
*note  三段臂（大臂、中臂、小臂）+ 腰部旋转
*       视觉模块偏移参数用于坐标转换
*/

#ifndef __IK_SOLVER_H
#define __IK_SOLVER_H

#include "sys.h"

// ----- 机械臂尺寸参数（单位：mm）-----
#define ARM_L1                  150.0f    // 大臂长度（肩关节到肘关节）
#define ARM_L2                  120.0f    // 中臂长度（肘关节到腕关节）
#define ARM_L3                  100.0f    // 小臂长度（腕关节到末端/机械爪）
#define BASE_HEIGHT             80.0f     // 地面到肩关节的垂直高度（含底座）

// ----- 视觉模块相对末端的偏移（单位：mm）-----
#define CAMERA_OFFSET_X         60.0f     // 相机相对末端沿小臂方向偏移（向前为正）
#define CAMERA_OFFSET_Z         40.0f     // 相机相对末端的垂直偏移（向上为正）

// ----- 关节角度范围限制（单位：度）-----
#define JOINT1_MIN             -180.0f
#define JOINT1_MAX              180.0f
#define JOINT2_MIN              -90.0f
#define JOINT2_MAX               90.0f
#define JOINT3_MIN              -90.0f
#define JOINT3_MAX               90.0f
#define JOINT4_MIN              -90.0f
#define JOINT4_MAX               90.0f

// ----- 公共接口（函数声明，无注释）-----
uint8_t IK_Solve(float x, float y, float z, float angles[4]);
void FK_Solve(float theta1, float theta2, float theta3, float theta4, float *x, float *y, float *z);
void FK_GetCameraPos(float theta1, float theta2, float theta3, float theta4, float *x, float *y, float *z);

#endif
