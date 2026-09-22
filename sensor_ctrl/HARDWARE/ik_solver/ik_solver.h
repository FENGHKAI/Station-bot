/* *file  ik_solver.h
 * *brief 四自由度机械臂逆运动学求解器
 *        零位：竖直向上 = 中位1500；方向已实测标定
 *
 * 几何法逆解（移植自学长 kinematics.py 并修正）：
 *  - 连杆参数：L0=100, L1=105, L2=88, L3=155（单位mm）
 *  - 坐标输入为世界坐标系（x, y, z），x右正、y前正、z上正
 *  - PWM 范围 500~2500，对应舵机角度 ±135°（270°舵机）
 *  - 零位约定：所有摆动关节 PWM=1500 时机械臂竖直向上
 *  - 方向约定（实测已确认，全部与PWM增大方向一致）：
 *      0 腰：   从上往下看，逆时针为正
 *      1 肩：   从左往右看，顺时针为正（臂向后摆）
 *      2 肘：   从左往右看，逆时针为正（臂向前摆）
 *      3 腕：   从左往右看，顺时针为正（爪向后摆）
 *      4 腕转： 不参与XYZ逆解，单独控制
 *      5 爪：   P0800=张开，P1700=夹紧
 */
#ifndef __IK_SOLVER_H
#define __IK_SOLVER_H

#include "sys.h"

/* ==================== 机械臂连杆参数（单位：mm） ==================== */
#define IK_L0   100.0f    /* 底座高度（地面到肩关节） */
#define IK_L1   105.0f    /* 大臂长度 */
#define IK_L2   88.0f     /* 中臂长度 */
#define IK_L3   155.0f    /* 小臂长度（腕关节到抓取点，含爪） */

/* ==================== PWM 映射参数 ==================== */
#define IK_PWM_CENTER   1500.0f   /* 中位 PWM */
#define IK_PWM_RANGE    2000.0f   /* 满量程偏移量 */
#define IK_ANGLE_RANGE  270.0f    /* 舵机角度范围（度） */
#define IK_PWM_MIN      500.0f
#define IK_PWM_MAX      2500.0f

/* ==================== 零位微调（度） ====================
 * 零位自检时若某关节竖直向上姿态不在1500，调这里（1° ≈ 7.4 PWM）
 * trim为正 = 该关节PWM增大方向补偿 */
#define IK_TRIM0    0.0f    /* 腰 */
#define IK_TRIM1    0.0f    /* 肩 */
#define IK_TRIM2    0.0f    /* 肘 */
#define IK_TRIM3    0.0f    /* 腕 */

/* ==================== 方向开关 ====================
 * +1 = PWM增大 即该关节定义正方向（已实测标定为全+1）
 * 若日后拆装舵机导致方向反了，把对应项改成 -1，几何代码不用动 */
#define IK_DIR0  (+1)
#define IK_DIR1  (+1)
#define IK_DIR2  (+1)
#define IK_DIR3  (+1)

/* ==================== Alpha 搜索参数 ==================== */
#define IK_ALPHA_MIN    -135     /* 搜索下界（度），爪越朝下抓取越稳 */
#define IK_ALPHA_MAX    0        /* 搜索上界（度） */
#define IK_ALPHA_STEP   1        /* 步长（度） */

/* ==================== 关节限位（度） ==================== */
#define IK_THETA6_MIN  -180.0f
#define IK_THETA6_MAX   180.0f
#define IK_THETA5_MIN   0.0f
#define IK_THETA5_MAX   180.0f
#define IK_THETA4_MIN  -135.0f
#define IK_THETA4_MAX   135.0f
#define IK_THETA3_MIN  -90.0f
#define IK_THETA3_MAX   90.0f

/* ==================== 错误码 ==================== */
#define IK_ERR_NONE            0   /* 成功 */
#define IK_ERR_BELOW_GROUND    1   /* 目标低于地面 */
#define IK_ERR_OUT_OF_REACH    2   /* 超出最大臂展 */
#define IK_ERR_COS_THETA3      3   /* 肘关节反余弦参数越界 */
#define IK_ERR_THETA4_OUT      4   /* 肘关节角度超限 */
#define IK_ERR_COS_THETA5      5   /* 肩关节反余弦参数越界 */
#define IK_ERR_THETA5_OUT      6   /* 肩关节角度超限 */
#define IK_ERR_THETA3_OUT      7   /* 腕关节角度超限 */

/* ==================== 机械爪 PWM 端点（不参与逆解） ==================== */
#define GRIP_OPEN_PWM    800     /* 全开 */
#define GRIP_CLOSE_PWM   1700    /* 夹紧 */

/* ==================== 结果结构体 ==================== */
typedef struct
{
    int16_t pwm[4];     /* 关节0~3 的 PWM 值 */
    float   angles[4];  /* 关节角度（度），竖直向上姿态时全为0 */
    float   alpha;      /* 本次解算使用的爪俯仰角（度） */
    uint8_t err;        /* 错误码 */
} IK_Result_t;

/* ==================== 公共接口 ==================== */

/* 逆解 + 自动搜索最优Alpha（爪尽量朝下），成功返回1，失败返回0 */
uint8_t IK_Solve(float x, float y, float z, IK_Result_t *result);

/* 指定Alpha逆解，成功返回1，失败返回0（失败时err含具体错误码） */
uint8_t IK_SolveWithAlpha(float x, float y, float z, float alpha,
                          IK_Result_t *result);

/* 把解算结果打包成 "{#000P...!#001P...!#002P...!#003P...!}" 指令串 */
void IK_BuildCmd(IK_Result_t *result, uint16_t time_ms,
                 char *cmd_buf, uint16_t buf_size);

#endif


