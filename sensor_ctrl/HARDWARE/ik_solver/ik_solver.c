/*
*file ik_solver.c
*brief 逆运动学 + 正运动学实现
*note  几何法求解，腕关节自动保持末端水平
*       输入目标点世界坐标 → 输出四个关节角度（度）
*       控制流程：正运动学(当前角度→视觉位置) + 视觉偏移 → 世界坐标 → 逆运动学(目标角度)
*/

#include "ik_solver.h"

#define DEG2RAD (3.14159265358979f / 180.0f)
#define RAD2DEG (180.0f / 3.14159265358979f)

/*
*brief 内部函数：计算垂直平面内的末端和相机位置
*param t2, t3, t4 肩/肘/腕角度（弧度）
*param px, pz 输出末端坐标（相对于肩关节）
*param cx, cz 输出相机坐标（相对于肩关节）
*/
static void fk_plane(float t2, float t3, float t4, float *px, float *pz, float *cx, float *cz)
{
    float L1 = ARM_L1, L2 = ARM_L2, L3 = ARM_L3;
    float cx_off = CAMERA_OFFSET_X, cz_off = CAMERA_OFFSET_Z;

    float elbow_x = L1 * cosf(t2);
    float elbow_z = L1 * sinf(t2);

    float wrist_x = elbow_x + L2 * cosf(t2 + t3);
    float wrist_z = elbow_z + L2 * sinf(t2 + t3);

    float arm_angle = t2 + t3 + t4;   // 小臂姿态角

    *px = wrist_x + L3 * cosf(arm_angle);
    *pz = wrist_z + L3 * sinf(arm_angle);

    *cx = wrist_x + cx_off * cosf(arm_angle) - cz_off * sinf(arm_angle);
    *cz = wrist_z + cx_off * sinf(arm_angle) + cz_off * cosf(arm_angle);
}

/*
*brief 将角度限制在指定范围内
*param angle 输入角度
*param min, max 范围
*retval 钳制后的角度
*/
static float clamp_angle(float angle, float min, float max)
{
    if (angle < min) return min;
    if (angle > max) return max;
    return angle;
}

// ============================================================
//                      正运动学
// ============================================================

/*
*brief 已知四个关节角度，计算机械爪末端世界坐标
*param theta1~theta4 四个关节角度（度）
*param x, y, z 输出末端坐标（mm）
*/
void FK_Solve(float theta1, float theta2, float theta3, float theta4, float *x, float *y, float *z)
{
    float t1 = theta1 * DEG2RAD;
    float t2 = theta2 * DEG2RAD;
    float t3 = theta3 * DEG2RAD;
    float t4 = theta4 * DEG2RAD;
    float H = BASE_HEIGHT;
    float px, pz;

    fk_plane(t2, t3, t4, &px, &pz, NULL, NULL);

    *x = px * cosf(t1);
    *y = px * sinf(t1);
    *z = pz + H;
}

/*
*brief 已知四个关节角度，计算视觉模块世界坐标
*param theta1~theta4 四个关节角度（度）
*param x, y, z 输出视觉模块坐标（mm）
*note  用于将视觉模块测得的相对坐标转换为世界坐标
*/
void FK_GetCameraPos(float theta1, float theta2, float theta3, float theta4, float *x, float *y, float *z)
{
    float t1 = theta1 * DEG2RAD;
    float t2 = theta2 * DEG2RAD;
    float t3 = theta3 * DEG2RAD;
    float t4 = theta4 * DEG2RAD;
    float H = BASE_HEIGHT;
    float cx, cz;

    fk_plane(t2, t3, t4, NULL, NULL, &cx, &cz);

    *x = cx * cosf(t1);
    *y = cx * sinf(t1);
    *z = cz + H;
}

// ============================================================
//                      逆运动学
// ============================================================

/*
*brief 逆运动学求解：已知目标点世界坐标，求四个关节角度
*param x, y, z 目标点坐标（mm，基座底面中心为原点，地面 z=0）
*param angles 输出四个关节角度（度）：[腰部, 肩, 肘, 腕]
*retval 1=成功，0=目标点不可达
*note  末端姿态保持水平（腕关节自动补偿）
*/
uint8_t IK_Solve(float x, float y, float z, float angles[4])
{
    float theta1, theta2, theta3, theta4;
    float r, X, Z;
    float L1 = ARM_L1, L2 = ARM_L2, L3 = ARM_L3, H = BASE_HEIGHT;
    float cos_theta3, sin_theta3;

    // 1. 腰部旋转角度
    theta1 = atan2f(y, x) * RAD2DEG;
    theta1 = fmodf(theta1, 360.0f);
    if (theta1 > 180.0f) theta1 -= 360.0f;
    if (theta1 < -180.0f) theta1 += 360.0f;
    theta1 = clamp_angle(theta1, JOINT1_MIN, JOINT1_MAX);

    // 2. 投影到垂直平面
    r = sqrtf(x*x + y*y);
    X = r - L3;   // 减去小臂长度（末端水平时）
    Z = z - H;    // 减去基座高度

    // 3. 检查目标是否可达
    float dist = sqrtf(X*X + Z*Z);
    if (dist < fabsf(L1 - L2) || dist > (L1 + L2)) {
        return 0;   // 不可达
    }

    // 4. 余弦定理求肘关节角度（取正解，肘向下）
    cos_theta3 = (X*X + Z*Z - L1*L1 - L2*L2) / (2.0f * L1 * L2);
    cos_theta3 = clamp_angle(cos_theta3, -1.0f, 1.0f);
    sin_theta3 = sqrtf(1.0f - cos_theta3 * cos_theta3);
    theta3 = atan2f(sin_theta3, cos_theta3) * RAD2DEG;
    theta3 = clamp_angle(theta3, JOINT3_MIN, JOINT3_MAX);

    // 5. 求肩关节角度
    float temp_angle = atan2f(L2 * sin_theta3, L1 + L2 * cos_theta3);
    theta2 = atan2f(Z, X) - temp_angle;
    theta2 *= RAD2DEG;
    theta2 = clamp_angle(theta2, JOINT2_MIN, JOINT2_MAX);

    // 6. 腕关节：保持末端水平（θ2 + θ3 + θ4 = 0）
    theta4 = -(theta2 + theta3);
    theta4 = clamp_angle(theta4, JOINT4_MIN, JOINT4_MAX);

    angles[0] = theta1;
    angles[1] = theta2;
    angles[2] = theta3;
    angles[3] = theta4;

    return 1;
}
