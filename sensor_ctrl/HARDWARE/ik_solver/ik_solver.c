/* *file  ik_solver.c
 * *brief 四自由度机械臂逆运动学求解器
 *        零位：竖直向上=1500；方向：PWM增大=定义正方向（全+1）
 */
#include "ik_solver.h"
#include <math.h>
#include <stdio.h>

#define DEG2RAD  (3.14159265358979f / 180.0f)
#define RAD2DEG  (180.0f / 3.14159265358979f)

/* ---------------- 工具函数 ---------------- */

static float safe_acos(float x)
{
    if (x > 1.0f)  x = 1.0f;
    if (x < -1.0f) x = -1.0f;
    return acosf(x);
}

static float normalize_angle(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/* ---------------- 角度→PWM 映射 ----------------
 * 统一公式：PWM = 1500 + 2000 · dir · (angle + trim) / 270
 *  - angle 的正方向 = 该关节定义正方向
 *  - 竖直向上姿态时四个摆动关节的 angle 全为 0 → 全部输出 1500
 *  - trim 用于零位微调；dir 用于方向翻转（当前全为 +1） */
static int16_t angle_to_pwm(float angle_deg, float trim, int dir)
{
    float pwm = IK_PWM_CENTER
              + IK_PWM_RANGE * (float)dir * (angle_deg + trim)
              / IK_ANGLE_RANGE;

    if (pwm < IK_PWM_MIN) pwm = IK_PWM_MIN;
    if (pwm > IK_PWM_MAX) pwm = IK_PWM_MAX;
    return (int16_t)(pwm + 0.5f);
}

/* ---------------- 逆解核心 ----------------
 * 返回 IK_ERR_NONE 表示成功，其他错误码见 ik_solver.h */
static uint8_t ik_core(float x, float y, float z, float alpha,
                       IK_Result_t *result)
{
    /* 坐标放大10倍，用浮点保留精度（与学长0.1mm方案等价） */
    float x10 = x * 10.0f;
    float y10 = y * 10.0f;
    float z10 = z * 10.0f;
    float l0  = IK_L0 * 10.0f;
    float l1  = IK_L1 * 10.0f;
    float l2  = IK_L2 * 10.0f;
    float l3  = IK_L3 * 10.0f;
    float theta6, theta5, theta4, theta3;
    float alpha_rad = alpha * DEG2RAD;

    /* ---- 关节0 腰：正方向 = 从上往下逆时针 ----
     * 顶视（+Y向前、+X向右）：逆时针转φ后臂指向(-sinφ, cosφ)方向
     * 故 φ = atan2(-x, y)；目标在右(x>0)时φ为负=顺时针，符合定义 */
    if (x10 == 0.0f && y10 == 0.0f) {
        theta6 = 0.0f;
    } else {
        theta6 = atan2f(-x10, y10) * RAD2DEG;
    }
    theta6 = normalize_angle(theta6);

    /* ---- 投影到 Y-Z 平面（几何内核与学长一致）----
     * 扣除爪长L3在Alpha方向的投影，把4自由度降为平面2连杆问题 */
    float y_proj = sqrtf(x10 * x10 + y10 * y10);
    y_proj = y_proj - l3 * cosf(alpha_rad);
    float z_proj = z10 - l0 - l3 * sinf(alpha_rad);

    if (z_proj < -l0)     return IK_ERR_BELOW_GROUND;

    float dist = sqrtf(y_proj * y_proj + z_proj * z_proj);
    if (dist > (l1 + l2)) return IK_ERR_OUT_OF_REACH;

    /* ---- 关节1 肩：theta5 = 大臂与水平面夹角，90°=竖直 ----
     * servo_angle1 = theta5 - 90，竖直时 = 0，对应PWM 1500 */
    float ccc = safe_acos(y_proj / dist);
    float bbb = (y_proj * y_proj + z_proj * z_proj + l1 * l1 - l2 * l2)
              / (2.0f * l1 * dist);
    if (bbb > 1.0f || bbb < -1.0f) return IK_ERR_COS_THETA5;

    float zf_flag = (z_proj < 0.0f) ? -1.0f : 1.0f;
    theta5 = (ccc * zf_flag + safe_acos(bbb)) * RAD2DEG;
    if (theta5 > IK_THETA5_MAX || theta5 < IK_THETA5_MIN)
        return IK_ERR_THETA5_OUT;

    /* ---- 关节2 肘：theta4 = 0 时大小臂伸直 ----
     * 正值 = 肘向"前"弯（小臂相对大臂向前收）= 你的前摆正方向 */
    float aaa = -(y_proj * y_proj + z_proj * z_proj - l1 * l1 - l2 * l2)
              / (2.0f * l1 * l2);
    if (aaa > 1.0f || aaa < -1.0f) return IK_ERR_COS_THETA3;
    theta4 = 180.0f - safe_acos(aaa) * RAD2DEG;
    if (theta4 > IK_THETA4_MAX || theta4 < IK_THETA4_MIN)
        return IK_ERR_THETA4_OUT;

    /* ---- 关节3 腕：theta3 = α - θ5 + θ4，保证爪绝对俯仰角恒为α ----
     * 正值 = 爪相对小臂向后抬起 = 你的后摆正方向 */
    theta3 = alpha - theta5 + theta4;
    if (theta3 > IK_THETA3_MAX || theta3 < IK_THETA3_MIN)
        return IK_ERR_THETA3_OUT;

    /* ---- 角度→PWM：四路统一公式，无需任何镜像 ----
     * 竖直向上姿态：θ5-90=0, θ4=0, θ3=0 → PWM 全为1500 */
    result->pwm[0] = angle_to_pwm(theta6,         IK_TRIM0, IK_DIR0);
    result->pwm[1] = angle_to_pwm(theta5 - 90.0f, IK_TRIM1, IK_DIR1);
    result->pwm[2] = angle_to_pwm(theta4,         IK_TRIM2, IK_DIR2);
    result->pwm[3] = angle_to_pwm(theta3,         IK_TRIM3, IK_DIR3);

    result->angles[0] = theta6;
    result->angles[1] = theta5 - 90.0f;
    result->angles[2] = theta4;
    result->angles[3] = theta3;
    result->alpha = alpha;
    result->err = IK_ERR_NONE;
    return IK_ERR_NONE;
}

/* ---------------- 公共接口 ---------------- */

/* 逆解 + 自动搜索最优Alpha：从0°扫到-135°，取最负的有效解
 * （Alpha越负爪越朝下，抓取越稳） */
uint8_t IK_Solve(float x, float y, float z, IK_Result_t *result)
{
    if (result == NULL) return 0;
    if (y < 0.0f) {                       /* 臂无法转到身后 */
        result->err = IK_ERR_OUT_OF_REACH;
        return 0;
    }

    int8_t best_alpha = 0;
    uint8_t found = 0;
    IK_Result_t temp;

    for (int alpha = IK_ALPHA_MAX; alpha >= IK_ALPHA_MIN;
         alpha -= IK_ALPHA_STEP)
    {
        uint8_t err = ik_core(x, y, z, (float)alpha, &temp);
        if (err == IK_ERR_NONE)
        {
            if (!found || alpha < best_alpha)
            {
                best_alpha = alpha;
                found = 1;
                *result = temp;
            }
        }
    }

    if (!found)
    {
        result->err = IK_ERR_OUT_OF_REACH;
        return 0;
    }
    result->alpha = (float)best_alpha;
    return 1;
}

/* 指定Alpha的逆解（不用搜索，速度快，适合已知姿态的场景） */
uint8_t IK_SolveWithAlpha(float x, float y, float z, float alpha,
                          IK_Result_t *result)
{
    if (result == NULL) return 0;
    if (y < 0.0f) {
        result->err = IK_ERR_OUT_OF_REACH;
        return 0;
    }

    uint8_t err = ik_core(x, y, z, alpha, result);
    if (err != IK_ERR_NONE) {
        result->err = err;
        return 0;
    }
    return 1;
}

/* 打包为协议指令串：
 * "{#000PxxxxTyyyy!#001PxxxxTyyyy!#002PxxxxTyyyy!#003PxxxxTyyyy!}"
 * 多指令必须用{}包裹（协议要求） */
void IK_BuildCmd(IK_Result_t *result, uint16_t time_ms,
                 char *cmd_buf, uint16_t buf_size)
{
    if (result == NULL || cmd_buf == NULL || buf_size < 64) return;

    if (time_ms > 9999) time_ms = 9999;

    snprintf(cmd_buf, buf_size,
        "{#000P%04dT%04d!#001P%04dT%04d!#002P%04dT%04d!#003P%04dT%04d!}",
        result->pwm[0], time_ms,
        result->pwm[1], time_ms,
        result->pwm[2], time_ms,
        result->pwm[3], time_ms);
}

