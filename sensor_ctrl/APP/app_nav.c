/* ================ app_nav.c ================
 * 导航执行器：循迹 → 到点(里程) → 查表转向(IMU) → 找线 → 循迹…
 * 转向类型由节点坐标几何推出，与 graph_config.h 的 H 图互为校验
 * ============================================================ */
#include "app_nav.h"
#include "app_config.h"
#include "tracking.h"
#include "timer.h"
#include "motor.h"
#include "mpu6050.h"
#include <math.h>

/* ★【必须核对】节点坐标（仅相对方向参与转向计算，与 graph_config.h
 * 示意图一致：0-1-3 右竖线、5-2-4 左竖线、1-2 横杠）。若场地拓扑
 * 改了，此表与 graph_config.h 必须同步改。 */
static const int16_t node_xy[6][2] = {
    {100,   0}, /* 0 原点   */
    {100,  80}, /* 1 右交点 */
    {  0,  80}, /* 2 左交点 */
    {100, 170}, /* 3 取件1  */
    {  0, 170}, /* 4 取件2  */
    {  0,   0}, /* 5 取件3  */
};

typedef enum { PH_TRACK, PH_TURN, PH_FINDLINE, PH_BLOCKED } NavPhase_t;

static Path_t m_path;
static uint8_t m_idx, m_stop, m_start;
static NavPhase_t m_phase, m_phase_before;
static float m_seg_mm[MAX_PATH_LEN];
static float m_seg_ref;            /* 本段起点里程 mm */
static float m_yaw0, m_turn_target;
static uint8_t m_turn_ok, m_turn_cyc;

/* ---- 里程基准（mm）---- */
static float odom_mm(void)
{
    return (float)Motor_GetOdometer() * ODOM_TO_MM;
}
static void odom_rebase(void)
{
    m_seg_ref = odom_mm();
}

/* ---- 压线检测：任一探头见黑（黑=低电平）---- */
static uint8_t ir_any(void)
{
    return (!GPIO_ReadInputDataBit(GPIOC, IR_X1_PIN)) ||
           (!GPIO_ReadInputDataBit(GPIOC, IR_X2_PIN)) ||
           (!GPIO_ReadInputDataBit(GPIOB, IR_X3_PIN)) ||
           (!GPIO_ReadInputDataBit(GPIOB, IR_X4_PIN));
}

/* ---- 转向类型：0直行 1左 2右 3掉头 ---- */
static uint8_t turn_type(uint8_t a, uint8_t b, uint8_t c)
{
    int32_t d1x = node_xy[b][0] - node_xy[a][0], d1y = node_xy[b][1] - node_xy[a][1];
    int32_t d2x = node_xy[c][0] - node_xy[b][0], d2y = node_xy[c][1] - node_xy[b][1];
    int32_t cross = d1x * d2y - d1y * d2x;
    int32_t dot   = d1x * d2x + d1y * d2y;

    if (dot < 0)   return 3;
    if (cross > 0) return 1;
    if (cross < 0) return 2;
    return 0;
}

static float yaw_err_norm(float err)
{
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

uint8_t nav_start(const Path_t *path, uint8_t start_idx, uint8_t stop_idx)
{
    uint8_t i;
    if (stop_idx <= start_idx || stop_idx >= path->len) return 0;

    m_path  = *path;
    m_start = start_idx;
    m_idx   = start_idx;
    m_stop  = stop_idx;

    for (i = start_idx; i < stop_idx; i++) {  /* 预取各段里程 */
        AdjEdge_t *e = PathPlanner_FindEdge(path->nodes[i], path->nodes[i + 1]);
        if (!e) return 0;
        m_seg_mm[i - start_idx] = (float)e->weight * EDGE_WEIGHT_TO_MM;
    }

    m_phase = PH_TRACK;
    odom_rebase();
    return 1;
}

void nav_pause(void)
{
    if (m_phase != PH_BLOCKED) {
        m_phase_before = m_phase;
        Motor_SetTargetSpeeds(0, 0, 0, 0);
        m_phase = PH_BLOCKED;
    }
}

void nav_resume(void)
{
    if (m_phase == PH_BLOCKED) {
        odom_rebase();
        m_phase = m_phase_before;
    }
}

NavResult_t nav_step(void)
{
    uint8_t seg_i;

    switch (m_phase) {
    case PH_BLOCKED:
        return NAV_BUSY;

    case PH_TRACK: {
        Tracking_Control(TRACK_BASE_SPEED);
        seg_i = m_idx - m_start;

        if (odom_mm() - m_seg_ref >= m_seg_mm[seg_i]) {
            m_idx++;  /* 到达节点 */
            Motor_SetTargetSpeeds(0, 0, 0, 0);

            if (m_idx >= m_stop) return NAV_ARRIVED;

            switch (turn_type(m_path.nodes[m_idx - 1], m_path.nodes[m_idx], m_path.nodes[m_idx + 1])) {
            case 0: /* 直行通过路口 */
                odom_rebase();
                break;
            default: {
                /* 起转：冻结速度环（timer.c 版：关 TIM6） */
                Motor_PauseControl();
                m_yaw0 = MPU6050_GetYaw();  /* ★核对函数名 */
                /* ★修改：删除了占位残句
                   m_turn_target = m_yaw0 + (turn_type(0,0,0), 0);
                   下面大括号内才是真实计算 */
                {
                    uint8_t tt = turn_type(m_path.nodes[m_idx - 1], m_path.nodes[m_idx], m_path.nodes[m_idx + 1]);
                    m_turn_target = m_yaw0 + (tt == 1 ? 90.0f : tt == 2 ? -90.0f : 180.0f);
                }
                m_turn_ok  = 0;
                m_turn_cyc = 0;
                m_phase = PH_TURN;
                break;
            }
            }
        }
        break;
    }

    case PH_TURN: {
        /* 开环差速自旋 + IMU 闭环判停 */
        float err = yaw_err_norm(m_turn_target - MPU6050_GetYaw());
        int16_t pwm = (fabsf(err) > 10.0f) ? SPIN_PWM : SPIN_PWM / 2;
        int8_t dir  = (err > 0) ? 1 : -1;  /* 正=左旋 */

        /* 左旋：左侧退、右侧进（LF,RF,LR,RR）★核对电机通道顺序 */
        Motor_SetPWM_Raw(-dir * pwm, dir * pwm, -dir * pwm, dir * pwm);

        if (fabsf(err) <= TURN_TOL_DEG) {
            if (++m_turn_ok >= TURN_OK_SAMPLES) {
                Motor_SetPWM_Raw(0, 0, 0, 0);
                Motor_ResumeControl();  /* 内部清编码器残留 + 复位PID，防恢复鬼速 */
                odom_rebase();
                m_phase = PH_FINDLINE;
            }
        } else {
            m_turn_ok = 0;
        }

        if (++m_turn_cyc > TURN_TIMEOUT_CYC) {  /* 3s 没转到位 */
            Motor_SetPWM_Raw(0, 0, 0, 0);
            Motor_ResumeControl();
            return NAV_FAIL;
        }
        break;
    }

    case PH_FINDLINE:
        /* 缓速直行重新压线 */
        Tracking_Control(FINDLINE_SPEED);
        if (ir_any()) {
            m_phase = PH_TRACK;
            odom_rebase();
        } else if (odom_mm() - m_seg_ref > FINDLINE_MAX_MM) {
            Motor_SetTargetSpeeds(0, 0, 0, 0);
            return NAV_FAIL;  /* 超距未压线 */
        }
        break;
    }

    return NAV_BUSY;
}
