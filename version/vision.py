"""
@file vision.py
@brief 视觉分析模块（色块识别 + 对准控制）
@note 视觉算法沿用原工程验证过的逻辑：
     LAB三色阈值 / 最大色块筛选 / 像素偏差→坐标增量 / 连续10次对准防抖
     输出为机械臂基座坐标系下的目标坐标(mm)，由main通过通信模块发给F板
"""

import sensor
import math
from pyb import Timer, Pin

# ==================== 颜色阈值 v2（2026-09-02 实测标定） ====================
# 标定条件：工作距离15cm | 补光PWM60% | 曝光20000µs | 增益/白平衡锁定
red_threshold   = (20, 100,  21,  41,   6,  26)
blue_threshold  = (20, 100,  -7,  13, -42, -22)
green_threshold = (20, 100, -35, -15,  20,  40)
ALL_THRESHOLDS  = [red_threshold, blue_threshold, green_threshold]
COLOR_NAMES     = ['R', 'B', 'G']

# ==================== 对准参数（需要现场标定，见说明文档） ====================
MID_BLOCK_CX = 80          # 期望色块中心X像素（QQVGA 160x120）
MID_BLOCK_CY = 80          # 期望色块中心Y像素（沿用原值，偏画面下部=对准近处物块）
ALIGN_TOL    = 3           # 对准判定容差(像素)
ALIGN_CNT    = 10          # 连续对准次数防抖
STEP_X       = 0.5         # X向每帧增量(mm)，色块偏右→增大
STEP_Y       = 0.3         # Y向每帧增量(mm)，色块偏下(太近)→减小
TGT_X_MIN, TGT_X_MAX = -120, 120   # 目标X限幅(mm)
TGT_Y_MIN, TGT_Y_MAX = 80, 160     # 目标Y限幅(mm)
SEARCH_TIMEOUT_MS = 10000          # 搜索超时(ms)，超时判定失败


class VisionCam:
    """摄像头与色块对准"""

    def __init__(self):
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QQVGA)
        sensor.skip_frames(n=2000)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)
        sensor.set_auto_exposure(False, exposure_us=1500)   # ← 4000→20000，标定值
        sensor.skip_frames(n=300)
        tim = Timer(4, freq=1000)
        self.led = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=100)   # ← 80→60，标定值


    def snapshot(self):
        """取一帧图像"""
        return sensor.snapshot()

    def find_blob(self, img):
        """
        @brief 三色同时识别，返回最大色块（沿用原代码逻辑）
        @return (blob, color_idx) 或 (None, -1)
        """
        best = None
        best_idx = -1
        best_size = 0
        for idx, th in enumerate(ALL_THRESHOLDS):
            blobs = img.find_blobs([th], x_stride=15, y_stride=15, pixels_threshold=25)
            for b in blobs:
                if b[2] * b[3] > best_size:
                    best = b
                    best_idx = idx
                    best_size = b[2] * b[3]
        return best, best_idx

    @staticmethod
    def align_update(tgt_x, tgt_y, blob):
        """
        @brief 对准增量更新（沿用原代码：像素偏差→坐标增量）
        @param tgt_x/tgt_y 当前目标坐标  blob 检测到的色块
        @return (new_x, new_y, aligned, color_idx)
        @note aligned=True仅当单帧满足容差；连续防抖由main用align_cnt实现
        """
        bcx, bcy = blob[5], blob[6]                 # blob[5]/[6]=色块中心

        if abs(bcx - MID_BLOCK_CX) > ALIGN_TOL:
            tgt_x += STEP_X if bcx > MID_BLOCK_CX else -STEP_X
        if abs(bcy - MID_BLOCK_CY) > ALIGN_TOL:
            tgt_y += -STEP_Y if bcy > MID_BLOCK_CY else STEP_Y

        tgt_x = max(TGT_X_MIN, min(TGT_X_MAX, tgt_x))
        tgt_y = max(TGT_Y_MIN, min(TGT_Y_MAX, tgt_y))

        aligned = (abs(bcx - MID_BLOCK_CX) <= ALIGN_TOL and
                   abs(bcy - MID_BLOCK_CY) <= ALIGN_TOL)
        return tgt_x, tgt_y, aligned

    @staticmethod
    def grasp_point(tgt_x, tgt_y, claw_len=89):
        """
        @brief 换算抓取点：目标点沿视线方向前伸爪长（沿用原代码几何换算）
        @return (gx, gy) 抓取点坐标(mm)
        """
        l = math.sqrt(tgt_x * tgt_x + tgt_y * tgt_y)
        if l < 1.0:
            l = 1.0                                 # 除零保护
        gx = (l + claw_len) * (tgt_x / l)
        gy = (l + claw_len) * (tgt_y / l)
        return gx, gy

