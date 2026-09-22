"""
@file main.py
@brief 视觉模块主程序（流程调度）
@note 状态流转：IDLE --HANDOVER--> ALIGN --对准--> ACT --完成上报--> IDLE
     抓取/放置由grab_count交替（0=下次抓取, 1=下次放置）
"""

import time
import vision
import uart_comm as uc

# ==================== 动作时序参数（现场标定） ====================
CLAW_UP_Z      = 70      # 悬停高度(mm)
GRAB_DOWN_Z    = -20     # 抓取下降高度(mm)
PLACE_DOWN_Z   = -10     # 放置下降高度(mm)
MOVE_WAIT_MS   = 1200    # 等F板完成移动的时间(ms)
CLAW_WAIT_MS   = 500     # 夹爪动作后等待(ms)
GRAB_TIMEOUT_MS = 1000   # 等夹爪回传最长时间(ms)

# ==================== 状态机 ====================
ST_IDLE  = 0     # 等待F板的HANDOVER
ST_ALIGN = 1     # 搜索+对准
ST_ACT   = 2     # 抓取或放置动作序列


def report_fail(comm, is_grab):
    """统一失败上报"""
    if is_grab:
        comm.send_grab_done(False)
    else:
        comm.send_place_done(False)


def do_action(comm, is_grab, gx, gy):
    """抓取/放置动作序列；返回True=成功"""
    if is_grab:
        # ---- 抓取：张爪 -> 悬停 -> 下降 -> 闭爪 ----
        if not comm.send_claw_open():
            report_fail(comm, True)
            return False
        time.sleep_ms(300)
        comm.read_servo_feedback(200)

        if not comm.send_move_coord(gx, gy, CLAW_UP_Z):
            report_fail(comm, True)
            return False
        time.sleep_ms(MOVE_WAIT_MS)

        if not comm.send_move_coord(gx, gy, GRAB_DOWN_Z):
            report_fail(comm, True)
            return False
        time.sleep_ms(MOVE_WAIT_MS)

        if not comm.send_claw_close():
            report_fail(comm, True)
            return False
        comm.read_servo_feedback(GRAB_TIMEOUT_MS)
        time.sleep_ms(CLAW_WAIT_MS)
        comm.send_grab_done(True)
    else:
        # ---- 放置：悬停 -> 下降 -> 张爪 ----
        if not comm.send_move_coord(gx, gy, CLAW_UP_Z):
            report_fail(comm, False)
            return False
        time.sleep_ms(MOVE_WAIT_MS)

        if not comm.send_move_coord(gx, gy, PLACE_DOWN_Z):
            report_fail(comm, False)
            return False
        time.sleep_ms(MOVE_WAIT_MS)

        if not comm.send_claw_open():
            report_fail(comm, False)
            return False
        comm.read_servo_feedback(GRAB_TIMEOUT_MS)
        time.sleep_ms(CLAW_WAIT_MS)
        comm.send_place_done(True)
    return True


def main():
    comm = uc.Comm()
    cam = vision.VisionCam()

    state = ST_IDLE
    grab_count = 0                   # 0=下次抓取, 1=下次放置
    tgt_x, tgt_y = 0.0, 140.0
    align_cnt = 0
    search_t0 = 0

    while True:
        comm.poll()

        img = cam.snapshot()          # ← 每轮都拍，帧缓冲区永远活着

        if state == ST_IDLE:
            if comm.handover_requested:
                comm.handover_requested = False
                tgt_x, tgt_y = 0.0, 140.0
                align_cnt = 0
                search_t0 = time.ticks_ms()
                state = ST_ALIGN
                print("[STATE] IDLE -> ALIGN")

        elif state == ST_ALIGN:
            blob, cidx = cam.find_blob(img)

            if blob is None:
                if time.ticks_diff(time.ticks_ms(), search_t0) > vision.SEARCH_TIMEOUT_MS:
                    print("[STATE] ALIGN search timeout -> report FAIL")
                    report_fail(comm, grab_count == 0)
                    state = ST_IDLE
            else:
                tgt_x, tgt_y, aligned = vision.VisionCam.align_update(tgt_x, tgt_y, blob)
                print("[ALIGN] blob(%d,%d) tgt=(%.1f,%.1f) cnt=%d"
                      % (blob[5], blob[6], tgt_x, tgt_y, align_cnt))

                if aligned:
                    align_cnt += 1
                    if align_cnt > vision.ALIGN_CNT:
                        align_cnt = 0
                        state = ST_ACT
                        print("[STATE] ALIGN -> ACT")
                else:
                    align_cnt = 0

        elif state == ST_ACT:
            is_grab = (grab_count == 0)
            gx, gy = vision.VisionCam.grasp_point(tgt_x, tgt_y)
            ok = do_action(comm, is_grab, gx, gy)
            if ok:
                grab_count = 1 - grab_count      # 只有成功才翻转
            state = ST_IDLE
            print("[STATE] ACT -> IDLE (round done, %s)"
                  % ("OK" if ok else "FAIL"))

        time.sleep_ms(10)



if __name__ == "__main__":
    main()
