"""
@file uart_comm.py
@brief 串口通信模块（V1.3协议 + 总线舵机回传）
@note 串口分配：
     UART1：P0=RX P1=TX，与F板通信（V1.3二进制协议帧）
     UART3：P5=RX，接收总线舵机回传（只收不发，辅助确认夹爪动作）
@note 协议与F板protocol.c严格同源：
     帧结构：帧头(1B)+长度(1B)+命令字(1B)+数据体(NB)+校验和(1B)+帧尾(0x0D 0x0A)
     长度字段=转义前数据体长度；校验和覆盖长度+命令字+转义后数据体
     转义：0xA5→7D 01, 0x5A→7D 04, 0x0D→7D 02, 0x0A→7D 03, 0x7D→7D 00
"""
from pyb import UART          # ← 补上这一行
import time

# ==================== 帧定义 ====================
FRAME_HEADER_VISION = 0xA5      # 视觉链路帧头
FRAME_HEADER_HMI    = 0x5A      # HMI链路帧头（仅解析兼容）
FRAME_TAIL1         = 0x0D
FRAME_TAIL2         = 0x0A
FRAME_ESC           = 0x7D
FRAME_MAX_LEN       = 128
DATA_MAX_LEN        = 64

# ==================== 命令字 ====================
CMD_ACK               = 0x00
CMD_VISION_HANDOVER   = 0x01    # F→视觉：交出控制权
CMD_VISION_MOVE_COORD = 0x10    # 视觉→F：移动到指定坐标(6B)
CMD_VISION_CLAW_CLOSE = 0x11    # 视觉→F：夹爪收紧
CMD_VISION_CLAW_OPEN  = 0x12    # 视觉→F：夹爪放开
CMD_VISION_GRAB_DONE  = 0x20    # 视觉→F：抓取完成(1B状态)
CMD_VISION_PLACE_DONE = 0x21    # 视觉→F：放置完成(1B状态)
CMD_TEST_LINK         = 0xFF    # 连接测试

# ==================== 数据编码 ====================
COORD_POS   = 0x00
COORD_NEG   = 0x01
COORD_MAX   = 255               # 坐标幅值上限(mm)
ACTION_OK   = 0x00
ACTION_FAIL = 0x01

# ==================== ACK等待参数 ====================
ACK_TIMEOUT_MS = 300            # 单次等ACK超时
ACK_RETRY      = 3              # 重发次数（协议定3次）


# ============================================================
# 协议底层函数（无状态，直接调用）
# ============================================================

def calc_checksum(buf):
    """校验和：累加取低8位"""
    return sum(buf) & 0xFF


def escape_encode(data):
    """转义编码"""
    out = bytearray()
    for b in data:
        if b == 0xA5:   out += b'\x7d\x01'
        elif b == 0x5A: out += b'\x7d\x04'
        elif b == 0x0D: out += b'\x7d\x02'
        elif b == 0x0A: out += b'\x7d\x03'
        elif b == 0x7D: out += b'\x7d\x00'
        else:           out.append(b)
    return bytes(out)


def escape_decode(data):
    """转义解码；非法转义返回None"""
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        if data[i] == FRAME_ESC:
            if i + 1 >= n:
                return None
            c = data[i + 1]
            if   c == 0x01: out.append(0xA5)
            elif c == 0x04: out.append(0x5A)
            elif c == 0x02: out.append(0x0D)
            elif c == 0x03: out.append(0x0A)
            elif c == 0x00: out.append(0x7D)
            else:           return None
            i += 2
        else:
            out.append(data[i])
            i += 1
    return bytes(out)


def build_frame(header, cmd, data=b''):
    """组帧：返回完整帧bytes；超长返回None"""
    if len(data) > DATA_MAX_LEN:
        return None
    escaped = escape_encode(data)
    if len(escaped) + 6 > FRAME_MAX_LEN:
        return None
    frame = bytearray()
    frame.append(header)
    frame.append(len(data))                 # 长度=转义前长度
    frame.append(cmd)
    frame += escaped
    frame.append(calc_checksum(frame[1:]))  # 校验和覆盖长度+命令字+转义后数据
    frame.append(FRAME_TAIL1)
    frame.append(FRAME_TAIL2)
    return bytes(frame)


def encode_coord(x, y, z):
    """坐标编码：每轴2字节（符号1B+幅值1B），幅值上限255mm，返回6B数据体"""
    out = bytearray()
    for v in (int(x), int(y), int(z)):
        if v < 0:
            out.append(COORD_NEG)
            out.append(min(-v, COORD_MAX))
        else:
            out.append(COORD_POS)
            out.append(min(v, COORD_MAX))
    return bytes(out)


class _Parser:
    """逐字节解析状态机；feed(字节)返回dict或None"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.buf = bytearray()
        self.in_frame = False

    def feed(self, b):
        if not self.in_frame:
            if b == FRAME_HEADER_VISION or b == FRAME_HEADER_HMI:
                self.buf = bytearray([b])
                self.in_frame = True
            return None

        self.buf.append(b)

        if len(self.buf) >= 2 and self.buf[-2] == FRAME_TAIL1 and self.buf[-1] == FRAME_TAIL2:
            frame = bytes(self.buf)
            self.reset()
            return self._parse(frame)

        if len(self.buf) >= FRAME_MAX_LEN:      # 超长残帧丢弃
            self.reset()
        return None

    @staticmethod
    def _parse(frame):
        n = len(frame)
        if n < 6:
            return None
        if frame[-2] != FRAME_TAIL1 or frame[-1] != FRAME_TAIL2:
            return None
        data_len = frame[1]
        cmd      = frame[2]
        escaped  = frame[3:n-3]
        if frame[n-3] != calc_checksum(frame[1:n-3]):
            return None
        raw = escape_decode(escaped) if escaped else b''
        if raw is None:
            return None
        if len(raw) != data_len:
            return None
        return {'header': frame[0], 'cmd': cmd, 'data': raw}


# ============================================================
# 通信类（main.py使用的接口）
# ============================================================

class Comm:
    """与F板通信 + 舵机回传读取"""

    def __init__(self):
        # ---- UART1：与F板通信（P0=RX, P1=TX）----
        self.uart_f = UART(1, 115200)
        self.uart_f.init(115200, bits=8, parity=None, stop=1)
        # ---- UART3：总线舵机回传（P5=RX，只收不发）----
        self.uart_sv = UART(3, 115200)
        self.uart_sv.init(115200, bits=8, parity=None, stop=1)

        self.parser = _Parser()
        self.ack_received = False            # ACK到达标志
        self.handover_requested = False      # 收到HANDOVER标志（供main轮询）

    # ---------- 接收处理 ----------

    def poll(self):
        """
        @brief 轮询UART1，处理收到的完整帧（main主循环每轮调用）
        @note 只置标志，不做业务，业务由main根据标志处理
        """
        while self.uart_f.any():
            res = self.parser.feed(self.uart_f.read(1)[0])
            if res is None:
                continue
            cmd = res['cmd']
            if cmd == CMD_ACK:
                self.ack_received = True
            elif cmd == CMD_TEST_LINK:
                self.send_ack()              # 上电自检：回ACK
            elif cmd == CMD_VISION_HANDOVER:
                self.send_ack()              # 接管：先回ACK
                self.handover_requested = True

    # ---------- 发送 ----------

    def send_ack(self):
        f = build_frame(FRAME_HEADER_VISION, CMD_ACK)
        if f:
            self.uart_f.write(f)

    def send_with_ack(self, cmd, data=b''):
        """
        @brief 发送命令帧并等ACK，失败重发（共3次，协议规定）
        @return True=收到ACK，False=3次超时
        """
        f = build_frame(FRAME_HEADER_VISION, cmd, data)
        if not f:
            return False
        for _ in range(ACK_RETRY):
            self.ack_received = False
            self.uart_f.write(f)
            t0 = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), t0) < ACK_TIMEOUT_MS:
                self.poll()
                if self.ack_received:
                    return True
                time.sleep_ms(5)
        return False

    def send_move_coord(self, x, y, z):
        """发送移动坐标命令"""
        return self.send_with_ack(CMD_VISION_MOVE_COORD, encode_coord(x, y, z))

    def send_claw_open(self):
        """发送夹爪张开命令"""
        return self.send_with_ack(CMD_VISION_CLAW_OPEN)

    def send_claw_close(self):
        """发送夹爪收紧命令"""
        return self.send_with_ack(CMD_VISION_CLAW_CLOSE)

    def send_grab_done(self, ok=True):
        """上报抓取完成（ok=False为失败）"""
        return self.send_with_ack(CMD_VISION_GRAB_DONE,
                                  bytes([ACTION_OK if ok else ACTION_FAIL]))

    def send_place_done(self, ok=True):
        """上报放置完成（ok=False为失败）"""
        return self.send_with_ack(CMD_VISION_PLACE_DONE,
                                  bytes([ACTION_OK if ok else ACTION_FAIL]))

    # ---------- 舵机回传（UART3，只收不发） ----------

    def read_servo_feedback(self, timeout_ms=200):
        """
        @brief 读UART3舵机回传，透传返回bytes（辅助确认夹爪动作）
        @note 收不到返回空串，不阻塞业务（业务有固定延时兜底）
        """
        buf = b''
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
            if self.uart_sv.any():
                buf += self.uart_sv.read()
            else:
                time.sleep_ms(5)
        return buf
