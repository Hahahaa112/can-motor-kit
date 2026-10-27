"""
RobStride RS04 CAN 驱动（python-can）

协议依据：实验室提供的《RS04 使用说明书》（RS04-CHN，灵足时代）。
帧格式、位定义、参数 ID、量程均以该手册为准，详见 docs/CAN协议.md。

用法：
    # Linux + SocketCAN
    sudo ip link set can0 up type can bitrate 1000000
    python robstride.py --interface socketcan --channel can0 --id 1

    # Windows + CANable(slcan)
    python robstride.py --interface slcan --channel COM5 --id 1

依赖：
    pip install python-can
"""

from __future__ import annotations

import argparse
import math
import struct
import time
from dataclasses import dataclass, field
from typing import Optional

import can

# ---------------------------------------------------------------------------
# RS04 量程（手册 §5.2 / §1）
# ---------------------------------------------------------------------------
P_MIN, P_MAX = -12.57, 12.57          # ±4π rad
V_MIN, V_MAX = -15.0, 15.0            # rad/s
T_MIN, T_MAX = -120.0, 120.0          # Nm
KP_MIN, KP_MAX = 0.0, 5000.0
KD_MIN, KD_MAX = 0.0, 100.0

# 通信类型
GET_ID = 0
MIT = 1
STATUS = 2
ENABLE = 3
DISABLE = 4
SET_ZERO = 6
SET_ID = 7
READ_PARAM = 17
WRITE_PARAM = 18
FAULT = 21
SAVE_PARAM = 22
SET_BAUDRATE = 23
ACTIVE_REPORT = 24
SET_PROTOCOL = 25

# run_mode
MODE_MIT = 0
MODE_PP = 1
MODE_VELOCITY = 2
MODE_CURRENT = 3
MODE_CSP = 5

MODE_NAMES = {0: "MIT", 1: "PP", 2: "VEL", 3: "CUR", 5: "CSP"}

# 常用参数 index
P_RUN_MODE = 0x7005
P_IQ_REF = 0x7006
P_SPD_REF = 0x700A
P_LIMIT_TORQUE = 0x700B
P_LOC_REF = 0x7016
P_LIMIT_SPD = 0x7017
P_LIMIT_CUR = 0x7018
P_MECH_POS = 0x7019
P_MECH_VEL = 0x701B
P_VBUS = 0x701C
P_LOC_KP = 0x701E
P_SPD_KP = 0x701F
P_SPD_KI = 0x7020
P_ACC_RAD = 0x7022
P_VEL_MAX = 0x7024
P_ACC_SET = 0x7025
P_EPSCAN_TIME = 0x7026
P_CAN_TIMEOUT = 0x7028
P_ZERO_STA = 0x7029
P_DAMPER = 0x702A
P_ADD_OFFSET = 0x702B
P_ALVEOLOUS_OPEN = 0x702C
P_IQ_TEST = 0x702D
P_DCC_SET = 0x702E


@dataclass
class Status:
    position: float
    velocity: float
    torque: float
    temperature: float
    motor_id: int
    mode: int = -1
    flags: dict = field(default_factory=dict)

    def __str__(self) -> str:
        bad = [k for k, v in self.flags.items() if v]
        tail = f"  [!] {','.join(bad)}" if bad else ""
        mode = MODE_NAMES.get(self.mode, str(self.mode))
        return (
            f"id={self.motor_id} mode={mode} pos={self.position:+.3f}rad "
            f"vel={self.velocity:+.3f}rad/s tau={self.torque:+.2f}Nm "
            f"temp={self.temperature:.1f}C{tail}"
        )


def float_to_uint(x: float, x_min: float, x_max: float, bits: int = 16) -> int:
    """手册附录原函数：把浮点映射到 0..(2^bits-1)。"""
    span = x_max - x_min
    x = max(x_min, min(x_max, x))
    return int((x - x_min) * ((1 << bits) - 1) / span)


class RobStride:
    def __init__(self, bus: can.BusABC, motor_id: int, host_id: int = 0xFD):
        self.bus = bus
        self.motor_id = motor_id
        self.host_id = host_id

    # ------------------------------------------------------------------
    @classmethod
    def open(
        cls,
        channel: str = "can0",
        interface: str = "socketcan",
        bitrate: int = 1_000_000,
        motor_id: int = 1,
        host_id: int = 0xFD,
    ) -> "RobStride":
        bus = can.interface.Bus(channel=channel, interface=interface, bitrate=bitrate)
        return cls(bus, motor_id, host_id)

    def close(self) -> None:
        try:
            self.disable()
        except Exception:
            pass
        finally:
            self.bus.shutdown()

    # ------------------------------------------------------------------
    # 底层
    # ------------------------------------------------------------------
    def _send(self, comm_type: int, extra: int, data: bytes) -> None:
        ext_id = ((comm_type & 0x1F) << 24) | ((extra & 0xFFFF) << 8) | (self.motor_id & 0xFF)
        self.bus.send(
            can.Message(arbitration_id=ext_id, is_extended_id=True, data=data, dlc=len(data))
        )

    def _send_byte_cmd(self, comm_type: int, cmd: int) -> None:
        """类型 23/24/25 共用格式：01 02 03 04 05 06 CMD 00。"""
        self._send(comm_type, self.host_id, bytes([1, 2, 3, 4, 5, 6, cmd & 0xFF, 0]))

    def _recv(self, timeout: float, want=(STATUS, FAULT)) -> Optional[can.Message]:
        deadline = time.monotonic() + timeout
        while True:
            remain = deadline - time.monotonic()
            if remain <= 0:
                return None
            msg = self.bus.recv(timeout=remain)
            if msg is None:
                return None
            if not msg.is_extended_id:
                continue
            comm = (msg.arbitration_id >> 24) & 0x1F
            extra = (msg.arbitration_id >> 8) & 0xFFFF
            if (extra & 0xFF) != self.motor_id:
                continue
            if comm not in want:
                continue
            return msg

    # ------------------------------------------------------------------
    # 使能 / 停止 / 零点
    # ------------------------------------------------------------------
    def enable(self, timeout: float = 0.2) -> Optional[Status]:
        self._send(ENABLE, self.host_id, bytes(8))
        return self.read_status(timeout)

    def disable(self, timeout: float = 0.2) -> Optional[Status]:
        self._send(DISABLE, self.host_id, bytes(8))
        return self.read_status(timeout)

    def clear_fault(self, timeout: float = 0.2) -> Optional[Status]:
        """停止帧 Byte0=1：清故障（手册 §5.5）。"""
        self._send(DISABLE, self.host_id, b"\x01" + bytes(7))
        return self.read_status(timeout)

    def set_zero(self) -> None:
        """设置机械零位；CSP / 运控模式有效，PP 模式会被屏蔽。"""
        self._send(SET_ZERO, self.host_id, b"\x01" + bytes(7))

    # ------------------------------------------------------------------
    # MIT 控制
    # ------------------------------------------------------------------
    def mit(
        self,
        position: float = 0.0,
        velocity: float = 0.0,
        kp: float = 0.0,
        kd: float = 0.0,
        torque: float = 0.0,
    ) -> None:
        """运控模式：t_ref = Kd*(v_set-v) + Kp*(p_set-p) + t_ff。"""
        data = struct.pack(
            ">HHHH",
            float_to_uint(position, P_MIN, P_MAX, 16),
            float_to_uint(velocity, V_MIN, V_MAX, 16),
            float_to_uint(kp, KP_MIN, KP_MAX, 16),
            float_to_uint(kd, KD_MIN, KD_MAX, 16),
        )
        self._send(MIT, float_to_uint(torque, T_MIN, T_MAX, 16), data)

    # ------------------------------------------------------------------
    # 状态
    # ------------------------------------------------------------------
    def read_status(self, timeout: float = 0.05) -> Optional[Status]:
        msg = self._recv(timeout)
        if msg is None:
            return None
        comm = (msg.arbitration_id >> 24) & 0x1F
        extra = (msg.arbitration_id >> 8) & 0xFFFF
        payload = bytes(msg.data)

        if comm == FAULT:
            fault, warn = struct.unpack("<LL", payload)
            return Status(0.0, 0.0, 0.0, 0.0, extra & 0xFF, flags={"fault_raw": fault, "warn_raw": warn})

        p_u, v_u, t_u, temp_u = struct.unpack(">HHHH", payload)
        return Status(
            position=(p_u / 0x7FFF - 1.0) * P_MAX,
            velocity=(v_u / 0x7FFF - 1.0) * V_MAX,
            torque=(t_u / 0x7FFF - 1.0) * T_MAX,
            temperature=temp_u * 0.1,
            motor_id=extra & 0xFF,
            mode=(extra >> 14) & 0x03,
            flags={
                "undervoltage": bool((extra >> 8) & 1),
                "phase_current": bool((extra >> 9) & 1),
                "overtemperature": bool((extra >> 10) & 1),
                "encoder": bool((extra >> 11) & 1),
                "stall": bool((extra >> 12) & 1),
                "uncalibrated": bool((extra >> 13) & 1),
            },
        )

    # ------------------------------------------------------------------
    # 参数读写（小端）
    # ------------------------------------------------------------------
    def write_param(self, param_id: int, value, fmt: str = "<f") -> Optional[Status]:
        payload = struct.pack(fmt, value).ljust(4, b"\x00")[:4]
        data = struct.pack("<HH", param_id, 0) + payload
        self._send(WRITE_PARAM, self.host_id, data)
        return self.read_status(0.1)

    def read_param(self, param_id: int, fmt: str = "<f", timeout: float = 0.2):
        data = struct.pack("<HHL", param_id, 0, 0)
        self._send(READ_PARAM, self.host_id, data)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if msg is None:
                return None
            if not msg.is_extended_id:
                continue
            comm = (msg.arbitration_id >> 24) & 0x1F
            extra = (msg.arbitration_id >> 8) & 0xFFFF
            if comm != READ_PARAM or (extra & 0xFF) != self.motor_id:
                continue
            payload = bytes(msg.data)[4:]
            return struct.unpack(fmt, payload[: struct.calcsize(fmt)])[0]
        return None

    def set_mode(self, mode: int) -> Optional[Status]:
        """0=运控, 1=位置PP, 2=速度, 3=电流, 5=位置CSP。切换前先停止。"""
        return self.write_param(P_RUN_MODE, mode, "<b")

    def save(self) -> None:
        """保存参数（手册要求数据区为 01 02 03 04 05 06 07 08）。"""
        self._send(SAVE_PARAM, self.host_id, bytes(range(1, 9)))

    def set_can_id(self, new_id: int) -> None:
        """新 ID 在 bit23~16（手册 §5.7），不是 bit23~8。"""
        if not 1 <= new_id <= 0x7F:
            raise ValueError("new_id 建议 1~0x7F")
        ext_id = (
            (SET_ID << 24)
            | ((new_id & 0xFF) << 16)
            | ((self.host_id & 0xFF) << 8)
            | (self.motor_id & 0xFF)
        )
        self.bus.send(can.Message(arbitration_id=ext_id, is_extended_id=True, data=bytes(8)))

    def set_baudrate(self, code: int) -> None:
        """1=1M, 2=500K, 3=250K, 4=125K；重新上电生效。"""
        self._send_byte_cmd(SET_BAUDRATE, code)

    def set_active_report(self, enable: bool, epscan_time: Optional[int] = None) -> None:
        """主动上报开关；开启后默认 10ms，可用 EPScan_time 调整。"""
        self._send_byte_cmd(ACTIVE_REPORT, 1 if enable else 0)
        if enable and epscan_time is not None:
            self.write_param(P_EPSCAN_TIME, epscan_time, "<H")

    def set_protocol(self, code: int) -> None:
        """0=私有, 1=Canopen, 2=MIT；重新上电生效。"""
        self._send_byte_cmd(SET_PROTOCOL, code)


# ---------------------------------------------------------------------------
# 演示：正弦位置跟踪
# ---------------------------------------------------------------------------
def main() -> None:
    ap = argparse.ArgumentParser(description="RobStride RS04 正弦位置跟踪 demo")
    ap.add_argument("--interface", default="socketcan", help="socketcan / slcan / pcan")
    ap.add_argument("--channel", default="can0", help="can0 或 COM5")
    ap.add_argument("--bitrate", type=int, default=1_000_000)
    ap.add_argument("--id", type=int, default=1, dest="motor_id")
    ap.add_argument("--kp", type=float, default=20.0)
    ap.add_argument("--kd", type=float, default=0.5)
    ap.add_argument("--amp", type=float, default=0.5, help="正弦幅值 rad")
    ap.add_argument("--freq", type=float, default=0.2, help="正弦频率 Hz")
    ap.add_argument("--duration", type=float, default=10.0, help="运行秒数")
    args = ap.parse_args()

    dev = RobStride.open(args.channel, args.interface, args.bitrate, args.motor_id)
    try:
        st = dev.enable()
        print("enabled:", st)
        if st is None:
            print("没收到状态帧：检查波特率 1Mbps、扩展帧、120Ω 终端电阻、电机 ID")
            return

        t0 = time.monotonic()
        while True:
            t = time.monotonic() - t0
            if t >= args.duration:
                break
            target = args.amp * math.sin(2.0 * math.pi * args.freq * t)
            dev.mit(position=target, kp=args.kp, kd=args.kd)
            st = dev.read_status(0.01)
            if st is not None:
                print(f"t={t:5.2f}s target={target:+.3f} | {st}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        dev.close()
        print("已停止")


if __name__ == "__main__":
    main()
