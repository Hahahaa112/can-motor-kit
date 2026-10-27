"""
RobStride RS04 CAN 驱动（python-can）

协议依据：RobStride 官方手册 + Seeed RobStride_Control 参考实现。

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
# RS04 量程（其他型号见 docs/CAN协议.md）
# ---------------------------------------------------------------------------
P_MAX = 4.0 * math.pi      # ±12.566 rad
V_MAX = 15.0               # ±15 rad/s
T_MAX = 120.0              # ±120 Nm
KP_MAX = 5000.0
KD_MAX = 100.0

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


@dataclass
class Status:
    position: float
    velocity: float
    torque: float
    temperature: float
    motor_id: int
    flags: dict = field(default_factory=dict)

    def __str__(self) -> str:
        f = self.flags
        bad = [k for k, v in f.items() if v]
        tail = f"  [!] {','.join(bad)}" if bad else ""
        return (
            f"id={self.motor_id} pos={self.position:+.3f}rad "
            f"vel={self.velocity:+.3f}rad/s tau={self.torque:+.2f}Nm "
            f"temp={self.temperature:.1f}C{tail}"
        )


def _clip(x: float, limit: float) -> float:
    return max(-limit, min(limit, x))


def _encode(x: float, limit: float) -> int:
    """浮点 -> 0..65535，用于位置/速度/力矩。"""
    raw = int((x / limit + 1.0) * 0x7FFF)
    return max(0, min(0xFFFF, raw))


class RobStride:
    def __init__(self, bus: can.BusABC, motor_id: int, host_id: int = 0xFD):
        self.bus = bus
        self.motor_id = motor_id
        self.host_id = host_id

    # ------------------------------------------------------------------
    # 构造 / 关闭
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
    # 底层收发
    # ------------------------------------------------------------------
    def _send(self, comm_type: int, extra: int, data: bytes) -> None:
        ext_id = ((comm_type & 0x1F) << 24) | ((extra & 0xFFFF) << 8) | (self.motor_id & 0xFF)
        msg = can.Message(
            arbitration_id=ext_id,
            is_extended_id=True,
            data=data,
            dlc=len(data),
        )
        self.bus.send(msg)

    def _recv(self, timeout: float) -> Optional[can.Message]:
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
            if comm not in (STATUS, FAULT):
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

    def set_zero(self) -> None:
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
        p = _clip(position, P_MAX)
        v = _clip(velocity, V_MAX)
        t = _clip(torque, T_MAX)
        kp = max(0.0, min(KP_MAX, kp))
        kd = max(0.0, min(KD_MAX, kd))

        data = struct.pack(
            ">HHHH",
            _encode(p, P_MAX),
            _encode(v, V_MAX),
            int(kp / KP_MAX * 0xFFFF),
            int(kd / KD_MAX * 0xFFFF),
        )
        self._send(MIT, _encode(t, T_MAX), data)

    # ------------------------------------------------------------------
    # 状态读取
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
            return Status(0.0, 0.0, 0.0, 0.0, extra & 0xFF, {"fault_raw": fault, "warn_raw": warn})

        p_u, v_u, t_u, temp_u = struct.unpack(">HHHH", payload)
        return Status(
            position=(p_u / 0x7FFF - 1.0) * P_MAX,
            velocity=(v_u / 0x7FFF - 1.0) * V_MAX,
            torque=(t_u / 0x7FFF - 1.0) * T_MAX,
            temperature=temp_u * 0.1,
            motor_id=extra & 0xFF,
            flags={
                "undervoltage": bool((extra >> 8) & 1),
                "overcurrent": bool((extra >> 9) & 1),
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
        """0=MIT, 1=位置, 2=速度。"""
        return self.write_param(0x7005, mode, "<b")

    def save(self) -> None:
        self._send(SAVE_PARAM, self.host_id, bytes(8))


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
            print("没收到状态帧：检查波特率、扩展帧、终端电阻、电机 ID")
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
