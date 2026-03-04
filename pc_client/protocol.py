import struct
from dataclasses import dataclass
from typing import Iterable, List, Optional

SOF = 0xAA55

FLAG_VACUUM = 0x80
FLAG_GRAVITY_COMP = 0x40


@dataclass
class TrajPoint:
    """轨迹点数据结构。"""
    index: int
    q1: float
    q2: float
    q3: float
    dq1: float
    dq2: float
    dq3: float
    flags: int


@dataclass
class Feedback:
    """下位机反馈数据结构。"""
    q1: float
    q2: float
    q3: float
    dq1: float
    dq2: float
    dq3: float
    pressure_kpa: float


def crc16_ibm(data: bytes) -> int:
    """计算 CRC16-IBM 校验值。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def pack_traj_points(points: Iterable[TrajPoint]) -> List[bytes]:
    """打包轨迹点列表为多帧数据。"""
    frames = []
    for p in points:
        frame = struct.pack(
            "<HHffffffB",
            SOF,
            p.index & 0xFFFF,
            p.q1,
            p.q2,
            p.q3,
            p.dq1,
            p.dq2,
            p.dq3,
            p.flags & 0xFF,
        )
        frames.append(frame)
    return frames


def try_parse_feedback(data: bytes) -> Optional[Feedback]:
    """从字节流中尝试解析反馈帧。"""
    frame_len = 2 + struct.calcsize("<7f")
    if len(data) < frame_len:
        return None

    idx = data.find(struct.pack("<H", SOF))
    if idx < 0 or len(data) < idx + frame_len:
        return None

    sof = struct.unpack("<H", data[idx : idx + 2])[0]
    if sof != SOF:
        return None

    q1, q2, q3, dq1, dq2, dq3, pressure_kpa = struct.unpack(
        "<7f", data[idx + 2 : idx + frame_len]
    )
    return Feedback(
        q1=q1,
        q2=q2,
        q3=q3,
        dq1=dq1,
        dq2=dq2,
        dq3=dq3,
        pressure_kpa=pressure_kpa,
    )
