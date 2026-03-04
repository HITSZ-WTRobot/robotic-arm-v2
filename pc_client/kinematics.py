import math
from dataclasses import dataclass
from typing import List, Tuple


def deg2rad(deg: float) -> float:
    """角度转弧度。"""
    return deg * math.pi / 180.0


def rad2deg(rad: float) -> float:
    """弧度转角度。"""
    return rad * 180.0 / math.pi


@dataclass
class ArmConfig:
    """机械臂参数配置。"""
    l1: float = 0.346
    l2: float = 0.382
    l3: float = 0.093
    offset_1: float = 0.0
    offset_2: float = -164.0
    offset_3: float = 90.0


class ArmKinematics:
    """3DOF 平面机械臂运动学。"""
    def __init__(self, cfg: ArmConfig) -> None:
        self.cfg = cfg

    def fk(self, q1_deg: float, q2_deg: float, q3_deg: float) -> Tuple[float, float, float]:
        """正解：由关节角得到末端位姿(x, y, phi)。"""
        q1 = deg2rad(q1_deg)
        q2 = deg2rad(q2_deg)
        q3 = deg2rad(q3_deg)

        q12 = q1 + q2
        q123 = q12 + q3

        x = self.cfg.l1 * math.cos(q1) + self.cfg.l2 * math.cos(q12) + self.cfg.l3 * math.cos(q123)
        y = self.cfg.l1 * math.sin(q1) + self.cfg.l2 * math.sin(q12) + self.cfg.l3 * math.sin(q123)
        phi = q123
        return x, y, phi

    def ik(self, x: float, y: float, phi_rad: float, elbow_up: bool = True) -> Tuple[float, float, float]:
        """逆解：给定末端位姿与姿态，求关节角。"""
        wx = x - self.cfg.l3 * math.cos(phi_rad)
        wy = y - self.cfg.l3 * math.sin(phi_rad)

        c2 = (wx * wx + wy * wy - self.cfg.l1 * self.cfg.l1 - self.cfg.l2 * self.cfg.l2) / (
            2.0 * self.cfg.l1 * self.cfg.l2
        )
        c2 = max(-1.0, min(1.0, c2))
        s2 = math.sqrt(max(0.0, 1.0 - c2 * c2))
        if not elbow_up:
            s2 = -s2

        q2 = math.atan2(s2, c2)
        k1 = self.cfg.l1 + self.cfg.l2 * c2
        k2 = self.cfg.l2 * s2
        q1 = math.atan2(wy, wx) - math.atan2(k2, k1)
        q3 = phi_rad - q1 - q2

        return rad2deg(q1), rad2deg(q2), rad2deg(q3)

    def joint_positions(self, q1_deg: float, q2_deg: float, q3_deg: float) -> List[Tuple[float, float]]:
        """计算基座到末端的各关节坐标点。"""
        q1 = deg2rad(q1_deg)
        q2 = deg2rad(q2_deg)
        q3 = deg2rad(q3_deg)

        q12 = q1 + q2
        q123 = q12 + q3

        p0 = (0.0, 0.0)
        p1 = (self.cfg.l1 * math.cos(q1), self.cfg.l1 * math.sin(q1))
        p2 = (p1[0] + self.cfg.l2 * math.cos(q12), p1[1] + self.cfg.l2 * math.sin(q12))
        p3 = (p2[0] + self.cfg.l3 * math.cos(q123), p2[1] + self.cfg.l3 * math.sin(q123))
        return [p0, p1, p2, p3]
