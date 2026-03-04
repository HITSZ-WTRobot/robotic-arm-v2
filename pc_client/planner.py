import math
import random
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple

import numpy as np

try:
    import toppra as ta
    import toppra.algorithm as algo
    import toppra.constraint as constraint
except ImportError:
    ta = None
    algo = None
    constraint = None

from .kinematics import ArmKinematics


@dataclass
class JointLimits:
    """关节约束参数。"""
    q_min: List[float]
    q_max: List[float]
    v_max: List[float]
    a_max: List[float]


@dataclass
class RectObstacle:
    """二维矩形障碍物。"""
    cx: float
    cy: float
    w: float
    h: float


class Planner:
    """基于 RRT* 的路径规划与 TOPPRA 时间参数化。"""
    def __init__(self, kin: ArmKinematics, limits: JointLimits) -> None:
        self.kin = kin
        self.limits = limits
        self.obstacles: List[RectObstacle] = []
        self.segment_step_deg = 0.5

    def set_obstacles(self, obstacles: Iterable[RectObstacle]) -> None:
        """设置环境中的固定障碍物。"""
        self.obstacles = list(obstacles)

    def rrt_star(self, q_start: List[float], q_goal: List[float], max_iter: int = 2000) -> List[List[float]]:
        """在关节空间用 RRT* 搜索可行路径。"""
        if not self._config_ok(q_start) or not self._config_ok(q_goal):
            raise RuntimeError("start/goal invalid")
        nodes = [RRTNode(q_start, parent=None, cost=0.0)]
        for _ in range(max_iter):
            q_rand = self._sample(q_goal)
            nearest = min(nodes, key=lambda n: dist(n.q, q_rand))
            q_new = steer(nearest.q, q_rand, step=5.0)
            if not self._segment_ok(nearest.q, q_new):
                continue

            new_node = RRTNode(q_new, parent=nearest, cost=nearest.cost + dist(nearest.q, q_new))
            near_nodes = [n for n in nodes if dist(n.q, q_new) < 15.0]
            for n in near_nodes:
                if self._segment_ok(n.q, q_new):
                    c = n.cost + dist(n.q, q_new)
                    if c < new_node.cost:
                        new_node.parent = n
                        new_node.cost = c

            nodes.append(new_node)
            for n in near_nodes:
                if self._segment_ok(q_new, n.q):
                    c = new_node.cost + dist(q_new, n.q)
                    if c < n.cost:
                        n.parent = new_node
                        n.cost = c

            if dist(q_new, q_goal) < 5.0 and self._segment_ok(q_new, q_goal):
                goal_node = RRTNode(q_goal, parent=new_node, cost=new_node.cost + dist(q_new, q_goal))
                return backtrack(goal_node)

        return [q_start, q_goal]

    def simplify_path(self, path: List[List[float]], attempts: int = 200) -> List[List[float]]:
        """随机捷径化简路径，去除多余拐点。"""
        if len(path) <= 2:
            return path

        path = path[:]
        for _ in range(attempts):
            if len(path) <= 2:
                break
            i = random.randint(0, len(path) - 2)
            j = random.randint(i + 1, len(path) - 1)
            if j == i + 1:
                continue
            if self._segment_ok(path[i], path[j]):
                path = path[: i + 1] + path[j:]
        return path

    def toppra_time_parameterize(
        self, path: List[List[float]], dt: float = 0.001
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """使用 TOPPRA 对路径进行时间参数化。"""
        if ta is None:
            raise RuntimeError("toppra is not available")

        waypoints = np.asarray(path, dtype=float)
        ss = np.linspace(0.0, 1.0, len(waypoints))
        spline = ta.SplineInterpolator(ss, waypoints)

        vlim = np.asarray(self.limits.v_max, dtype=float)
        alim = np.asarray(self.limits.a_max, dtype=float)

        vlim = np.vstack((-vlim, vlim)).T
        alim = np.vstack((-alim, alim)).T

        pc_vel = constraint.JointVelocityConstraint(vlim)
        pc_acc = constraint.JointAccelerationConstraint(alim)
        instance = algo.TOPPRA([pc_vel, pc_acc], spline)

        traj = instance.compute_trajectory(0.0, 0.0)
        t_grid = np.arange(0.0, traj.duration + dt, dt)
        q = traj(t_grid)
        dq = traj(t_grid, 1)
        if not self._trajectory_ok(q):
            raise RuntimeError("trajectory invalid")
        return t_grid, q, dq

    def _sample(self, q_goal: List[float]) -> List[float]:
        """采样关节空间点，带一定目标偏置。"""
        if random.random() < 0.15:
            return q_goal[:]
        return [
            random.uniform(self.limits.q_min[i], self.limits.q_max[i])
            for i in range(len(self.limits.q_min))
        ]

    def _segment_ok(self, qa: List[float], qb: List[float]) -> bool:
        """检查两点之间的插值路径是否碰撞。"""
        steps = max(2, int(dist(qa, qb) / self.segment_step_deg))
        for i in range(steps + 1):
            t = i / steps
            q = lerp(qa, qb, t)
            if not self._config_ok(q):
                return False
        return True

    def _config_ok(self, q: List[float]) -> bool:
        """检查单个关节姿态是否合法且不碰撞。"""
        for i, v in enumerate(q):
            if v < self.limits.q_min[i] or v > self.limits.q_max[i]:
                return False

        links = self._fk_links(q)
        for obs in self.obstacles:
            for seg in links:
                if segment_intersects_rect(seg[0], seg[1], obs):
                    return False
        return True

    def _trajectory_ok(self, q: np.ndarray) -> bool:
        """检查整段轨迹是否越界或碰撞。"""
        for i in range(q.shape[0]):
            qi = [float(q[i, 0]), float(q[i, 1]), float(q[i, 2])]
            if not self._config_ok(qi):
                return False
        return True

    def _fk_links(self, q: List[float]) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
        """用简化 FK 计算连杆线段。"""
        q1, q2, q3 = q
        q12 = math.radians(q1 + q2)
        q123 = math.radians(q1 + q2 + q3)
        q1r = math.radians(q1)

        p0 = (0.0, 0.0)
        p1 = (self.kin.cfg.l1 * math.cos(q1r), self.kin.cfg.l1 * math.sin(q1r))
        p2 = (p1[0] + self.kin.cfg.l2 * math.cos(q12), p1[1] + self.kin.cfg.l2 * math.sin(q12))
        p3 = (p2[0] + self.kin.cfg.l3 * math.cos(q123), p2[1] + self.kin.cfg.l3 * math.sin(q123))

        return [(p0, p1), (p1, p2), (p2, p3)]


def dist(a: List[float], b: List[float]) -> float:
    """关节空间欧氏距离。"""
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(len(a))))


def lerp(a: List[float], b: List[float], t: float) -> List[float]:
    """线性插值。"""
    return [a[i] + (b[i] - a[i]) * t for i in range(len(a))]


def steer(a: List[float], b: List[float], step: float) -> List[float]:
    """按固定步长向目标推进。"""
    d = dist(a, b)
    if d <= step or d <= 1e-6:
        return b[:]
    scale = step / d
    return [a[i] + (b[i] - a[i]) * scale for i in range(len(a))]


@dataclass
class RRTNode:
    """RRT* 树节点。"""
    q: List[float]
    parent: Optional["RRTNode"]
    cost: float


def backtrack(node: RRTNode) -> List[List[float]]:
    """从目标节点回溯得到路径。"""
    path = []
    cur = node
    while cur is not None:
        path.append(cur.q)
        cur = cur.parent
    return list(reversed(path))


def segment_intersects_rect(a: Tuple[float, float], b: Tuple[float, float], obs: RectObstacle) -> bool:
    """线段与矩形碰撞检测。"""
    left = obs.cx - obs.w * 0.5
    right = obs.cx + obs.w * 0.5
    bottom = obs.cy - obs.h * 0.5
    top = obs.cy + obs.h * 0.5

    if point_in_rect(a, left, right, bottom, top) or point_in_rect(b, left, right, bottom, top):
        return True

    rect_edges = [
        ((left, bottom), (right, bottom)),
        ((right, bottom), (right, top)),
        ((right, top), (left, top)),
        ((left, top), (left, bottom)),
    ]

    for p1, p2 in rect_edges:
        if segments_intersect(a, b, p1, p2):
            return True

    return False


def point_in_rect(p: Tuple[float, float], left: float, right: float, bottom: float, top: float) -> bool:
    """点是否在矩形内。"""
    return left <= p[0] <= right and bottom <= p[1] <= top


def segments_intersect(a1: Tuple[float, float], a2: Tuple[float, float], b1: Tuple[float, float], b2: Tuple[float, float]) -> bool:
    """两线段是否相交。"""
    return (ccw(a1, b1, b2) != ccw(a2, b1, b2)) and (ccw(a1, a2, b1) != ccw(a1, a2, b2))


def ccw(a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> bool:
    """判断三点是否逆时针。"""
    return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])
