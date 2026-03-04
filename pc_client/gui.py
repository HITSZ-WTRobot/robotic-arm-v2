import bisect
import math
import sys
import time
from dataclasses import dataclass
from typing import List

import pyqtgraph as pg
from PyQt6 import QtCore, QtWidgets

from .arm_client import ArmClient, SerialConfig
from .kinematics import ArmConfig, ArmKinematics
from .planner import JointLimits, Planner, RectObstacle
from .protocol import FLAG_GRAVITY_COMP, FLAG_VACUUM, TrajPoint


@dataclass
class UiState:
    """GUI 状态缓存。"""
    q1: float = 0.0
    q2: float = 0.0
    q3: float = 0.0
    dq1: float = 0.0
    dq2: float = 0.0
    dq3: float = 0.0
    pressure_kpa: float = 0.0


class MainWindow(QtWidgets.QWidget):
    """机械臂控制 GUI。"""
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Arm Client")

        self.kin = ArmKinematics(ArmConfig())
        self.limits = JointLimits(
            q_min=[0.0, -170.0, -170.0],
            q_max=[200.0, 170.0, 170.0],
            v_max=[500.0, 200.0, 500.0],
            a_max=[50.0, 120.0, 30.0],
        )
        self.planner = Planner(self.kin, self.limits)
        self.planner.set_obstacles([RectObstacle(0.0, 0.5, 0.2, 0.1)])

        self.client: ArmClient | None = None
        self.state = UiState(q1=180, q2=self.kin.cfg.offset_2, q3=self.kin.cfg.offset_3)  # type: ignore
        self._preview_points: List[TrajPoint] = []
        self._preview_path_xy: List[tuple[float, float]] = []
        self._preview_q: List[tuple[float, float, float]] = []
        self._preview_dq: List[tuple[float, float, float]] = []
        self._preview_t: List[float] = []
        self._preview_start: float = 0.0
        self._preview_active = False

        self._build_ui()
        self._update_plot()

    def _build_ui(self) -> None:
        """初始化界面控件。"""
        layout = QtWidgets.QHBoxLayout(self)

        left_panel = QtWidgets.QVBoxLayout()
        right_panel = QtWidgets.QVBoxLayout()

        self.plot = pg.PlotWidget()
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True)
        left_panel.addWidget(self.plot)

        self.traj_q_plot = pg.PlotWidget()
        self.traj_q_plot.showGrid(x=True, y=True)
        self.traj_q_plot.setLabel("left", "Angle (deg)")
        self.traj_q_plot.setLabel("bottom", "Time (s)")
        self.traj_q_plot.addLegend()
        right_panel.addWidget(self.traj_q_plot)

        self.traj_dq_plot = pg.PlotWidget()
        self.traj_dq_plot.showGrid(x=True, y=True)
        self.traj_dq_plot.setLabel("left", "Velocity (deg/s)")
        self.traj_dq_plot.setLabel("bottom", "Time (s)")
        self.traj_dq_plot.addLegend()
        right_panel.addWidget(self.traj_dq_plot)

        self.link_plot = self.plot.plot([], [], pen=pg.mkPen("w", width=3))
        self.obs_plot = self.plot.plot([], [], pen=pg.mkPen("r", width=2))
        self.path_plot = self.plot.plot([], [], pen=pg.mkPen("y", width=2, style=QtCore.Qt.PenStyle.DashLine))
        self.joint_scatter = pg.ScatterPlotItem(size=8, brush=pg.mkBrush("c"))
        self.plot.addItem(self.joint_scatter)
        self.reach_outer_plot = self.plot.plot([], [], pen=pg.mkPen("#666", width=1))
        self.reach_inner_plot = self.plot.plot([], [], pen=pg.mkPen("#444", width=1, style=QtCore.Qt.PenStyle.DotLine))

        self.q1_curve = self.traj_q_plot.plot([], [], pen=pg.mkPen("#5ac8fa", width=2), name="q1")
        self.q2_curve = self.traj_q_plot.plot([], [], pen=pg.mkPen("#ffd60a", width=2), name="q2")
        self.q3_curve = self.traj_q_plot.plot([], [], pen=pg.mkPen("#ff6b6b", width=2), name="q3")

        self.dq1_curve = self.traj_dq_plot.plot([], [], pen=pg.mkPen("#5ac8fa", width=2), name="dq1")
        self.dq2_curve = self.traj_dq_plot.plot([], [], pen=pg.mkPen("#ffd60a", width=2), name="dq2")
        self.dq3_curve = self.traj_dq_plot.plot([], [], pen=pg.mkPen("#ff6b6b", width=2), name="dq3")

        total_len = self.kin.cfg.l1 + self.kin.cfg.l2 + self.kin.cfg.l3
        self.plot.setXRange(-total_len, total_len)
        self.plot.setYRange(-0.2 * total_len, total_len)
        self._update_reachability_plot()

        form = QtWidgets.QFormLayout()
        self.port_edit = QtWidgets.QLineEdit("/dev/tty.usbserial")
        self.baud_edit = QtWidgets.QLineEdit("115200")
        self.btn_connect = QtWidgets.QPushButton("Connect")
        self.btn_preview = QtWidgets.QPushButton("Preview")
        self.btn_send = QtWidgets.QPushButton("Send Path")
        self.chk_vacuum = QtWidgets.QCheckBox("Vacuum")
        self.chk_grav = QtWidgets.QCheckBox("Gravity Comp")
        self.chk_elbow_up = QtWidgets.QCheckBox("Elbow Up")
        self.chk_elbow_up.setChecked(True)
        self.target_x = QtWidgets.QLineEdit("0.3")
        self.target_y = QtWidgets.QLineEdit("0.2")
        self.target_phi = QtWidgets.QLineEdit("90")

        form.addRow("Port", self.port_edit)
        form.addRow("Baud", self.baud_edit)
        form.addRow(self.btn_connect)
        form.addRow("Target X", self.target_x)
        form.addRow("Target Y", self.target_y)
        form.addRow("Target Phi(deg)", self.target_phi)
        form.addRow(self.chk_vacuum, self.chk_grav)
        form.addRow(self.chk_elbow_up)
        form.addRow(self.btn_preview, self.btn_send)
        left_panel.addLayout(form)

        layout.addLayout(left_panel, 2)
        layout.addLayout(right_panel, 1)

        self.btn_connect.clicked.connect(self._on_connect)
        self.btn_preview.clicked.connect(self._on_preview)
        self.btn_send.clicked.connect(self._on_send)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update_plot)
        self.timer.start(30)

        self.plot.scene().sigMouseClicked.connect(self._on_plot_click)

    def _on_connect(self) -> None:
        """连接/断开串口。"""
        if self.client:
            self.client.close()
            self.client = None
            self.btn_connect.setText("Connect")
            return

        port = self.port_edit.text().strip()
        baud = int(self.baud_edit.text())
        self.client = ArmClient(SerialConfig(port=port, baud=baud))
        self.client.set_feedback_callback(self._on_feedback)
        self.client.connect()
        self.btn_connect.setText("Disconnect")

    def _on_send(self) -> None:
        """发送已预览的轨迹。"""
        if not self.client:
            return
        if not self._preview_points:
            return
        self.client.send_points(self._preview_points)

    def _on_preview(self) -> None:
        """生成轨迹并启动预览动画。"""
        try:
            points, path_xy, preview_q, preview_dq, preview_t = self._build_points()
        except RuntimeError as exc:
            QtWidgets.QMessageBox.warning(self, "规划失败", str(exc))
            self._preview_points = []
            self._preview_path_xy = []
            self._preview_q = []
            self._preview_t = []
            self._preview_active = False
            self.path_plot.setData([], [])
            return

        self._preview_points = points
        self._preview_path_xy = path_xy
        self._preview_q = preview_q
        self._preview_dq = preview_dq
        self._preview_t = preview_t
        self._preview_start = time.time()
        self._preview_active = bool(self._preview_q)
        if self._preview_path_xy:
            xs = [p[0] for p in self._preview_path_xy]
            ys = [p[1] for p in self._preview_path_xy]
            self.path_plot.setData(xs, ys)

        if self._preview_t:
            q1 = [v[0] for v in self._preview_q]
            q2 = [v[1] for v in self._preview_q]
            q3 = [v[2] for v in self._preview_q]
            dq1 = [v[0] for v in self._preview_dq]
            dq2 = [v[1] for v in self._preview_dq]
            dq3 = [v[2] for v in self._preview_dq]
            self.q1_curve.setData(self._preview_t, q1)
            self.q2_curve.setData(self._preview_t, q2)
            self.q3_curve.setData(self._preview_t, q3)
            self.dq1_curve.setData(self._preview_t, dq1)
            self.dq2_curve.setData(self._preview_t, dq2)
            self.dq3_curve.setData(self._preview_t, dq3)

    def _on_plot_click(self, event: QtCore.QEvent) -> None:
        """点击图表设置目标位置。"""
        if not self.plot.sceneBoundingRect().contains(event.scenePos()):
            return
        mouse_point = self.plot.plotItem.vb.mapSceneToView(event.scenePos())
        x = float(mouse_point.x())
        y = float(mouse_point.y())
        if not self._is_reachable_xy(x, y):
            return
        self.target_x.setText(f"{x:.3f}")
        self.target_y.setText(f"{y:.3f}")
        self._preview_points = []
        self._preview_q = []
        self._preview_dq = []
        self._preview_t = []
        self._preview_active = False
        self.path_plot.setData([], [])
        self.q1_curve.setData([], [])
        self.q2_curve.setData([], [])
        self.q3_curve.setData([], [])
        self.dq1_curve.setData([], [])
        self.dq2_curve.setData([], [])
        self.dq3_curve.setData([], [])

    def _build_points(
        self,
    ) -> tuple[
        List[TrajPoint],
        List[tuple[float, float]],
        List[tuple[float, float, float]],
        List[tuple[float, float, float]],
        List[float],
    ]:
        """生成轨迹点与预览数据。"""
        x = float(self.target_x.text())
        y = float(self.target_y.text())
        phi_deg = float(self.target_phi.text())
        phi_rad = math.radians(phi_deg)

        q_goal = list(self.kin.ik(x, y, phi_rad, elbow_up=self.chk_elbow_up.isChecked()))
        q_start = [self.state.q1, self.state.q2, self.state.q3]

        path = self.planner.rrt_star(q_start, q_goal)
        path = self.planner.simplify_path(path)
        t_grid, q, dq = self.planner.toppra_time_parameterize(path)

        flags = 0
        if self.chk_vacuum.isChecked():
            flags |= FLAG_VACUUM
        if self.chk_grav.isChecked():
            flags |= FLAG_GRAVITY_COMP

        points: List[TrajPoint] = []
        path_xy: List[tuple[float, float]] = []
        preview_q: List[tuple[float, float, float]] = []
        preview_dq: List[tuple[float, float, float]] = []
        preview_t: List[float] = []
        for i in range(len(q)):
            t_ms = int(round(t_grid[i] * 1000.0))
            points.append(
                TrajPoint(
                    index=i,
                    q1=float(q[i, 0]),
                    q2=float(q[i, 1]),
                    q3=float(q[i, 2]),
                    dq1=float(dq[i, 0]),
                    dq2=float(dq[i, 1]),
                    dq3=float(dq[i, 2]),
                    flags=flags,
                )
            )
            x_i, y_i, _ = self.kin.fk(float(q[i, 0]), float(q[i, 1]), float(q[i, 2]))
            path_xy.append((x_i, y_i))
            preview_q.append((float(q[i, 0]), float(q[i, 1]), float(q[i, 2])))
            preview_dq.append((float(dq[i, 0]), float(dq[i, 1]), float(dq[i, 2])))
            preview_t.append(float(t_grid[i]))
        return points, path_xy, preview_q, preview_dq, preview_t

    def _on_feedback(self, fb) -> None:
        """接收下位机反馈并更新状态。"""
        self.state.q1 = fb.q1
        self.state.q2 = fb.q2
        self.state.q3 = fb.q3
        self.state.dq1 = fb.dq1
        self.state.dq2 = fb.dq2
        self.state.dq3 = fb.dq3
        self.state.pressure_kpa = fb.pressure_kpa

    def _update_plot(self) -> None:
        """刷新机械臂与障碍物显示。"""
        q1, q2, q3 = self.state.q1, self.state.q2, self.state.q3
        if self._preview_active and self._preview_q and self._preview_t:
            elapsed = time.time() - self._preview_start
            idx = bisect.bisect_right(self._preview_t, elapsed) - 1
            if idx < 0:
                idx = 0
            if idx >= len(self._preview_q):
                idx = len(self._preview_q) - 1
                self._preview_active = False
            q1, q2, q3 = self._preview_q[idx]

        joints = self.kin.joint_positions(q1, q2, q3)
        xs = [p[0] for p in joints]
        ys = [p[1] for p in joints]

        self.link_plot.setData(xs, ys)
        self.joint_scatter.setData(xs, ys)

        if self.planner.obstacles:
            obs = self.planner.obstacles[0]
            left = obs.cx - obs.w * 0.5
            right = obs.cx + obs.w * 0.5
            bottom = obs.cy - obs.h * 0.5
            top = obs.cy + obs.h * 0.5
            self.obs_plot.setData([left, right, right, left, left], [bottom, bottom, top, top, bottom])

    def _update_reachability_plot(self) -> None:
        """绘制可达区域边界。"""
        l1, l2, l3 = self.kin.cfg.l1, self.kin.cfg.l2, self.kin.cfg.l3
        r_max = l1 + l2 + l3
        r_min = max(0.0, l1 - (l2 + l3), l2 - (l1 + l3), l3 - (l1 + l2))

        angles = [i * 2.0 * math.pi / 360.0 for i in range(361)]
        outer_x = [r_max * math.cos(a) for a in angles]
        outer_y = [r_max * math.sin(a) for a in angles]
        self.reach_outer_plot.setData(outer_x, outer_y)

        if r_min > 1e-6:
            inner_x = [r_min * math.cos(a) for a in angles]
            inner_y = [r_min * math.sin(a) for a in angles]
            self.reach_inner_plot.setData(inner_x, inner_y)
        else:
            self.reach_inner_plot.setData([], [])

    def _is_reachable_xy(self, x: float, y: float) -> bool:
        """判断目标点是否在可达环形区域内。"""
        l1, l2, l3 = self.kin.cfg.l1, self.kin.cfg.l2, self.kin.cfg.l3
        r_max = l1 + l2 + l3
        r_min = max(0.0, l1 - (l2 + l3), l2 - (l1 + l3), l3 - (l1 + l2))
        r = math.hypot(x, y)
        return r_min <= r <= r_max


def main() -> None:
    """GUI 入口。"""
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.resize(900, 700)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
