import threading
import time
from dataclasses import dataclass
from typing import Callable, Iterable, Optional

import serial

from .protocol import Feedback, TrajPoint, pack_traj_points, try_parse_feedback


@dataclass
class SerialConfig:
    """串口配置。"""
    port: str
    baud: int = 115200
    timeout: float = 0.05


class ArmClient:
    """串口通信客户端，负责发送轨迹与接收反馈。"""
    def __init__(self, cfg: SerialConfig) -> None:
        self.cfg = cfg
        self._ser: Optional[serial.Serial] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._rx_running = False
        self._on_feedback: Optional[Callable[[Feedback], None]] = None

    def connect(self) -> None:
        """打开串口并启动接收线程。"""
        self._ser = serial.Serial(self.cfg.port, self.cfg.baud, timeout=self.cfg.timeout)
        self._rx_running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def close(self) -> None:
        """关闭串口并停止接收线程。"""
        self._rx_running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=0.5)
        if self._ser:
            self._ser.close()
            self._ser = None

    def set_feedback_callback(self, cb: Callable[[Feedback], None]) -> None:
        """注册反馈回调。"""
        self._on_feedback = cb

    def send_points(self, points: Iterable[TrajPoint]) -> None:
        """发送轨迹点序列。"""
        if not self._ser:
            raise RuntimeError("serial not connected")
        frames = pack_traj_points(points)
        for f in frames:
            self._ser.write(f)

    def _rx_loop(self) -> None:
        """后台接收并解析反馈。"""
        buf = bytearray()
        while self._rx_running:
            if not self._ser:
                time.sleep(0.01)
                continue
            data = self._ser.read(128)
            if not data:
                continue
            buf.extend(data)

            if len(buf) > 512:
                buf = buf[-512:]

            fb = try_parse_feedback(buf)
            if fb and self._on_feedback:
                self._on_feedback(fb)
                buf.clear()
