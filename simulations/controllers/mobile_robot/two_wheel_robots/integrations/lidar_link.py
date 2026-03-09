from __future__ import annotations

import json
import queue
import socket
import threading
from typing import Optional

import numpy as np
from rplidar import RPLidar

import json
import socket


class RemoteRPLidar:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = int(port)
        self.sock = None
        self.file = None
        self._info = None
        self._health = None

    def connect(self) -> None:
        self.sock = socket.create_connection((self.host, self.port))
        self.file = self.sock.makefile("r")

    def _read_msg(self) -> dict:
        if self.file is None:
            raise RuntimeError("Remote lidar is not connected")

        line = self.file.readline()
        if not line:
            raise ConnectionError("Remote lidar connection closed")

        msg = json.loads(line)

        msg_type = msg.get("type")
        if msg_type == "info":
            self._info = msg.get("data")
        elif msg_type == "health":
            self._health = msg.get("data")

        return msg

    def get_info(self):
        if self._info is not None:
            return self._info

        while True:
            msg = self._read_msg()
            if msg.get("type") == "info":
                return msg.get("data")

    def get_health(self):
        if self._health is not None:
            return self._health

        while True:
            msg = self._read_msg()
            if msg.get("type") == "health":
                return msg.get("data")

    def iter_scans(self):
        while True:
            msg = self._read_msg()
            if msg.get("type") == "scan":
                yield msg.get("data")

    def start_motor(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def stop_motor(self) -> None:
        pass

    def disconnect(self) -> None:
        try:
            if self.file is not None:
                self.file.close()
        finally:
            self.file = None
            if self.sock is not None:
                self.sock.close()
                self.sock = None

def stream_lidar_to_queue(
    out_q: queue.Queue,
    port,
    resolution,
    fov_range,  # (start_deg, end_deg)
    default_m=5.0,
    *,
    stop_event: threading.Event | None = None,
    err_q: queue.Queue | None = None,
):
    def norm_deg(x: float) -> float:
        x = x % 360.0
        return x if x >= 0 else x + 360.0

    start = norm_deg(float(fov_range[0]))
    end = norm_deg(float(fov_range[1]))
    fov = (end - start) % 360.0
    if fov == 0.0:
        fov = 360.0

    if resolution <= 0:
        raise ValueError("resolution must be > 0")

    unit_angle = fov / float(resolution)

    def angle_in_fov(a: float) -> bool:
        return ((a - start) % 360.0) <= fov

    lidar = None

    try:
        if isinstance(port, tuple) and len(port) == 2:
            host, tcp_port = port
            lidar = RemoteRPLidar(str(host), int(tcp_port))
            lidar.connect()
        else:
            lidar = RPLidar(port)
            lidar.start_motor()

        for scan in lidar.iter_scans():
            if stop_event is not None and stop_event.is_set():
                break

            readings = np.full(int(resolution), float(default_m), dtype=np.float32)
            written_mask = np.zeros(resolution, dtype=bool)

            for point in scan:
                if len(point) < 3:
                    continue

                _, a_deg, d = point
                a = norm_deg(float(a_deg))
                if not angle_in_fov(a):
                    continue

                offset = (a - start) % 360.0
                idx = int(offset / unit_angle)
                if idx < 0:
                    continue
                if idx >= resolution:
                    idx = resolution - 1

                dist = float(d)

                if dist < readings[idx]:
                    readings[idx] = dist
                    written_mask[idx] = True

            readings[~written_mask] = 0

            try:
                while True:
                    out_q.get_nowait()
            except queue.Empty:
                pass

            out_q.put((readings, scan))

    except Exception as e:
        if err_q is not None:
            err_q.put(e)
    finally:
        if lidar is not None:
            try:
                lidar.stop()
            except Exception:
                pass
            try:
                lidar.stop_motor()
            except Exception:
                pass
            try:
                lidar.disconnect()
            except Exception:
                pass


class LidarLink:
    def __init__(
        self,
        port,
        resolution,
        fov_range,
        
    ):
        if resolution <= 0:
            raise ValueError("resolution must be > 0")

        self.resolution = resolution
        self.fov_range = fov_range
        self.serial_port = port

        self.th: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.err_q = queue.Queue()
        self.q = queue.Queue(maxsize=1)

    def start(self) -> bool:
        self.th = threading.Thread(
            target=stream_lidar_to_queue,
            args=(
                self.q,
                self.serial_port,
                self.resolution,
                self.fov_range,
                float("inf"),
            ),
            kwargs={
                "stop_event": self.stop_event,
                "err_q": self.err_q,
            },
            daemon=True,
        )
        self.th.start()
        return True

    def close(self) -> None:
        self.stop_event.set()
        if self.th is not None:
            self.th.join(timeout=2.0)

    def get_state(self):
        try:
            err = self.err_q.get_nowait()
            raise RuntimeError("Reader thread failed") from err
        except queue.Empty:
            pass
        return self.q.get()