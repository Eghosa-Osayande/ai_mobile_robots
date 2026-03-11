from __future__ import annotations

import queue
import threading
from typing import Optional

import numpy as np
from rplidar import RPLidar


def stream_lidar_to_queue(
    out_q: queue.Queue,
    port,
    resolution,
    fov_range,  # (start_deg, end_deg)
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

    def angle_in_fov(a: float) -> bool:
        return ((a - start) % 360.0) <= fov

    lidar = None

    try:

        lidar = RPLidar(port)
        lidar.start_motor()

        for scan in lidar.iter_scans():
            if stop_event is not None and stop_event.is_set():
                break

            readings = []

            for point in scan:
                if len(point) < 3:
                    continue

                _, a_deg, d = point
                a = norm_deg(float(a_deg))
                if not angle_in_fov(a):
                    continue

                dist = float(d)

                readings.append((a, dist))

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
            raise RuntimeError("Lidar reader thread failed") from err
        except queue.Empty:
            pass
        return self.q.get()
