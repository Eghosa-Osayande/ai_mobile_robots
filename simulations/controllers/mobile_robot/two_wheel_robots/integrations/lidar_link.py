from __future__ import annotations

import threading
import time
from typing import Optional
from rplidar import RPLidar, RPLidarException
import threading
from typing import Optional
import numpy as np
import threading
import queue


def stream_lidar_to_queue(
    out_q: queue.Queue,
    port,
    resolution,
    fov_range,  # (start_deg, end_deg)
    default_m=5.0,
    *,
    stop_event: threading.Event | None = None,
    err_q: queue.Queue = None,
):
    from rplidar import RPLidar
    import numpy as np

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

    lidar = RPLidar(port)
    try:
        for scan in lidar.iter_scans():
            if stop_event is not None and stop_event.is_set():
                break

            readings = np.full(int(resolution), float(default_m), dtype=np.float32)
            written_mask = np.zeros(resolution, dtype=bool)

            for _, a_deg, d in scan:
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

            # keep only the most recent value
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

    def close(self) -> None:
        self.stop_event.set()

    def get_state(self) -> list[float]:
        # if reader crashed, exit
        try:
            err = self.err_q.get_nowait()
            raise RuntimeError("Reader thread failed") from err
        except queue.Empty:
            pass
        return self.q.get()
