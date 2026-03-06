from __future__ import annotations

import socket
import threading
import queue
from typing import Optional


def put_latest(q: queue.Queue, value):
    try:
        while True:
            q.get_nowait()
    except queue.Empty:
        pass

    try:
        q.put_nowait(value)
    except queue.Full:
        pass


def tcp_cmd_server(host: str, port: int, cmd_q: queue.Queue, stop_event: threading.Event):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(5)
    srv.settimeout(0.5)

    try:
        while not stop_event.is_set():
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue

            with conn:
                conn.settimeout(0.5)
                buf = b""

                while not stop_event.is_set():
                    try:
                        chunk = conn.recv(1024)
                    except socket.timeout:
                        continue

                    if not chunk:
                        break

                    buf += chunk

                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        cmd = line.decode("utf-8", errors="ignore").strip()
                        if cmd:
                            put_latest(cmd_q, cmd)
    finally:
        srv.close()


def stream_lidar_to_queue(
    out_q: queue.Queue,
    resolution,
    *,
    stop_event: threading.Event | None = None,
    err_q: queue.Queue | None = None,
    cmd_q: queue.Queue | None = None,
):
    try:
        res = int(resolution)
        default_readings = [5 for _ in range(res)]
        readings = list(default_readings)
        res_mid = res // 2

        put_latest(out_q, readings)

        while not (stop_event and stop_event.is_set()):
            cmd = None
            if cmd_q is not None:
                try:
                    while True:
                        cmd = cmd_q.get_nowait()
                except queue.Empty:
                    pass

            if cmd == "l":
                readings = [5 if i >= res_mid else 0 for i in range(res)]
            elif cmd == "r":
                readings = [5 if i < res_mid else 0 for i in range(res)]
            elif cmd == "c":
                readings = [5 for i in range(res)]

            put_latest(out_q, readings)

            if stop_event is not None:
                stop_event.wait(0.05)

    except Exception as e:
        if err_q is not None:
            err_q.put(e)


class LidarLink:
    def __init__(self, port, resolution, fov_range, tcp_host="0.0.0.0", tcp_port=9999):
        self.serial_port = port
        self.resolution = resolution
        self.fov_range = fov_range

        self.tcp_host = tcp_host
        self.tcp_port = tcp_port

        self.stop_event = threading.Event()
        self.err_q = queue.Queue()
        self.q = queue.Queue(maxsize=1)
        self.cmd_q = queue.Queue(maxsize=1)

        self._last = [5 for _ in range(8)]
        self.reader_th: Optional[threading.Thread] = None
        self.tcp_th: Optional[threading.Thread] = None

    def start(self):
        self.reader_th = threading.Thread(
            target=stream_lidar_to_queue,
            args=(self.q, self.resolution),
            kwargs={
                "stop_event": self.stop_event,
                "err_q": self.err_q,
                "cmd_q": self.cmd_q,
            },
            daemon=True,
        )
        self.reader_th.start()

        self.tcp_th = threading.Thread(
            target=tcp_cmd_server,
            args=(self.tcp_host, self.tcp_port, self.cmd_q, self.stop_event),
            daemon=True,
        )
        self.tcp_th.start()

    def close(self):
        self.stop_event.set()

    def get_state(self) -> list[float]:
        try:
            err = self.err_q.get_nowait()
            raise RuntimeError("Reader thread failed") from err
        except queue.Empty:
            pass

        values = self._last
        try:
            values = self.q.get_nowait()
            if values is not None:
                self._last = values
        except queue.Empty:
            pass

        return self._last