import socket
import struct
import threading
import queue

import cv2
import numpy as np


def recv_exact(sock, n):
    data = b""
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise ConnectionError("socket closed")
        data += chunk
    return data


def q_put_latest(q, item):
    try:
        q.put_nowait(item)
        return
    except queue.Full:
        pass

    try:
        q.get_nowait()
    except queue.Empty:
        pass

    try:
        q.put_nowait(item)
    except queue.Full:
        pass


class CameraLink:
    def __init__(
        self,
        src=0,
        tcp=None,
    ):
        self.cap = None
        self.sock = None
        self.q = None
        self._tcp = tcp
        self._src = src
        self._is_connected = False

    def start(self):
        if self._is_connected:
            return

        tcp = self._tcp
        src = self._src
        if tcp is None:
            self.cap = cv2.VideoCapture(src)
            if not self.cap.isOpened():
                raise RuntimeError("could not open camera")
        else:
            host, port = tcp
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((host, port))
            self.q = queue.Queue(maxsize=1)
            threading.Thread(target=self._tcp_reader, daemon=True).start()

        self._is_connected = True

    def _tcp_reader(self):
        while True:
            hdr = recv_exact(self.sock, 4)
            size = struct.unpack(">I", hdr)[0]
            jpg = recv_exact(self.sock, size)

            frame = cv2.imdecode(
                np.frombuffer(jpg, dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )
            if frame is not None:
                q_put_latest(self.q, frame)

    def read(self):
        if self.cap is not None:
            return self.cap.read()

        try:
            frame = self.q.get(timeout=1.0)
            return True, frame
        except queue.Empty:
            return False, None

    def release(self):
        if self.cap is not None:
            self.cap.release()
        if self.sock is not None:
            self.sock.close()
        self._is_connected = False
