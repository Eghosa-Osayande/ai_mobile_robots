import cv2
import threading
import time
import queue
import socket
import struct
import sys

camera_index = int(sys.argv[1])
PORT = int(sys.argv[2])

ROTATE_CW_90 = 0

_latest_q: "queue.Queue[bytes]" = queue.Queue(maxsize=1)


def _q_put_latest(q: queue.Queue, item: bytes) -> None:
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


def _rotate_frame(frame, cw_90: int):
    cw_90 %= 4
    if cw_90 == 0:
        return frame
    if cw_90 == 1:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if cw_90 == 2:
        return cv2.rotate(frame, cv2.ROTATE_180)
    return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)


def camera_loop(device_index=0, width=None, height=None, jpeg_quality=80):
    cap = cv2.VideoCapture(device_index)

    if width is not None:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
    if height is not None:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))

    if not cap.isOpened():
        raise RuntimeError("Could not open camera")

    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)]

    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.001)
            continue

        frame = _rotate_frame(frame[:, : ], ROTATE_CW_90)

        ok, buf = cv2.imencode(".jpg", frame, encode_params)
        if not ok:
            continue

        _q_put_latest(_latest_q, buf.tobytes())


def client_handler(conn):
    last_jpg = None

    try:
        while True:
            try:
                last_jpg = _latest_q.get_nowait()
            except queue.Empty:
                pass

            if last_jpg is None:
                time.sleep(0.01)
                continue

            header = struct.pack(">I", len(last_jpg))
            conn.sendall(header + last_jpg)

            time.sleep(0.03)

    except (BrokenPipeError, ConnectionResetError):
        pass
    finally:
        conn.close()


def tcp_server(host="0.0.0.0", port=PORT):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((host, port))
    s.listen()

    print(f"TCP image stream on {host}:{port}")

    while True:
        conn, addr = s.accept()
        print("client connected:", addr)

        t = threading.Thread(target=client_handler, args=(conn,), daemon=True)
        t.start()


def main(device=0):
    t = threading.Thread(target=camera_loop, kwargs={"device_index": device}, daemon=True)
    t.start()

    tcp_server()


if __name__ == "__main__":
    main(device=camera_index)