import cv2
from http.server import BaseHTTPRequestHandler, HTTPServer
import threading
import time

# Shared latest frame (JPEG bytes)
_latest_jpeg = None
_lock = threading.Lock()
camera_index = 1


def camera_loop(device_index=0, width=None, height=None, jpeg_quality=80):
    global _latest_jpeg
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
            time.sleep(0.01)
            continue

        ok, buf = cv2.imencode(".jpg", frame, encode_params)
        if not ok:
            continue

        jpg = buf.tobytes()
        with _lock:
            _latest_jpeg = jpg


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path not in ("/", "/mjpeg"):
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        try:
            while True:
                with _lock:
                    jpg = _latest_jpeg

                if jpg is None:
                    time.sleep(0.01)
                    continue

                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode("ascii"))
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")

                # Throttle a bit to reduce CPU/bandwidth (adjust as needed)
                time.sleep(0.03)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, format, *args):
        # Silence default logging
        return


def main(host="0.0.0.0", port=8080, device=0):
    t = threading.Thread(
        target=camera_loop, kwargs={"device_index": device}, daemon=True
    )
    t.start()

    server = HTTPServer((host, port), MJPEGHandler)
    print(f"MJPEG stream: http://{host}:{port}/mjpeg")
    server.serve_forever()


if __name__ == "__main__":
    main(
        device=camera_index,
    )
