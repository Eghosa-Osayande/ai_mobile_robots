import json
import socket
from rplidar import RPLidar


HOST = "127.0.0.1"
PORT = 6000
LIDAR_PORT = "/dev/ttyUSB0"


def send_msg(f, msg_type, data):
    f.write(json.dumps({"type": msg_type, "data": data}) + "\n")
    f.flush()


def main():
    print("opening lidar...", flush=True)
    lidar = RPLidar(LIDAR_PORT)
    print("lidar opened", flush=True)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)

    print(f"waiting for client on {HOST}:{PORT} ...", flush=True)
    conn, addr = srv.accept()
    print(f"client connected: {addr}", flush=True)

    f = conn.makefile("w")

    try:
        print("sending info...", flush=True)
        send_msg(f, "info", lidar.get_info())

        print("sending health...", flush=True)
        send_msg(f, "health", lidar.get_health())

        print("starting scan loop...", flush=True)
        for i, scan in enumerate(lidar.iter_scans()):
            send_msg(f, "scan", scan)
            print(f"sent scan {i}, len={len(scan)}", flush=True)

    except KeyboardInterrupt:
        print("stopping", flush=True)
    except Exception as e:
        print(f"server error: {e!r}", flush=True)
        raise
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
        try:
            f.close()
        except Exception:
            pass
        try:
            conn.close()
        except Exception:
            pass
        try:
            srv.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()