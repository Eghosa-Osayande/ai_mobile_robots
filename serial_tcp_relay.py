import argparse
import socket
import threading
import time
from typing import List, Tuple

import serial

TCP_HOST = "0.0.0.0"
BAUD = 9600
SERIAL_TIMEOUT_S = 0.1

SERIAL_READ_CHUNK = 1024
TCP_RECV_CHUNK = 4096


def relay_serial_to_tcp(ser: serial.Serial, conn: socket.socket, stop: threading.Event) -> None:
    try:
        while not stop.is_set():
            data = ser.read(SERIAL_READ_CHUNK)
            if data:
                conn.sendall(data)
            else:
                time.sleep(0.005)
    except Exception:
        stop.set()


def relay_tcp_to_serial(ser: serial.Serial, conn: socket.socket, stop: threading.Event) -> None:
    try:
        conn.settimeout(0.5)
        while not stop.is_set():
            try:
                chunk = conn.recv(TCP_RECV_CHUNK)
            except socket.timeout:
                continue
            if not chunk:
                stop.set()
                break
            ser.write(chunk)
    except Exception:
        stop.set()


def serve(serial_port: str, tcp_port: int, shutdown: threading.Event) -> None:
    ser = None
    server = None

    try:
        ser = serial.Serial(
            port=serial_port,
            baudrate=BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=SERIAL_TIMEOUT_S,
        )

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((TCP_HOST, tcp_port))
        server.listen(1)
        server.settimeout(0.5)  # so we can periodically check shutdown

        print(f"[{serial_port}:{tcp_port}] Serial: {serial_port} @ {BAUD}")
        print(f"[{serial_port}:{tcp_port}] TCP:    {TCP_HOST}:{tcp_port}")

        while not shutdown.is_set():
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            print(f"[{serial_port}:{tcp_port}] Client connected: {addr}")

            stop = threading.Event()
            t1 = threading.Thread(target=relay_serial_to_tcp, args=(ser, conn, stop), daemon=True)
            t2 = threading.Thread(target=relay_tcp_to_serial, args=(ser, conn, stop), daemon=True)
            t1.start()
            t2.start()

            # Wait until either per-connection stop triggers, or global shutdown triggers
            while not stop.is_set() and not shutdown.is_set():
                time.sleep(0.05)

            stop.set()
            try:
                conn.close()
            except Exception:
                pass

            print(f"[{serial_port}:{tcp_port}] Client disconnected")

    finally:
        try:
            if server is not None:
                server.close()
        except Exception:
            pass
        try:
            if ser is not None:
                ser.close()
        except Exception:
            pass
        print(f"[{serial_port}:{tcp_port}] Relay stopped")


def parse_mapping(s: str) -> Tuple[str, int]:
    # Accept "/dev/ttyUSB0:7000" or "COM3:7000"
    if ":" not in s:
        raise argparse.ArgumentTypeError("Mapping must look like SERIAL_PORT:TCP_PORT")

    serial_port, tcp_port_str = s.rsplit(":", 1)
    serial_port = serial_port.strip()
    tcp_port_str = tcp_port_str.strip()

    if not serial_port:
        raise argparse.ArgumentTypeError("SERIAL_PORT is empty")

    try:
        tcp_port = int(tcp_port_str)
    except ValueError:
        raise argparse.ArgumentTypeError("TCP_PORT must be an integer")

    if not (1 <= tcp_port <= 65535):
        raise argparse.ArgumentTypeError("TCP_PORT must be 1..65535")

    return serial_port, tcp_port


def main() -> None:
    parser = argparse.ArgumentParser(description="Run one or more Serial<->TCP relays concurrently.")
    parser.add_argument(
        "mappings",
        nargs="+",
        type=parse_mapping,
        help='One or more mappings like "/dev/ttyUSB0:7000" (or "COM3:7000" on Windows)',
    )
    args = parser.parse_args()

    shutdown = threading.Event()
    threads: List[threading.Thread] = []

    for serial_port, tcp_port in args.mappings:
        t = threading.Thread(target=serve, args=(serial_port, tcp_port, shutdown), daemon=True)
        t.start()
        threads.append(t)

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        shutdown.set()
        for t in threads:
            t.join(timeout=2.0)


if __name__ == "__main__":
    main()