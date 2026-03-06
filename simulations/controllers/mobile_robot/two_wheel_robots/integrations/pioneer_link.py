# Pioneer P3-DX Serial / MobileSim

from __future__ import annotations
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple
import socket
import threading
import time
from typing import Optional
import serial
import math
from queue import Queue, Empty


HEADER = bytes([0xFA, 0xFB])

SYNC0 = bytes([0xFA, 0xFB, 0x03, 0x00, 0x00, 0x00])
SYNC1 = bytes([0xFA, 0xFB, 0x03, 0x01, 0x00, 0x01])
SYNC2 = bytes([0xFA, 0xFB, 0x03, 0x02, 0x00, 0x02])

CMD_PULSE = 0
CMD_OPEN = 1
CMD_ENABLE = 4
CMD_VEL = 11
CMD_RVEL = 21
CMD_SONAR = 28
CMD_STOP = 29

ARG_POS_INT = 0x3B
ARG_NEG_INT = 0x1B


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def wrap_deg_180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def deg360(th: float) -> float:
    return th - 360.0 * math.floor(th / 360.0)


def calc_checksum(packet: bytes) -> int:
    c = 0
    i = 3
    n = packet[2] - 2
    while n > 1:
        c += (packet[i] << 8) | packet[i + 1]
        c &= 0xFFFF
        n -= 2
        i += 2
    if n > 0:
        c ^= packet[i]
    return c & 0xFFFF


def _i16_le(buf: bytes, off: int) -> int:
    return struct.unpack_from("<h", buf, off)[0]


def parse_sip(
    packet: bytes, prev_sonars: List[float]
) -> Optional[Tuple[float, float, float, float, float, List[float]]]:
    if len(packet) < 6 or packet[:2] != HEADER:
        return None
    bytecount = packet[2]
    if len(packet) < 3 + bytecount:
        return None

    payload = packet[3 : 3 + bytecount]
    if len(payload) < 5:
        return None

    data = payload[1:-2]
    if len(data) < 2 * 5 + 1 + 2 * 3 + 1 + 1:
        return None

    off = 0
    x_raw = float(_i16_le(data, off))
    off += 2
    y_raw = float(_i16_le(data, off))
    off += 2
    th_raw = float(_i16_le(data, off))
    off += 2
    lvel = float(_i16_le(data, off))
    off += 2
    rvel = float(_i16_le(data, off))
    off += 2

    _battery = data[off]
    off += 1
    _stall = _i16_le(data, off)
    off += 2
    _control = _i16_le(data, off)
    off += 2
    _flags = _i16_le(data, off)
    off += 2
    _compass = data[off]
    off += 1
    sonar_count = int(data[off])
    off += 1

    sonars = prev_sonars[:] if prev_sonars else [0.0] * 16
    for _ in range(sonar_count):
        if off + 3 > len(data):
            break
        sn = int(data[off])
        off += 1
        sr = float(_i16_le(data, off))
        off += 2
        if 0 <= sn < len(sonars):
            sonars[sn] = sr

    return x_raw, y_raw, th_raw, lvel, rvel, sonars


@dataclass
class RobotState:
    x_mm: float = 0.0
    y_mm: float = 0.0
    th_deg_wrap: float = 0.0
    th_deg_360: float = 0.0

    left_vel_mm_s: float = 0.0
    right_vel_mm_s: float = 0.0

    v_mm_s: float = 0.0
    w_deg_s: float = 0.0

    sonars_mm: List[float] = None

    def __post_init__(self):
        if self.sonars_mm is None:
            self.sonars_mm = [0.0] * 16


class PioneerLink:
    def __init__(
        self,
    ):
        self.use_serial = False
        self.sock: Optional[socket.socket] = None
        self.ser = None

        self._rx_thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        self._buf = bytearray()

        self._latest: Queue[RobotState] = Queue(maxsize=1)

        self._origin_set = False
        self._x0 = 0.0
        self._y0 = 0.0
        self._th0 = 0.0
        self._sonars = [0.0] * 16

        self._last_pulse = 0.0

        self.endpoint = ""
        self.transport = ""

    def _theta_raw_to_deg(self, th_raw: float) -> float:
        # Both the simulator and the physical hardware
        # send heading data as an integer between 0 and 4095 ticks per revolution.
        # if self.use_serial:
        #     return float(th_raw) / 10.0
        return float(th_raw) * 360.0 / 4095.0

    def _write(self, b: bytes) -> None:
        try:
            if self.use_serial and self.ser is not None:
                self.ser.write(b)
            elif self.sock is not None:
                self.sock.sendall(b)
        except Exception as e:
            print(f"[WARN] write failed ({e}); disconnecting")
            self.close()

    def _read_some(self) -> bytes:
        if self.use_serial and self.ser is not None:
            return self.ser.read(4096)
        if self.sock is not None:
            try:
                data = self.sock.recv(4096)
                if data == b"":
                    raise ConnectionError("TCP closed")
                return data
            except socket.timeout:
                return b""
        return b""

    def _start_reader_and_wait_sip(self, timeout_s: float = 0.9) -> bool:
        try:
            while True:
                self._latest.get_nowait()
        except Empty:
            pass

        self._origin_set = False
        self._stop_evt.clear()
        self._rx_thread = threading.Thread(
            target=self._reader_loop,
            daemon=True,
        )
        self._rx_thread.start()

        self._last_pulse = time.time()
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self.get_state() is not None:
                return True
                
        return False

    def connect_tcp(
        self,
        host: str,
        port: int,
    ) -> bool:

        self.close()
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((host, port))
            self.sock.settimeout(0.05)
            self.use_serial = False
            self.transport = "tcp"
            self.endpoint = f"{host}:{port}"
            print(f"[OK] TCP connected: {self.endpoint}")
        except Exception as e:
            print(f"[ERR] TCP connect failed: {e}")
            self.sock = None
            return False

        if not self._handshake():
            self.close()
            return False

        if not self._start_reader_and_wait_sip():
            print("[ERROR] TCP connected but no SIP received.")
            self.close()
            return False

        return True

    def connect_serial(
        self,
        port: str,
        baud: int,
    ) -> bool:

        # self.close()
        if serial is None:
            print("[ERR] pyserial not installed; cannot use Serial")
            return False
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            self.use_serial = True
            self.transport = "serial"
            self.endpoint = f"{port} @ {baud}"
            print(f"[OK] Serial connected: {self.endpoint}")
        except Exception as e:
            print(f"[ERR] Serial connect failed: {e}")
            self.ser = None
            return False

        if not self._handshake():
            self.close()
            return False

        if not self._start_reader_and_wait_sip():
            print("[ERR] Serial connected but no SIP received (wrong port/baud?)")
            self.close()
            return False

        return True

    def close(self) -> None:
        print("CLOSEEE")
        self._stop_evt.set()
        try:
            self.stop_motion()
        except Exception:
            pass

        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass

        try:
            if self.sock is not None:
                self.sock.close()
        except Exception:
            pass

        self.ser = None
        self.sock = None
        self.use_serial = False

        try:
            while True:
                self._latest.get_nowait()
        except Empty:
            pass

    def is_connected(self) -> bool:
        return self.ser is not None or self.sock is not None

    def send_cmd(self, cmd: int, arg: Optional[int] = None) -> None:
        if arg is None:
            bytecount = 1 + 2
            pkt_wo = HEADER + bytes([bytecount]) + bytes([cmd])
            chk = calc_checksum(pkt_wo + b"\x00\x00").to_bytes(2, "big")
            self._write(pkt_wo + chk)
            return

        if isinstance(arg, str):
            argtype = bytes([0x2B])
            aval = arg
        elif arg < 0:
            argtype = ARG_NEG_INT
            aval = abs(int(arg))
        else:
            argtype = ARG_POS_INT
            aval = int(arg)

        arg_bytes = struct.pack("<H", aval & 0xFFFF)
        bytecount = 1 + 1 + 2 + 2
        pkt_wo = HEADER + bytes([bytecount]) + bytes([cmd, argtype]) + arg_bytes
        chk = calc_checksum(pkt_wo + b"\x00\x00").to_bytes(2, "big")
        self._write(pkt_wo + chk)

    def _handshake(self) -> bool:
        try:
            for p in (SYNC0, SYNC1, SYNC2):
                self._write(p)
                time.sleep(0.05)
                _ = self._read_some()

            self.send_cmd(CMD_OPEN)
            self.send_cmd(CMD_ENABLE, 1)
            self.send_cmd(CMD_SONAR, 1)
            self.send_cmd(CMD_PULSE)
            print("[OK] Handshake done (SYNC/OPEN/ENABLE/SONAR)")
            return True
        except Exception as e:
            print(f"[ERR] Handshake failed: {e}")
            return False

    def _reader_loop(self) -> None:
        while not self._stop_evt.is_set():
            self.pulse_if_needed()
            try:
                data = self._read_some()
                if data:
                    self._buf.extend(data)
                    self._consume_packets()
                else:
                    # time.sleep(0.01)
                    ...
            except Exception as e:
                print(f"[WARN] reader stopped ({e}); disconnecting")
                self.close()
                break

    def _consume_packets(self) -> None:
        while True:
            if len(self._buf) < 4:
                return

            h = self._buf.find(HEADER)
            if h < 0:
                self._buf.clear()
                return
            if h > 0:
                del self._buf[:h]
                if len(self._buf) < 4:
                    return

            bytecount = self._buf[2]
            total_len = 3 + bytecount
            if len(self._buf) < total_len:
                return

            pkt = bytes(self._buf[:total_len])
            del self._buf[:total_len]

            ptype = pkt[3] if len(pkt) > 3 else 0
            if not (0x30 <= ptype <= 0x33):
                continue

            parsed = parse_sip(pkt, self._sonars)
            if parsed is None:
                print("parse is None")
                continue

            x_raw, y_raw, th_raw, lvel, rvel, sonars = parsed
            th_deg_abs = self._theta_raw_to_deg(th_raw)
            self._sonars = sonars[:]

            if not self._origin_set:
                self._origin_set = True
                self._x0, self._y0, self._th0 = x_raw, y_raw, th_deg_abs

            x_rel = x_raw - self._x0
            y_rel = y_raw - self._y0
            th_rel = th_deg_abs - self._th0

            st = RobotState(
                x_mm=x_rel,
                y_mm=y_rel,
                th_deg_wrap=wrap_deg_180(th_rel),
                th_deg_360=deg360(th_rel),
                left_vel_mm_s=lvel,
                right_vel_mm_s=rvel,
                v_mm_s=(lvel + rvel) / 2.0,
                w_deg_s=(rvel - lvel) * 0.1,
                sonars_mm=sonars[:],
            )

            self._latest.put(st)

    def get_state(self) -> Optional[RobotState]:
        s = self._latest.get()
        if s is None:
            return None

        return RobotState(
            x_mm=s.x_mm,
            y_mm=s.y_mm,
            th_deg_wrap=s.th_deg_wrap,
            th_deg_360=s.th_deg_360,
            left_vel_mm_s=s.left_vel_mm_s,
            right_vel_mm_s=s.right_vel_mm_s,
            v_mm_s=s.v_mm_s,
            w_deg_s=s.w_deg_s,
            sonars_mm=s.sonars_mm[:],
        )

    def pulse_if_needed(self) -> None:
        WATCHDOG_SEC = 1.0
        now = time.time()

        if now - self._last_pulse >= WATCHDOG_SEC:
            try:
                self.send_cmd(CMD_PULSE)
            except Exception:
                pass
            self._last_pulse = now
