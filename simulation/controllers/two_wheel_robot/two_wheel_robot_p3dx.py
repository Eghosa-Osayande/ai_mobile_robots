import time
import threading
import struct
import time
import serial

from dataclasses import dataclass

from two_wheel_robot_base import TwoWheelRobot


# Synchronization packets
HEADER = bytes([0xFA, 0xFB])
SYNC0 = bytes([250, 251, 3, 0, 0, 0])
SYNC1 = bytes([250, 251, 3, 1, 0, 1])
SYNC2 = bytes([250, 251, 3, 2, 0, 2])


@dataclass
class TwoWheelRobot_P3DX(TwoWheelRobot):

    connection: serial.Serial
    pulseThread: threading.Thread
    stopEvent: threading.Event

    def __init__(
        self,
        port: str,
        baudRate: int = 9600,
    ):
        try:
            self.connection = serial.Serial(port, baudRate, timeout=1)
        except Exception as e:
            print("Serial connection failed. Attempting TCP connection...")
            raise e

        self.stopEvent = threading.Event()
        self.pulseThread = threading.Thread(
            target=self.watchdog_pulse, args=(), daemon=True
        )

        if not self.send_initial_packets():
            print("initial robot packet not sent")
            raise Exception("initial robot packet not sent")

        self.pulseThread.start()
        self.enableMotors(True)

    def enableMotors(self, enable):
        self.send_command(4, argument_data=1 if enable else 0)

    def send_initial_packets(self) -> bool:
        sync_packets = [SYNC0, SYNC1, SYNC2]
        for packet in sync_packets:
            self.connection.write(packet)
            response = self.connection.read(1024)

            if response != packet and packet != SYNC2:
                print(f"Error: {packet} response mismatch")
                return False
            elif packet == SYNC2:
                header = response[:2]
                if header != bytes([0xFA, 0xFB]):
                    print("Error: Invalid SIP header")
                    return
                print(response[3:29].decode("utf-8", errors="ignore"))

        print("Connection established successfully")
        self.send_command(1)  # open
        self.enableMotors(True)  # enable motors
        return True

    def watchdog_pulse(self):
        while not self.stopEvent.is_set():
            self.send_command(0)  # PULSE
            time.sleep(1)  # or 1.5

    def send_command(
        self,
        command_number,
        argument_data=0,
        argument_type="int",
    ):
        COMMAND_NUMBER = bytes([command_number])

        if argument_type == "int":
            ARGUMENT_TYPE = bytes([0x3B])  # signed 16-bit
            ARGUMENT_DATA = struct.pack(">H", argument_data & 0xFFFF)
        elif argument_type == "string":
            ARGUMENT_TYPE = bytes([0x2B])
            ARGUMENT_DATA = argument_data
        else:
            raise ValueError("Unsupported argument type")

        BYTE_COUNT = len(COMMAND_NUMBER) + len(ARGUMENT_TYPE) + len(ARGUMENT_DATA) + 2

        checksum_data = (
            HEADER
            + bytes([BYTE_COUNT])
            + COMMAND_NUMBER
            + ARGUMENT_TYPE
            + ARGUMENT_DATA
        )

        checksum = self.calc_checksum(checksum_data).to_bytes(2, "big")

        packet = checksum_data + checksum
        self.connection.write(packet)

    def calc_checksum(self, packet: bytes) -> int:
        c: int = 0
        i = 3
        n = packet[2] - 2
        while n > 1:
            c += (packet[i] << 8) | packet[i + 1]
            c &= 0xFFFF
            n -= 2
            i += 2
        if n > 0:
            c ^= packet[i]
        return c

    def _transformVelocity(self, velocity):
        initialVelocity = velocity / 0.02
        velocityStep = int(initialVelocity)
        remainder = initialVelocity - velocityStep
        return velocityStep

    # def move(self, velocity: int):
    #     velocity=self._transformVelocity(velocity)
    #     self.enableMotors(True)

    #     arg = ((velocity & 0xFF) << 8) | (velocity & 0xFF)
    #     self.send_command(32, argument_data=arg)

    # def rotate(self, velocity: int):
    #     velocity=self._transformVelocity(velocity)
    #     self.enableMotors(True)

    #     arg = ((-velocity & 0xFF) << 8) | (velocity & 0xFF)
    #     self.send_command(32, argument_data=arg)
    
    def move_wheels(self, v_right, v_left):
        v_right=self._transformVelocity(v_right)
        v_left=self._transformVelocity(v_left)
        self.enableMotors(True)

        arg = ((v_right & 0xFF) << 8) | (v_left & 0xFF)
        self.send_command(32, argument_data=arg)

    def step(self, timeStep):
        time.sleep(timeStep)

    def receive_sips(self):
        sip_response = self.connection.read(1024)

        if sip_response[:2] != bytes([0xFA, 0xFB]):
            return None

        data = {
            "byte_count": sip_response[2],
            "xpos": sip_response[3],
            "ypos": sip_response[4],
            "theta": sip_response[5],
            "lvel": sip_response[6],
            "rvel": sip_response[7],
            "battery": sip_response[8],
            "stall_bumpers": sip_response[9],
            "control": sip_response[10],
            "flags": sip_response[11],
            "compass": sip_response[12],
            "sonar_count": sip_response[13],
            "sonar_number": sip_response[14],
        }

        sonars = []
        for i in range(16):
            idx = 24 + i * 3
            sonars.append(struct.unpack("<h", sip_response[idx : idx + 2])[0])
        data["sonars"] = sonars

        return data

    def state(self) -> tuple[float, float, float]:
        data = self.receive_sips()
        if data is None:
            return None

        return (data["xpos"], data["ypos"], data["theta"] * 0.001534)
