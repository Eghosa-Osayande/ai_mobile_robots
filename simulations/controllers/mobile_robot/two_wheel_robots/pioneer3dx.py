import time
import time

from .two_wheel_robot_base import TwoWheelRobotBase
from .integrations.pioneer_link import PioneerLink
from .integrations.lidar_link import LidarLink


class Pioneer3dx(TwoWheelRobotBase):

    def __init__(
        self,
        serial_port_baud=None,
        tcp_host_port=None,
        lidar_port=None,
    ):

        self.link = PioneerLink()
        self.lidar_link = None
        self.pos_offset = (0, 0, 0)

        def _connect():
            try:
                if serial_port_baud:
                    port, baudRate = serial_port_baud
                    serial_ok = self.link.connect_serial(
                        port,
                        baudRate,
                    )

                    if not serial_ok:
                        raise Exception("Pioneer serial connection failed")
                elif tcp_host_port:
                    tcp_host, tcp_port = tcp_host_port

                    tcp_ok = self.link.connect_tcp(
                        tcp_host,
                        tcp_port,
                    )

                    if not tcp_ok:
                        raise Exception("Pioneer tcp connection failed")
                else:
                    raise Exception(
                        "serial_port_baud or tcp_host_port must be provided"
                    )

            except Exception as e:
                print("Pioneer Connection Failed", e)
                raise e

            self.lidar_link = None
            if lidar_port is not None:
                self.lidar_link = LidarLink(
                    port=lidar_port,
                    resolution=8,
                    fov_range=(-90, 90),
                )

                self.lidar_link.start()

            self.enableMotors(True)

        self._reconnect = _connect
        _connect()

    def enableMotors(self, enable):
        self.send_command(4, argument_data=1 if enable else 0)

    def send_command(
        self,
        command_number,
        argument_data=None,
        argument_type="int",
    ):
        self.link.send_cmd(
            command_number, argument_data if argument_data is not None else 0
        )

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

    def move_wheels(self, v_right, v_left):
        v_right = self._transformVelocity(v_right)
        v_left = self._transformVelocity(v_left)
        self.enableMotors(True)

        arg = ((v_right & 0xFF) << 8) | (v_left & 0xFF)
        self.send_command(32, argument_data=arg)

    def step(self, timeStep):
        time.sleep(timeStep)

    def state(self) -> tuple[float, float, float]:
        data = self.link.get_state()
        if data is None:
            return (
                0 + self.pos_offset[0],
                0 + self.pos_offset[1],
                0 + self.pos_offset[2],
                [5 for _ in range(8)],
            )

        lidars = None
        scan = []
        if self.lidar_link:
            lidars, scan = self.lidar_link.get_state()

        proximity_data = lidars if lidars is not None else data.sonars_mm
        proximity_data = [l / 1000 for l in proximity_data]

        return (
            (data.x_mm / 1000) + self.pos_offset[0],
            (data.y_mm / 1000) + self.pos_offset[1],
            (data.th_deg_360) + self.pos_offset[2],
            proximity_data,
            scan,
        )

    def reset(
        self,
        pos: tuple[float, float] = None,
        **kw,
    ):
        if pos is not None:
            self.pos_offset = pos

    def shutdown(self):
        try:
            if self.lidar_link:
                self.lidar_link.close()
        except Exception:
            pass

        try:
            self.link.close()
        except Exception:
            pass
