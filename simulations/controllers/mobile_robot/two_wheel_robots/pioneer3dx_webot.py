from dataclasses import dataclass

from .integrations.lidar_link import LidarLink
from .two_wheel_robot_base import TwoWheelRobotBase
from controller import (
    Robot,
    Motor,
    DistanceSensor,
    PositionSensor,
    GPS,
    InertialUnit,
)
import math


@dataclass
class Pioneer3dxWebot(TwoWheelRobotBase):

    def __init__(
        self,
        robot: Robot,
        leftMotorDeviceName: str,
        rightMotorDeviceName: str,
        gpsOffset: tuple[float, float] = (0, 0),
    ):

        self.robot = robot
        timestep = int(self.robot.getBasicTimeStep())

        self.gpsOffset = gpsOffset

        leftMotor: Motor = self.robot.getDevice(leftMotorDeviceName)
        rightMotor: Motor = self.robot.getDevice(rightMotorDeviceName)

        self.leftMotor = leftMotor
        self.rightMotor = rightMotor

        self.leftMotor.setPosition(float("inf"))
        self.rightMotor.setPosition(float("inf"))
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)

        sonarSensorNames = [f"so{i}" for i in range(8)]

        self.frontSonars = []
        for name in sonarSensorNames:
            sonar: DistanceSensor = self.robot.getDevice(name)
            sonar.enable(timestep)
            self.frontSonars.append(sonar)

        leftPosSensor: PositionSensor = self.robot.getDevice("left wheel sensor")
        self.leftPosSensor = leftPosSensor
        leftPosSensor.enable(timestep)

        rightPosSensor: PositionSensor = self.robot.getDevice("right wheel sensor")
        self.rightPosSensor = rightPosSensor
        rightPosSensor.enable(timestep)

        imu: InertialUnit = self.robot.getDevice("inertial unit")
        self.imu = imu
        imu.enable(timestep)

        gps: GPS = self.robot.getDevice("gps")
        self.gps = gps
        gps.enable(timestep)

        self.lidar_link = LidarLink(
            port="lidar_port",
            resolution=8,
            fov_range=(-90, 90),
        )

        self.lidar_link.start()

    def step(
        self,
        timeStepSeconds=None,
        **kw,
    ):
        timestep = int(self.robot.getBasicTimeStep())
        if not timeStepSeconds:
            return self.robot.step(timestep)

        start_time = self.robot.getTime()
        while self.robot.step(timestep) != -1:
            elapsed = self.robot.getTime() - start_time
            if elapsed >= timeStepSeconds:
                return

    def move_wheels(self, v_right, v_left):
        self.rightMotor.setVelocity(v_right)
        self.leftMotor.setVelocity(v_left)

    def state(self) -> tuple[float, float, float, float, float]:
        x, y, _ = self.gps.getValues()
        roll, pitch, yaw = self.imu.getRollPitchYaw()

        distances = []
        for s in self.frontSonars:
            value = s.getValue()
            distance_m = 5.0 * (1.0 - value / 1024.0)
            distance_m = max(0.0, min(distance_m, 5.0))
            distances.append(distance_m)

        lidars = None

        if self.lidar_link:
            lidars = self.lidar_link.get_state()
            if lidars is not None:
                distances = [l for l in lidars]

        if not distances:
            distances = [5 for _ in range(8)]
        print("===>", distances)

        return (
            x + self.gpsOffset[0],
            y + self.gpsOffset[1],
            math.degrees(yaw),
            *distances,
        )

    def reset(
        self,
        pos: tuple[float, float] = None,
        **kw,
    ):
        self.theta = 0.0
        self.x = 0.0
        self.y = 0.0
        self.v = 0.0
        self.omega = 0.0

        if pos:
            self.x = pos[0]
            self.y = pos[1]
            self.theta = pos[2]
