from dataclasses import dataclass
from two_wheel_robot_base import TwoWheelRobot
from controller import Robot, Motor, DistanceSensor, PositionSensor, GPS


@dataclass
class TwoWheelRobotWebot(TwoWheelRobot):

    robot: Robot
    leftMotor: Motor
    rightMotor: Motor
    frontSonars: list[DistanceSensor]
    wheelRadius: float
    wheelSeperation: float
    gps: GPS
    gpsOffset: tuple[float, float]

    def __init__(
        self,
        leftMotorDeviceName: str,
        rightMotorDeviceName: str,
        wheelRadius: float,
        wheelSeperation: float,
        gpsOffset: tuple[float, float] = (0, 0),
    ):

        self.robot = Robot()
        timestep = int(self.robot.getBasicTimeStep())

        self.wheelRadius = wheelRadius
        self.wheelSeperation = wheelSeperation
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

        gps: GPS = self.robot.getDevice("gps")
        self.gps = gps
        gps.enable(timestep)

    # def move(self, velocity: float):
    #     wheel_rad_s = velocity / self.wheelRadius
    #     self.rightMotor.setVelocity(wheel_rad_s)
    #     self.leftMotor.setVelocity(wheel_rad_s)

    # def rotate(self, velocity: float):
    #     wheel_rad_s = velocity / self.wheelRadius
    #     self.leftMotor.setVelocity(wheel_rad_s)
    #     self.rightMotor.setVelocity(-wheel_rad_s)

    def step(self, timeStepSeconds):
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

    def state(self) -> tuple[float, float, float]:
        x, y, _ = self.gps.getValues()
        return (
            x + self.gpsOffset[0],
            y + self.gpsOffset[1],
            0,
        )
