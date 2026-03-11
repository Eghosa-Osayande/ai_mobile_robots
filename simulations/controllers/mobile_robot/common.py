from two_wheel_robots.two_wheel_robot_base import TwoWheelRobotBase
from two_wheel_robots.integrations.camera_link import (
    CameraLink,
)


class RobotSingleton:
    _instance: TwoWheelRobotBase = None

    @classmethod
    def set(cls, robot: TwoWheelRobotBase):
        RobotSingleton._instance = robot


class CameraLinkSingleton:
    _instance: CameraLink = None

    @classmethod
    def set(cls, cam: TwoWheelRobotBase):
        CameraLinkSingleton._instance = cam


def robot_instance():
    return RobotSingleton._instance


def camera_instance():
    return CameraLinkSingleton._instance
