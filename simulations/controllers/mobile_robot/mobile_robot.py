import os
import sys
import common
from dotenv import load_dotenv
from two_wheel_robots.integrations.camera_link import (
    CameraLink,
)

load_dotenv()

pioneer_serial_port = os.getenv("PIONEER_SERIAL_PORT")

pioneer_serial_baud = os.getenv("PIONEER_SERIAL_BAUD", "9600")

pioneer_tcp_host = os.getenv("PIONEER_TCP_HOST")

pioneer_tcp_port = int(os.getenv("PIONEER_TCP_PORT", "8080"))

camera_src = os.getenv("CAMERA_SRC", "0")

webot_left_wheel = os.getenv("WEBOT_LEFT_WHEEL", "left wheel")

webot_right_wheel = os.getenv("WEBOT_RIGHT_WHEEL", "right wheel")

lidar_port = os.getenv("LIDAR_PORT")

is_serial = os.environ.get("IS_WEBOTS") != "true"

experiment_id = ""

robot_2wd = None


camera_link = CameraLink(
    src=camera_src,
)


if is_serial:
    from two_wheel_robots.pioneer3dx import Pioneer3dx

    serial_port_baud = None
    tcp_host_port = None

    if pioneer_serial_port and pioneer_serial_baud:
        serial_port_baud = (
            pioneer_serial_port,
            int(pioneer_serial_baud),
        )

    if pioneer_tcp_host and pioneer_tcp_port:
        tcp_host_port = (
            pioneer_tcp_host,
            int(pioneer_tcp_port),
        )

    robot_2wd = Pioneer3dx(
        serial_port_baud=serial_port_baud,
        tcp_host_port=tcp_host_port,
        lidar_port=lidar_port,
    )

else:
    from two_wheel_robots.pioneer3dx_webot import Pioneer3dxWebot
    from controller import Robot

    robot_webot = Robot()
    robot_2wd = Pioneer3dxWebot(
        robot_webot,
        leftMotorDeviceName=webot_left_wheel,
        rightMotorDeviceName=webot_right_wheel,
    )

    robot_webot.step(int(robot_webot.getBasicTimeStep()))

common.RobotSingleton.set(robot_2wd)
common.CameraLinkSingleton.set(camera_link)


from experiments import exp1, exp2, exp3

try:
    experiment_id = sys.argv[1]
except:
    ...

eval(experiment_id).main()
