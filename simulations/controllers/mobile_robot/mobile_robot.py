import os
import json
from datetime import datetime
import world_core, world_rl

isSerial = os.environ.get("IS_WEBOTS") != "true"

robot_2wd = None
world_id = "world_rl"

if isSerial:
    from two_wheel_robots.pioneer3dx import Pioneer3dx

    lidar_port = "/dev/cu.usbserial-0001"
    lidar_port = "/dev/ttyUSB0"
    # lidar_port = ("localhost", 5000)
    # lidar_port = None

    # serial_port_baud = ("/dev/tty.usbserial-10", 9600)
    serial_port_baud = ("/dev/ttyUSB1", 9600)
    # serial_port_baud = None

    # tcp_host_port = ("127.0.0.1", 8008)
    tcp_host_port = None

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
        leftMotorDeviceName="left wheel",
        rightMotorDeviceName="right wheel",
        gpsOffset=(0, 0),
    )

    robot_webot.step(int(robot_webot.getBasicTimeStep()))

    world_id = robot_webot.getCustomData()


runner = {
    "world_core": world_core,
    "world_rl": world_rl,
}[world_id]


current_dir = os.path.dirname(os.path.abspath(__file__))

ts = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")

logs_path = (
    f"{current_dir}/logs/{world_id}/{ts}"
)

os.makedirs(
    logs_path,
    exist_ok=True,
)

infos = runner.run(robot_2wd, logs_path)

info_file = f"{logs_path}/infos.json"

with open(info_file, "w") as fd:
    print("Saving logs")
    json.dump(infos, fd)
