import os
import json
from datetime import datetime
import world_core, world_rl

ts = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")

isSerial = os.environ.get("IS_WEBOTS") != "true"

robot_2wd = None
world_id = "world_core"

if isSerial:
    from two_wheel_robots.pioneer3dx import Pioneer3dx

    lidar_port = "/dev/cu.usbserial-0001"
    # lidar_port = ("localhost", 5000)
    # lidar_port = None

    # serial_port_baud = ("/dev/tty.usbserial-10", 9600)
    serial_port_baud = ("/dev/ttyUSB0", 9600)
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


logs_path = (
    f"/Users/mac/Desktop/mr/simulations/controllers/mobile_robot/logs/{world_id}/{ts}"
)

os.makedirs(
    os.path.dirname(logs_path),
    exist_ok=True,
)

infos = runner.run(robot_2wd, world_id)

info_file = f"{logs_path}/infos.json"

with open(info_file, "w") as fd:
    print("Saving logs")
    json.dump(infos, fd)
