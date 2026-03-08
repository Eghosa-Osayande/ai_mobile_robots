# CONFIG

# webot config
leftMotorDeviceName = "left wheel"
rightMotorDeviceName = "right wheel"

# pioneer config
serial_port_baud = ("/dev/tty.usbserial-10", 9600)
serial_port_baud = ("/dev/ttyUSB0", 9600)
serial_port_baud = None
tcp_host_port = ("127.0.0.1", 8008)
# tcp_host_port = None

# odometry
wheelSeperation = 0.32125
odometry_approach_v_ms = 0.02 * 8
odometry_turn_v_ms = 0.02 * 3
goal_dist_thres = 0.1
goal_heading_thres = 2.5
odometry_nodes = [
    # 1
    (1.1, 0),
    (2.1, 0),
    # 2
    # (2.1, -1.3),
    (2.1, -2.3),
    # 3
    (3.2, -2.4),
    (3.8, -2.4, 0.02 * 5),
    (4.4, -2.4),
    # 4
    (4.4, -3.7),
    # 5
    (3.4, -3.7),
    (0, -3.9),
    (-1.4, -3.9),
    #
    (-1.4, -3.899),
]

odometry_nodes = [
    # 1
    (1.1, 0),
    (2.1, 0),
    # 2
    (2.1, -1.3),
    (2.1, -2.3),
    # 3
    (3.2, -2.4),
    (3.8, -2.4, 0.02 * 5),
    (4.4, -2.4),
    # 4
    (4.4, -3.7),
    # 5
    (2, -3.7),
    (-1.4, -3.7),
    #
    (-1.4, -3.7),
]



# avoid and track
tracking_approach_v_ms = 0.08
tracking_turn_v_ms = 0.04
avoidance_v_ms = 0.06
tracking_dt = 0.01
avoidance_dt = 0.01

target_color_hex = "#ff0000"
use_cascade = False
area_constraint = None

avoid_dist_thres = 0.3
front_idxs = [2, 3, 4, 5]
left_idxs = [0, 1, 2, 3]
right_idxs = [4, 5, 6, 7]

camera_index = 1
camera_tcp_addr = ("0.0.0.0", 8081)
camera_tcp_addr = None

lidar_port = "/dev/cu.usbserial-0001"
# lidar_port = None

import os

from two_wheel_robots.integrations.camera_client import (
    CamClient,
)

isSerial = os.environ.get("IS_WEBOTS") != "true"

robot_2wd = None
worldID = None

if isSerial:
    from two_wheel_robots.pioneer3dx import Pioneer3dx

    robot_2wd = Pioneer3dx(
        serial_port_baud=serial_port_baud,
        tcp_host_port=tcp_host_port,
        lidar_port=lidar_port,
    )

    worldID = "core"

else:
    from two_wheel_robots.pioneer3dx_webot import Pioneer3dxWebot
    from controller import Robot

    robot_webot = Robot()
    robot_2wd = Pioneer3dxWebot(
        robot_webot,
        leftMotorDeviceName=leftMotorDeviceName,
        rightMotorDeviceName=rightMotorDeviceName,
        gpsOffset=(0, 0),
    )

    robot_webot.step(int(robot_webot.getBasicTimeStep()))

    worldID = robot_webot.getCustomData()

infos = []

if worldID == "core":
    from mdps import (
        OdometryEnv,
        OdometryAgent,
        AvoidAndTrackingEnv,
        AvoidAndTrackAgent,
    )

    # Odometry
    print("Odometry Start")

    odom_agent = OdometryAgent(
        approach_v_ms=odometry_approach_v_ms,
        turn_v_ms=odometry_turn_v_ms,
        wheel_seperation=wheelSeperation,
    )

    odom_env = OdometryEnv(
        robot=robot_2wd,
        goal=(0, 0),
    )
    odom_env.render()
    robot_2wd.step(3)

    for node in odometry_nodes:
        obs, _ = odom_env.reset(
            goal=node,
        )

        while True:
            action = odom_agent.act(obs)
            obs, _, terminated, truncated, info = odom_env.step(action)
            infos.append(info)
            if terminated or truncated:
                break

    print("Odometry End")

    # Tracking/Avoidance

    avoid_track_agent = AvoidAndTrackAgent(
        dt=tracking_dt,
    )

    avoid_track_env = AvoidAndTrackingEnv(
        robot=robot_2wd,
        cam_client=CamClient(
            src=camera_index,
            tcp=camera_tcp_addr,
        ),
        # avoid
        avoid_dist_thres=avoid_dist_thres,
        front_idxs=front_idxs,
        left_idxs=left_idxs,
        right_idxs=right_idxs,
        avoid_velocity=avoidance_v_ms,
        # track
        target_color_hex=target_color_hex,
        approach_v=tracking_approach_v_ms,
        turn_v=tracking_turn_v_ms,
        area_constraint=area_constraint,
        use_cascade=use_cascade,
    )

    avoid_track_obs, _ = avoid_track_env.reset()

    print("Tracking/Avoidance Start")
    robot_2wd.step(5)

    while True:

        avoid_track_action = avoid_track_agent.act(avoid_track_obs)

        avoid_track_obs, _, terminated, truncated, info = avoid_track_env.step(
            avoid_track_action
        )

        infos.append(info)

        if terminated or truncated:
            break

    print("Tracking/Avoidance End")

    avoid_track_env.close()
    odom_env.close()

    print("Saving logs")
    import json
    import time

    with open(f"mobile_robot_logs_{time.time()}.json", "w") as fd:
        json.dump(infos, fd)


if worldID == "contrib":
    ...
