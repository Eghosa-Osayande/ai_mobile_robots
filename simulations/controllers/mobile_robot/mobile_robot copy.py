# CONFIG

# webot config
leftMotorDeviceName = "left wheel"
rightMotorDeviceName = "right wheel"

# pioneer config
serial_port_baud = ("/dev/tty.usbserial-10", 9600)
# serial_port_baud = ("/dev/ttyUSB0", 9600)
# serial_port_baud = None
# tcp_host_port = ("127.0.0.1", 8008)
tcp_host_port = None

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
lidar_port = ("localhost", 5000)
# lidar_port = None

import math
import os

from two_wheel_robots.integrations.camera_link import (
    CameraLink,
)

isSerial = os.environ.get("IS_WEBOTS") != "true"

robot_2wd = None
world_id = "rl"

if isSerial:
    from two_wheel_robots.pioneer3dx import Pioneer3dx

    robot_2wd = Pioneer3dx(
        serial_port_baud=serial_port_baud,
        tcp_host_port=tcp_host_port,
        lidar_port=lidar_port,
    )

    world_id = world_id if world_id else "core"

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

    world_id = world_id if world_id else robot_webot.getCustomData()

infos = []

if world_id == "core":
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
        cam_client=CameraLink(
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

if world_id == "contrib":
    from stable_baselines3 import SAC
    from contribution.rl.mdp.navigation.env import TwoWheelNavigationEnv

    odometry_nodes = [
        (1, 0),
        (1, -2.3),
        (2, -2.3),
        # (2.1, 0),
        # (2.1, -2.3),
        # (4.4, -2.3),
        # (4.4, -3.7),
        # (2, -3.7),
        # (-1.4, -3.7),
    ]

    model = SAC.load("contribution/rl/training_outputs/navigation/model.zip")

    evn = TwoWheelNavigationEnv(
        robot_factory=lambda *_, **kw: robot_2wd,
        wheel_speed_limit=0.02 * 10,
        dt=0.0001,
        max_steps=1000,
        goal_radius=0.2,
        arena_radius=6.0,
        obs_norm_radius=6.0,
        render_enabled=True,
        render_path="output.png",
        robot_theta_tranformer=lambda th: math.radians(th),
    )

    for node in odometry_nodes:
        obs, _ = evn.reset(
            goal_xy=node,
            reset_robot=False,
        )

        while True:
            action, _ = model.predict(obs)
            obs, _, terminated, truncated, _ = evn.step(action[::-1])

            x, y, t, *_ = robot_2wd.state()
            print(x, y, t)

            if terminated or truncated:
                break

    evn.close()

    print("Done")


if world_id == "rl":
    from stable_baselines3 import SAC
    from contribution.rl.mdp.obstacle_navigation.env import (
        TwoWheelObstacleNavigationEnv,
    )

    odometry_nodes = [
        (2, 0),
        (2, -2.3),
        (4.4, -2.3),
        (4.4, -3.7),
        (3.4, -3.7),
        (2.4, -3.7),
        (1.4, -3.7),
        (0.4, -3.7),
        (-1.4, -3.7),
    ]

    model = SAC.load(
        "contribution/rl/training_outputs/obstacle_navigation/checkpoints/_1680000_steps.zip"
    )

    evn = TwoWheelObstacleNavigationEnv(
        robot_factory=lambda *_, **kw: robot_2wd,
        wheel_speed_limit=0.02 * 7,
        dt=0.001,
        max_steps=999,
        goal_radius=0.15,
        arena_radius=6.0,
        obs_norm_radius=6.0,
        render_enabled=True,
        allow_reverse=True,
        robot_theta_tranformer=lambda th: math.radians(th),
        render_path="output.png",
        safe_distance=0.2,
    )

    for node in odometry_nodes:
        obs, _ = evn.reset(
            goal_xy=node,
            reset_robot=False,
        )

        print(obs)

        while True:
            action, _ = model.predict(obs)
            obs, _, terminated, truncated, _ = evn.step(action[::-1])

            x, y, t, *_ = robot_2wd.state()
            # print(x, y, t)

            if terminated or truncated:
                break

    evn.close()

    print("Done")


print("Saving logs")
import json
import time

logs_path = f"/Users/mac/Desktop/mr/simulations/controllers/mobile_robot/logs/{world_id}/{time.time()}.json"

os.makedirs(
    os.path.dirname(logs_path),
    exist_ok=True,
)

with open(logs_path, "w") as fd:
    json.dump(infos, fd)
