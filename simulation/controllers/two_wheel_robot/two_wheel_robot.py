import math
from two_wheel_robot_base import TwoWheelRobot
from helpers import calculateNavigation, turnTime, travelTime
import config


robot: TwoWheelRobot = None


if config.isSerial:
    from two_wheel_robot_p3dx import TwoWheelRobot_P3DX

    robot = TwoWheelRobot_P3DX(
        port=config.port,
    )
else:
    from two_wheel_robot_webot import TwoWheelRobotWebot

    robot = TwoWheelRobotWebot(
        leftMotorDeviceName=config.leftMotorDeviceName,
        rightMotorDeviceName=config.rightMotorDeviceName,
        wheelRadius=config.wheelRadius,
        wheelSeperation=config.wheelSeperation,
        gpsOffset=(2, -2),
    )


velocity = 1 * 0.02
if config.isSerial:
    velocity = int(velocity / 0.02) * 0.02


print("starting loop")
robot.step(2)
currentPose = (0, 0, 0)

unitTime = 0.5

for i, node in enumerate(config.nodes[::1], 0):
    if i == 0:
        currentPose = (*node, 0)
        continue

    dist, turn = calculateNavigation(
        currentPose[:2],
        node,
        currentPose[2],
    )

    # TODO: use v_right and v_left below

    timeToTurn = turnTime(
        velocity,
        config.wheelSeperation,
        turn,
    )

    timeToTravel = travelTime(velocity, dist)

    print(
        f"""
Current pose {currentPose[:2]} {math.degrees(currentPose[2])} degrees
Going to node {i} -> {node}. 
Turn {math.degrees(turn)} degrees
Travel {dist} meters
Turn time {timeToTurn}s
Travel time {timeToTravel}s"""
    )

    if timeToTurn > 0 and 1 == 1:
        f = -1 if turn > 0 else 1
        robot.move_wheels(f * velocity, f * -velocity)

        robot.step(timeToTurn - 0.0)
        robot.move_wheels(0, 0)
        robot.step(1)

    if timeToTravel > 0 and 1 == 1:
        robot.move_wheels(velocity, velocity)
        robot.step(timeToTravel)
        robot.move_wheels(0, 0)
        robot.step(1)

    currentPose = (
        node[0],
        node[1],
        currentPose[2] + turn,
    )
