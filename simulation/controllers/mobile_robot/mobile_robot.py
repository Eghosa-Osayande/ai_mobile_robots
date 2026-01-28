import math
from lab_config import LabConfig
from p3dx_robot import P3DX_Robot
from helpers import calculateNavigation, turnTime, travelTime


config = LabConfig()

robot: P3DX_Robot = None


if config.isSerial:
    from p3dx_robot_serial import P3DX_Robot_Serial

    robot = P3DX_Robot_Serial(
        port=config.port,
    )
else:
    from p3dx_robot_webot import P3DX_Robot_Webot

    robot = P3DX_Robot_Webot(
        leftMotorDeviceName=config.leftMotorDeviceName,
        rightMotorDeviceName=config.rightMotorDeviceName,
        wheelRadius=config.wheelRadius,
        wheelSeperation=config.wheelSeperation,
        gpsOffset=(2, -2),
    )

unitVelocity = 0.02
velocity = 10


print("starting loop")
robot.step(2)
currentPose = (0, 0, 0)
for i, node in enumerate(config.nodes, 1):
    target = robot.getActualPose()

    dist, turn = calculateNavigation(
        currentPose[:2],
        node,
        currentPose[2],
    )

    print(
        f"Going to node {i} -> {node}. Turn {math.degrees(turn)} degrees and travel {dist} meters, Current pose {currentPose[:2]} {math.degrees(currentPose[2])} degrees"
    )

    timeToTurn = turnTime(
        velocity * unitVelocity,
        config.wheelSeperation,
        turn,
    )

    if timeToTurn > 0:
        robot.rotate((-1 if turn > 0 else 1) * velocity)
        robot.step(timeToTurn)
        robot.stop()
        robot.step(1)

    timeToTravel = travelTime(velocity * unitVelocity, dist)

    if timeToTravel > 0:
        robot.move(velocity)
        robot.step(timeToTravel)
        robot.stop()
        robot.step(1)

    currentPose = (
        node[0],
        node[1],
        currentPose[2] + turn,
    )
