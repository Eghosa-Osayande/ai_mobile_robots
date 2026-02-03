import math
from p3dx_robot import P3DX_Robot
from helpers import calculateNavigation, turnTime, travelTime
import config


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
velocity = 1


print("starting loop")
robot.step(2)
currentPose = (0, 0, 0)

unitTime = 0.5

for i, node in enumerate(config.nodes[::1], 0):
    if i == 0:
        currentPose = (*node, 0)
        continue

    target = robot.getActualPose()

    dist, turn = calculateNavigation(
        currentPose[:2],
        node,
        currentPose[2],
    )

    timeToTurn = turnTime(
        velocity * unitVelocity,
        config.wheelSeperation,
        turn,
    )

    timeToTravel = travelTime(velocity * unitVelocity, dist)

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
        robot.rotate((-1 if turn > 0 else 1) * velocity)

        # while timeToTurn > 0:
        #     robot.step(unitTime if timeToTurn > unitTime else timeToTurn)

        #     timeToTurn -= unitTime

        #     actualPose = robot.getActualPose()

        #     print(actualPose)

        robot.step(timeToTurn-0.0)
        robot.stop()
        robot.step(1)

    if timeToTravel > 0 and 1 == 1:
        robot.move(velocity)
        robot.step(timeToTravel)
        robot.stop()
        robot.step(1)

    currentPose = (
        node[0],
        node[1],
        currentPose[2] + turn,
    )
