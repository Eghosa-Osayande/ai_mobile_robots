import math


def calculateNavigation(p1: tuple, p2: tuple, current_th: float):
    x1, y1 = p1
    x2, y2 = p2

    # Calculate Euclidean Distance
    dx = x2 - x1
    dy = y2 - y1
    distance = math.sqrt(dx**2 + dy**2)

    # Calculate the absolute angle to the target point
    target_angle = math.atan2(dy, dx)

    # Calculate the relative turn needed
    relative_turn = target_angle - current_th

    # Normalize the turn to be between -pi and pi
    # This ensures the robot takes the "shortest" turn
    relative_turn = math.atan2(math.sin(relative_turn), math.cos(relative_turn))

    return distance, relative_turn


def turnTime(
    v,
    wheel_separation: float,
    turn_radians: float,
) -> float:

    # Robot angular velocity (rad/s)
    angular_velocity = (2 * v) / wheel_separation

    if angular_velocity == 0:
        print("Cannot turn: wheel velocities are equal, no rotation occurs.")
        return 0

    # Time = angle / angular velocity
    time_to_turn = abs(turn_radians / angular_velocity)
    return time_to_turn


def travelTime(v, dist: float) -> float:
    if abs(v) > 0:
        return abs(dist / v)
    return 0
