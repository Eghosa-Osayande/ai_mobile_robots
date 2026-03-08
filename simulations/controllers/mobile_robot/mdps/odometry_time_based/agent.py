import math
import time


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


class OdometryAgent:
    def __init__(
        self,
        wheel_seperation,
        approach_v_ms,
        turn_v_ms,
    ):
        self.wheel_separation = wheel_seperation
        self.approach_v_ms = approach_v_ms
        self.turn_v_ms = turn_v_ms

    def act(self, obs):
        pos, scans, goal, dist_err, heading_err, prev_action, *_ = obs

        if prev_action is not None:
            return prev_action

        config = [(0, 0, 0, None), (0, 0, 0, None)]


        if abs(heading_err) > 0:
            vr = -1 if heading_err > 0 else 1
            vl = -vr

            v = self.turn_v_ms
            if len(goal) > 2:
                v = goal[2]

            dt = turnTime(
                wheel_separation=self.wheel_separation,
                turn_radians=math.radians(heading_err),
                v=v,
            )

            config[0] = (
                v * vr,
                v * vl,
                dt,
                None,
            )

        if abs(dist_err) > 0:

            v = self.approach_v_ms
            if len(goal) > 2:
                v = goal[2]

            config[1] = (
                v,
                v,
                dist_err / v,
                None,
            )

        return config
