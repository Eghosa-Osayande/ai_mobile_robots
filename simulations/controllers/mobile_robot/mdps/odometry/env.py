from __future__ import annotations
import math
import time

from two_wheel_robots.two_wheel_robot_base import TwoWheelRobotBase


def heading_error(p, g):
    x, y, th = p
    x2, y2 = g

    dx = x2 - x
    dy = y2 - y

    goal_heading = math.degrees(math.atan2(dy, dx))

    err = goal_heading - th

    # wrap to [-180, 180]
    err = (err + 180) % 360 - 180

    return err


class OdometryEnv:

    def __init__(
        self,
        robot: TwoWheelRobotBase,
        goal: tuple[float, float],
        approach_v_ms,
        turn_v_ms,
        dist_err_thres=0,
        heading_err_thres=0,
    ):
        super().__init__()
        self.robot = robot
        self.goal = goal
        self.approach_v_ms = approach_v_ms
        self.turn_v_ms = turn_v_ms
        self.dist_err_thres = dist_err_thres
        self.heading_err_thres = heading_err_thres
        self._turning_complete = False

    def _obs(self):
        x1, y1, th, *scans = self.robot.state()

        x2, y2 = self.goal[:2]

        # Calculate Euclidean Distance
        dx = x2 - x1
        dy = y2 - y1

        dist_err = math.sqrt(dx**2 + dy**2)

        heading_err = heading_error((x1, y1, th), (x2, y2))

        print(heading_err)

        if abs(heading_err) < self.heading_err_thres and not self._turning_complete:
            self._turning_complete = True

        return [
            (x1, y1, th),
            scans,
            self.goal,
            dist_err,
            heading_err,
            self._turning_complete,
        ]

    def close(self): ...

    def reset(
        self,
        goal=None,
    ):
        self.goal = self.goal if goal is None else goal
        self._turning_complete = False
        return self._obs(), {}

    def step(self, action):

        vr, vl, dt = action

        velocity = self.approach_v_ms if vr == vl else self.turn_v_ms

        self.robot.move_wheels(
            vr * velocity,
            vl * velocity,
        )

        self.robot.step(dt)

        obs = self._obs()

        pos, scans, goal, dist_err, heading_err, turning_complete = obs

        terminated = dist_err <= self.dist_err_thres
        if terminated:
            self.robot.move_wheels(0, 0)

        self.render(obs=obs)

        info = {
            "time": time.time(),
            "x": pos[0],
            "y": pos[1],
            "th": pos[2],
            "gx": goal[0],
            "gy": goal[1],
            "is_at_goal": terminated,
            "v": velocity,
            "vr": vr,
            "vl": vl,
            "env": "odom",
        }

        return obs, 0, terminated, False, info

    def render(self, obs=None): ...
