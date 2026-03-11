from __future__ import annotations
import math
import time

from two_wheel_robots.two_wheel_robot_base import TwoWheelRobotBase

import matplotlib.pyplot as plt


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
        render_filename="",
    ):
        super().__init__()
        self.robot = robot
        self.goal = goal
        self._traj = []
        self._last_cmd = None
        self.render_filename = render_filename

    def _obs(self):
        x1, y1, th, corridor,*_ = self.robot.state()

        x2, y2 = self.goal[:2]

        # Calculate Euclidean Distance
        dx = x2 - x1
        dy = y2 - y1

        dist_err = math.sqrt(dx**2 + dy**2)

        heading_err = heading_error((x1, y1, th), (x2, y2))

        return [
            (x1, y1, th),
            corridor,
            self.goal,
            dist_err,
            heading_err,
        ]

    def close(self): ...

    def reset(
        self,
        goal=None,
    ):
        self.goal = self.goal if goal is None else goal
        # self._traj = []
        self._last_cmd = None
        return self._obs(), {}

    def step(self, action):

        vr, vl, dt = action

        if self._last_cmd != (vr, vl):
            self._last_cmd = (vr, vl)
            self.robot.move_wheels(
                vr,
                vl,
            )

        self.robot.step(dt)
        obs = self._obs()

        pos, scans, goal, dist_err, heading_err, *_ = obs

        self._traj.append(pos[:2])

        terminated = action == (0, 0, 0)

        self.render(obs=obs)

        info = {
            "time": time.time(),
            "x": float(pos[0]),
            "y": float(pos[1]),
            "th": float(pos[2]),
            "gx": float(goal[0]),
            "gy": float(goal[1]),
            "is_at_goal": terminated,
            "vr": float(vr),
            "vl": float(vl),
            "env": "odom",
        }

        return obs, 0, terminated, False, info

    def render(self, obs=None, path=None):
        if obs is None:
            obs = self._obs()

        if not self.render_filename and not path:
            return

        if len(self._traj) < 1:
            return

        pos, _, goal, *_ = obs
        x, y, theta = pos

        xs = [p[0] for p in self._traj]
        ys = [p[1] for p in self._traj]

        plt.clf()

        plt.plot(xs, ys)
        plt.scatter(xs[0], ys[0])
        plt.scatter(x, y)

        arrow_len = 0.5
        dx = arrow_len * math.cos(theta)
        dy = arrow_len * math.sin(theta)
        # plt.arrow(
        #     x,
        #     y,
        #     dx,
        #     dy,
        #     head_width=0.15,
        #     head_length=0.2,
        #     length_includes_head=True,
        # )

        goal_radius = 0
        plt.scatter(goal[0], goal[1])
        plt.gca().add_patch(
            plt.Circle(
                (goal[0], goal[1]),
                goal_radius,
                fill=False,
            )
        )
        arena_radius = 7
        obs_norm_radius = 7
        if arena_radius is not None:
            plt.gca().add_patch(plt.Circle((0, 0), arena_radius, fill=False))

        plt.gca().set_aspect("equal", adjustable="box")
        plt.xlim(-obs_norm_radius, obs_norm_radius)
        plt.ylim(-obs_norm_radius, obs_norm_radius)
        plt.savefig(self.render_filename if not path else path)
