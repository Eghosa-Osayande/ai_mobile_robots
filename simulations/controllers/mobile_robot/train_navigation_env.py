from __future__ import annotations

import csv
import math
import os
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3.common.callbacks import BaseCallback

import csv
import os
from collections import deque
from dataclasses import dataclass
from typing import Optional

import numpy as np
from dataclasses import dataclass
import math
import numpy as np

import math
import numpy as np

from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv


class TwoWheelRobotWithoutObstacles:
    def __init__(
        self,
        wheel_distance,
    ):
        assert wheel_distance > 0

        self.wheel_distance = float(wheel_distance)

        self.theta = 0.0
        self.x = 0.0
        self.y = 0.0
        self.v = 0.0
        self.omega = 0.0

    def reset(
        self,
        pos: tuple[float, float] = None,
        theta=None,
        **kw,
    ):
        self.theta = 0.0 if theta is None else float(theta)
        self.x = 0.0
        self.y = 0.0
        self.v = 0.0
        self.omega = 0.0

        if pos is not None:
            self.x = float(pos[0])
            self.y = float(pos[1])

    def state(self):

        return (
            self.x,
            self.y,
            self.theta,
        )

    def move_wheels(self, v_right, v_left):
        self.v = (v_right + v_left) * 0.5
        self.omega = (v_right - v_left) / self.wheel_distance

    def step(self, dt):
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt


class TwoWheelNavigationEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(
        self,
        *,
        robot_factory: Callable[[], Any],
        wheel_speed_limit: float = 0.02 * 10,
        dt: float = 0.1,
        max_steps: int = 250,
        goal_radius: float = 0.2,
        spawn_xy: Tuple[float, float] = (0.0, 0.0),
        spawn_theta: float = 0.0,
        spawn_sampler: Optional[
            Callable[[np.random.Generator], Tuple[Tuple[float, float], float]]
        ] = None,
        goal_xy: Tuple[float, float] = (3.0, 0.0),
        goal_xy_sampler: Optional[
            Callable[[np.random.Generator], Tuple[float, float]]
        ] = None,
        arena_radius: Optional[float] = 6.0,
        obs_norm_radius: float = 6.0,
        effort_penalty: float = 0.01,
        spin_penalty: float = 0.002,
        stall_penalty: float = 0.05,
        stall_window: int = 12,
        stall_tol: float = 1e-3,
        success_bonus: float = 10.0,
        oob_penalty: float = 2.0,
        render_enabled: bool = False,
        allow_reverse: bool = False,
        seed: Optional[int] = None,
        robot_theta_tranformer=None,
        render_path="",
    ):
        super().__init__()
        self.render_enabled = render_enabled
        self._trajectory: list[Tuple[float, float]] = []
        self._render_path = render_path
        self.robot_theta_tranformer = robot_theta_tranformer

        self.allow_reverse = bool(allow_reverse)

        self.robot_factory = robot_factory
        self.wheel_speed_limit = float(wheel_speed_limit)
        self.dt = float(dt)
        self.max_steps = int(max_steps)
        self.goal_radius = float(goal_radius)

        self.spawn_xy = tuple(map(float, spawn_xy))
        self.spawn_theta = float(spawn_theta)
        self.spawn_sampler = spawn_sampler

        self.goal_xy = tuple(map(float, goal_xy))
        self.goal_xy_sampler = goal_xy_sampler

        self.arena_radius = None if arena_radius is None else float(arena_radius)
        self.obs_norm_radius = float(obs_norm_radius)

        self.effort_penalty = float(effort_penalty)
        self.spin_penalty = float(spin_penalty)
        self.stall_penalty = float(stall_penalty)
        self.stall_window = int(stall_window)
        self.stall_tol = float(stall_tol)
        self.success_bonus = float(success_bonus)
        self.oob_penalty = float(oob_penalty)

        low = (
            np.array(
                [-self.wheel_speed_limit, -self.wheel_speed_limit], dtype=np.float32
            )
            if self.allow_reverse
            else np.array([0.0, 0.0], dtype=np.float32)
        )

        self.action_space = spaces.Box(
            low=low,
            high=np.array(
                [self.wheel_speed_limit, self.wheel_speed_limit], dtype=np.float32
            ),
            dtype=np.float32,
        )

        self.observation_space = spaces.Box(
            low=np.array([-1.0, -1.0, -1.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0, 1.0, 1.0], dtype=np.float32),
            dtype=np.float32,
        )

        self.np_random = np.random.default_rng(seed)
        self.robot = None
        self.goal = np.zeros(2, dtype=np.float32)
        self.steps = 0

        self.prev_dist: float = 0.0
        self.dist_hist: list[float] = []

        self._ep_start_xy: Tuple[float, float] = (0.0, 0.0)
        self._ep_path_len: float = 0.0
        self._ep_heading_abs_sum: float = 0.0
        self._ep_smooth_sum: float = 0.0
        self._ep_reverse_steps: int = 0
        self._ep_last_xy: Optional[Tuple[float, float]] = None
        self._ep_prev_action: Optional[Tuple[float, float]] = None

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        reset_robot: bool = True,
        goal_xy=None,
        robot_pos=None,
        options: Optional[Dict[str, Any]] = None,
    ):
        super().reset(seed=seed)
        if seed is not None:
            self.np_random = np.random.default_rng(seed)

        self.robot = self.robot_factory()

        if robot_pos:
            self.robot.reset(
                pos=robot_pos[:2],
                theta=robot_pos[2],
                obstacles=[],
            )

        if reset_robot and not robot_pos:
            if self.spawn_sampler is not None:
                (sx, sy), stheta = self.spawn_sampler(self.np_random)
            else:
                (sx, sy), stheta = self.spawn_xy, self.spawn_theta

            self.robot.reset(
                pos=(float(sx), float(sy)),
                theta=float(stheta),
                obstacles=[],
            )
            self._trajectory = []

        if goal_xy is not None:
            gx, gy, *_ = goal_xy
            self.goal_xy = (gx, gy)
            self.goal_xy_sampler = None

        if self.goal_xy_sampler is not None:
            gx, gy = self.goal_xy_sampler(self.np_random)
        else:
            gx, gy, *_ = self.goal_xy
        self.goal[:] = (float(gx), float(gy))

        self.steps = 0
        self.prev_dist = self._dist_to_goal()
        self.dist_hist = [self.prev_dist]

        x, y, *_ = self._get_robot_state()
        self._trajectory.append((x, y))
        self._ep_start_xy = (float(x), float(y))
        self._ep_last_xy = (float(x), float(y))
        self._ep_path_len = 0.0
        self._ep_heading_abs_sum = 0.0
        self._ep_smooth_sum = 0.0
        self._ep_reverse_steps = 0
        self._ep_prev_action = None

        obs = self._obs()
        info = {"goal": self.goal.copy()}
        return obs, info

    def step(self, action):
        self.steps += 1

        a = np.asarray(action, dtype=np.float32)
        a = np.clip(a, self.action_space.low, self.action_space.high)
        v_r, v_l = float(a[0]), float(a[1])
        if not self.allow_reverse:
            v_r = max(0.0, v_r)
            v_l = max(0.0, v_l)

        self.robot.move_wheels(v_r, v_l)
        self.robot.step(self.dt)

        x, y, theta, *_ = self._get_robot_state()

        dx = float(self.goal[0] - x)
        dy = float(self.goal[1] - y)
        c = math.cos(theta)
        s = math.sin(theta)
        gx_r = c * dx + s * dy
        gy_r = -s * dx + c * dy
        heading_err = float(abs(math.atan2(gy_r, gx_r)))

        if self._ep_last_xy is not None:
            px, py = self._ep_last_xy
            self._ep_path_len += float(math.hypot(x - px, y - py))
        self._ep_last_xy = (float(x), float(y))

        self._ep_heading_abs_sum += heading_err
        self._ep_reverse_steps += int(v_r < 0.0 and v_l < 0.0)

        if self._ep_prev_action is not None:
            pr, pl = self._ep_prev_action
            self._ep_smooth_sum += float(abs(v_r - pr) + abs(v_l - pl))
        self._ep_prev_action = (v_r, v_l)

        d = float(math.hypot(self.goal[0] - x, self.goal[1] - y))

        success = d <= self.goal_radius

        reward = float(self.prev_dist - d)
        reward -= self.effort_penalty * (abs(v_r) + abs(v_l))
        reward -= self.spin_penalty * abs(v_r - v_l)

        self.dist_hist.append(d)
        if len(self.dist_hist) > self.stall_window:
            self.dist_hist.pop(0)
            if (self.dist_hist[0] - self.dist_hist[-1]) < self.stall_tol:
                reward -= self.stall_penalty

        if success:
            reward += self.success_bonus

        terminated = bool(success)
        truncated = self.steps >= self.max_steps

        if self.arena_radius is not None:
            if float(np.hypot(x, y)) > self.arena_radius:
                truncated = True
                reward -= self.oob_penalty

        self.prev_dist = d
        obs = self._obs()

        info = {
            "dist_to_goal": d,
            "success": bool(success),
            "heading_error": heading_err,
            "v_r": v_r,
            "v_l": v_l,
        }

        if terminated or truncated:
            sx, sy = self._ep_start_xy
            straight = float(math.hypot(self.goal[0] - sx, self.goal[1] - sy))
            path = float(self._ep_path_len)
            eff = float(straight / max(path, 1e-9))

            ep_len = int(self.steps)
            info.update(
                {
                    "ep_final_dist": float(d),
                    "ep_path_len": path,
                    "ep_straight_dist": straight,
                    "ep_path_eff": eff,
                    "ep_reverse_ratio": float(self._ep_reverse_steps / max(ep_len, 1)),
                    "ep_heading_abs_mean": float(
                        self._ep_heading_abs_sum / max(ep_len, 1)
                    ),
                    "ep_action_smooth_mean": float(
                        self._ep_smooth_sum / max(ep_len - 1, 1)
                    ),
                }
            )

        self._trajectory.append((x, y))
        self.render()

        return obs, reward, terminated, truncated, info

    def _dist_to_goal(self) -> float:
        x, y, *_ = self._get_robot_state()
        return float(np.hypot(self.goal[0] - x, self.goal[1] - y))

    def _obs(self) -> np.ndarray:
        x, y, theta, *_ = self._get_robot_state()
        dx = float(self.goal[0] - x)
        dy = float(self.goal[1] - y)

        c = math.cos(theta)
        s = math.sin(theta)
        gx_r = c * dx + s * dy
        gy_r = -s * dx + c * dy

        r = max(self.obs_norm_radius, 1e-6)
        gx_n = float(np.clip(gx_r / r, -1.0, 1.0))
        gy_n = float(np.clip(gy_r / r, -1.0, 1.0))

        return np.array([gx_n, gy_n, float(c), float(s)], dtype=np.float32)

    def close(self):
        self.robot.move_wheels(0, 0)

    def render(self):
        if not self.render_enabled:
            return
        if not self._trajectory:
            return

        xs = [p[0] for p in self._trajectory]
        ys = [p[1] for p in self._trajectory]

        plt.clf()

        plt.plot(xs, ys)
        plt.scatter(xs[0], ys[0])

        x, y, theta, *_ = self._get_robot_state()
        plt.scatter(x, y)

        arrow_len = 0.5
        dx = arrow_len * math.cos(theta)
        dy = arrow_len * math.sin(theta)
        plt.arrow(
            x,
            y,
            dx,
            dy,
            head_width=0.15,
            head_length=0.2,
            length_includes_head=True,
        )

        plt.scatter(self.goal[0], self.goal[1])
        plt.gca().add_patch(
            plt.Circle((self.goal[0], self.goal[1]), self.goal_radius, fill=False)
        )

        if self.arena_radius is not None:
            plt.gca().add_patch(plt.Circle((0, 0), self.arena_radius, fill=False))

        plt.gca().set_aspect("equal", adjustable="box")
        plt.xlim(-self.obs_norm_radius, self.obs_norm_radius)
        plt.ylim(-self.obs_norm_radius, self.obs_norm_radius)
        if self._render_path:
            plt.savefig(self._render_path)
        # plt.show()

    def _get_robot_state(self):
        x, y, theta, *_ = self.robot.state()
        if self.robot_theta_tranformer:
            theta = self.robot_theta_tranformer(theta)

        return (x, y, theta, *_)
