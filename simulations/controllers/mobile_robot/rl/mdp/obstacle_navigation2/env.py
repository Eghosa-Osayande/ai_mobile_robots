from __future__ import annotations

import math
from typing import Any, Callable, Dict, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import gymnasium as gym
from gymnasium import spaces


def corridor_distances_lfr(
    readings,
    robot_diameter,
    lookahead,
    margin=0.0,
    front_width_ratio=0.5,
):
    half_width = robot_diameter / 2.0 + margin
    front_half_width = front_width_ratio * half_width

    left_min = None
    front_min = None
    right_min = None

    for theta_deg, r in readings:
        if r is None:
            continue
        if not math.isfinite(r) or r <= 0.0:
            continue

        theta = math.radians(theta_deg)
        x = r * math.cos(theta)
        y = r * math.sin(theta)

        if not (0.0 < x <= lookahead and abs(y) <= half_width):
            continue

        if abs(y) <= front_half_width:
            if front_min is None or x < front_min:
                front_min = x
        elif y > 0.0:
            if left_min is None or x < left_min:
                left_min = x
        else:
            if right_min is None or x < right_min:
                right_min = x

    def normalize(d):
        if d is None:
            return 1.0
        return min(d / lookahead, 1.0)

    return normalize(left_min), normalize(front_min), normalize(right_min)


class TwoWheelRobotWithObstacles2:
    def __init__(
        self,
        wheel_distance,
        lookahead,
        robot_diameter,
        lidar_fov_deg=180.0,
        lidar_num_beams=181,
        corridor_margin=0.0,
        front_width_ratio=0.5,
    ):
        assert wheel_distance > 0
        assert lookahead > 0
        assert robot_diameter > 0

        self.wheel_distance = float(wheel_distance)
        self.lookahead = float(lookahead)
        self.robot_diameter = float(robot_diameter)
        self.robot_radius = 0.5 * self.robot_diameter

        self.lidar_fov_deg = float(lidar_fov_deg)
        self.lidar_num_beams = int(lidar_num_beams)
        self.corridor_margin = float(corridor_margin)
        self.front_width_ratio = float(front_width_ratio)

        self.obstacles = []
        self.reset()

    def reset(
        self,
        pos: tuple[float, float] = None,
        obstacles=None,
        theta=None,
        **kw,
    ):
        self.theta = 0.0 if theta is None else float(theta)
        self.x = 0.0
        self.y = 0.0
        self.v = 0.0
        self.omega = 0.0
        self.obstacles = obstacles if obstacles is not None else self.obstacles

        if pos is not None:
            self.x = float(pos[0])
            self.y = float(pos[1])

    def state(self):
        return (
            self.x,
            self.y,
            self.theta,
            list(self._read_corridor()),
        )

    def move_wheels(self, v_right, v_left):
        self.v = 0.5 * (v_right + v_left)
        self.omega = (v_right - v_left) / self.wheel_distance

    def step(self, dt):
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt

    @staticmethod
    def _ray_circle_distance(x, y, dx, dy, ox, oy, r):
        fx = x - ox
        fy = y - oy

        b = 2.0 * (fx * dx + fy * dy)
        c = (fx * fx + fy * fy) - (r * r)

        disc = b * b - 4.0 * c
        if disc < 0.0:
            return None

        sqrt_disc = math.sqrt(disc)
        t1 = (-b - sqrt_disc) * 0.5
        t2 = (-b + sqrt_disc) * 0.5

        hits = [t for t in (t1, t2) if t >= 0.0]
        if not hits:
            return None
        return min(hits)

    def _ray_distance(self, x, y, angle):
        dx = math.cos(angle)
        dy = math.sin(angle)

        best = self.lookahead

        for ox, oy, r in self.obstacles:
            t = self._ray_circle_distance(
                x,
                y,
                dx,
                dy,
                ox,
                oy,
                r + self.robot_radius,
            )
            if t is not None and t < best:
                best = t

        return best

    def _lidar_readings(self):
        if self.lidar_num_beams <= 1:
            angles_deg = [0.0]
        else:
            angles_deg = np.linspace(
                -0.5 * self.lidar_fov_deg,
                0.5 * self.lidar_fov_deg,
                self.lidar_num_beams,
            )

        readings = []
        for rel_deg in angles_deg:
            dist = self._ray_distance(
                self.x,
                self.y,
                self.theta + math.radians(float(rel_deg)),
            )
            readings.append((float(rel_deg), float(dist)))

        return readings

    def _read_corridor(self):
        readings = self._lidar_readings()
        return corridor_distances_lfr(
            readings=readings,
            robot_diameter=self.robot_diameter,
            lookahead=self.lookahead,
            margin=self.corridor_margin,
            front_width_ratio=self.front_width_ratio,
        )


def point_in_any_obstacle(x, y, obstacles, margin=0.0):
    for ox, oy, r in obstacles:
        if np.hypot(x - ox, y - oy) <= (r + margin):
            return True
    return False


class TwoWheelObstacleNavigationEnv2(gym.Env):
    metadata = {"render_modes": []}

    def __init__(
        self,
        *,
        robot_factory: Callable[[], Any],
        wheel_speed_limit: float = 0.02 * 10,
        dt: float = 0.1,
        max_steps: int = 250,
        goal_radius: float = 0.10,
        spawn_xy: Tuple[float, float] = (0.0, 0.0),
        spawn_theta: float = 0.0,
        spawn_sampler: Optional[
            Callable[[np.random.Generator], Tuple[Tuple[float, float], float]]
        ] = None,
        goal_xy: Tuple[float, float] = (3.0, 0.0),
        goal_xy_sampler: Optional[
            Callable[[np.random.Generator], Tuple[float, float]]
        ] = None,
        obstacles=None,
        obstacle_sampler: Optional[Callable[[np.random.Generator], list]] = None,
        arena_radius: Optional[float] = 6.0,
        obs_norm_radius: float = 6.0,
        effort_penalty: float = 0.01,
        spin_penalty: float = 0.002,
        stall_penalty: float = 0.05,
        stall_window: int = 12,
        stall_tol: float = 1e-3,
        success_bonus: float = 10.0,
        oob_penalty: float = 2.0,
        obstacle_penalty: float = 0.2,
        collision_penalty: float = 5.0,
        safe_distance: float = 0.25,
        collision_margin: float = 0.0,
        render_enabled: bool = False,
        allow_reverse: bool = False,
        seed: Optional[int] = None,
        robot_theta_tranformer=None,
        render_path="obstacle_navigation.png",
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

        self.obstacles = list(obstacles) if obstacles is not None else []
        self.obstacle_sampler = obstacle_sampler

        self.arena_radius = None if arena_radius is None else float(arena_radius)
        self.obs_norm_radius = float(obs_norm_radius)

        self.effort_penalty = float(effort_penalty)
        self.spin_penalty = float(spin_penalty)
        self.stall_penalty = float(stall_penalty)
        self.stall_window = int(stall_window)
        self.stall_tol = float(stall_tol)
        self.success_bonus = float(success_bonus)
        self.oob_penalty = float(oob_penalty)

        self.obstacle_penalty = float(obstacle_penalty)
        self.collision_penalty = float(collision_penalty)
        self.safe_distance = float(safe_distance)
        self.collision_margin = float(collision_margin)

        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32,
        )

        probe_robot = self.robot_factory()
        probe_state = probe_robot.state()
        self.num_sonars = max(0, len(probe_state[3]) if len(probe_state) > 3 else 0)
        self.sonar_max_range = 1.0

        obs_dim = 4 + self.num_sonars
        self.observation_space = spaces.Box(
            low=np.full((obs_dim,), -1.0, dtype=np.float32),
            high=np.full((obs_dim,), 1.0, dtype=np.float32),
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
        self._ep_success_flag = 0
        self._ep_failure_flag = 0

    def _map_action_to_wheels(self, action: np.ndarray) -> Tuple[float, float]:
        a = np.asarray(action, dtype=np.float32)
        a = np.clip(a, -1.0, 1.0)

        if self.allow_reverse:
            v_r = float(a[0]) * self.wheel_speed_limit
            v_l = float(a[1]) * self.wheel_speed_limit
        else:
            v_r = 0.5 * (float(a[0]) + 1.0) * self.wheel_speed_limit
            v_l = 0.5 * (float(a[1]) + 1.0) * self.wheel_speed_limit

        return v_r, v_l

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

        if self.obstacle_sampler is not None:
            self.obstacles = list(self.obstacle_sampler(self.np_random))

        if robot_pos:
            self.robot.reset(
                pos=robot_pos[:2],
                theta=robot_pos[2],
                obstacles=self.obstacles,
            )

        if reset_robot and not robot_pos:
            if self.spawn_sampler is not None:
                (sx, sy), stheta = self.spawn_sampler(self.np_random)
            else:
                (sx, sy), stheta = self.spawn_xy, self.spawn_theta

            self.robot.reset(
                pos=(float(sx), float(sy)),
                theta=float(stheta),
                obstacles=self.obstacles,
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
        self._ep_success_flag = 0
        self._ep_failure_flag = 0

        obs = self._obs()
        info = {
            "goal": self.goal.copy(),
            "obstacles": self.obstacles,
        }
        return obs, info

    def step(self, action):
        self.steps += 1

        v_r, v_l = self._map_action_to_wheels(action)

        self.robot.move_wheels(v_r, v_l)
        self.robot.step(self.dt)

        x, y, theta, sonars, *_ = self._get_robot_state()

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

        min_sonar = float(min(sonars)) if len(sonars) > 0 else 1.0
        collision = point_in_any_obstacle(
            x, y, self.obstacles, margin=self.collision_margin
        )

        reward = float(self.prev_dist - d)
        reward -= self.effort_penalty * (abs(v_r) + abs(v_l))
        reward -= self.spin_penalty * abs(v_r - v_l)

        if min_sonar < self.safe_distance:
            reward -= (
                self.obstacle_penalty
                * (self.safe_distance - min_sonar)
                / max(self.safe_distance, 1e-6)
            )

        self.dist_hist.append(d)
        if len(self.dist_hist) > self.stall_window:
            self.dist_hist.pop(0)
            if (self.dist_hist[0] - self.dist_hist[-1]) < self.stall_tol:
                reward -= self.stall_penalty

        if success:
            reward += self.success_bonus

        terminated = bool(success or collision)
        truncated = self.steps >= self.max_steps

        if collision:
            reward -= self.collision_penalty

        if self.arena_radius is not None:
            if float(np.hypot(x, y)) > self.arena_radius:
                truncated = True
                reward -= self.oob_penalty

        self.prev_dist = d
        obs = self._obs()

        info = {
            "dist_to_goal": d,
            "success": bool(success),
            "collision": bool(collision),
            "min_sonar": min_sonar,
            "heading_error": heading_err,
            "v_r": v_r,
            "v_l": v_l,
            "left_clearance": float(sonars[0]) if len(sonars) > 0 else 1.0,
            "front_clearance": float(sonars[1]) if len(sonars) > 1 else 1.0,
            "right_clearance": float(sonars[2]) if len(sonars) > 2 else 1.0,
        }

        if terminated or truncated:
            sx, sy = self._ep_start_xy
            straight = float(math.hypot(self.goal[0] - sx, self.goal[1] - sy))
            path = float(self._ep_path_len)
            eff = float(straight / max(path, 1e-9))

            ep_len = int(self.steps)
            failure = not success

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
                    "ep_success": float(success),
                    "ep_failure": float(failure),
                }
            )

        self._trajectory.append((x, y))
        self.render()

        return obs, reward, terminated, truncated, info

    def _dist_to_goal(self) -> float:
        x, y, *_ = self._get_robot_state()
        return float(np.hypot(self.goal[0] - x, self.goal[1] - y))

    def _obs(self) -> np.ndarray:
        x, y, theta, sonars, *_ = self._get_robot_state()
        dx = float(self.goal[0] - x)
        dy = float(self.goal[1] - y)

        c = math.cos(theta)
        s = math.sin(theta)
        gx_r = c * dx + s * dy
        gy_r = -s * dx + c * dy

        r = max(self.obs_norm_radius, 1e-6)
        gx_n = float(np.clip(gx_r / r, -1.0, 1.0))
        gy_n = float(np.clip(gy_r / r, -1.0, 1.0))

        sonar_n = [float(np.clip((2.0 * s) - 1.0, -1.0, 1.0)) for s in sonars]

        return np.array(
            [
                gx_n,
                gy_n,
                float(c),
                float(s),
                *sonar_n,
            ],
            dtype=np.float32,
        )

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

        for ox, oy, r in self.obstacles:
            plt.gca().add_patch(plt.Circle((ox, oy), r, fill=False))

        plt.scatter(self.goal[0], self.goal[1])
        plt.gca().add_patch(
            plt.Circle((self.goal[0], self.goal[1]), self.goal_radius, fill=False)
        )

        if self.arena_radius is not None:
            plt.gca().add_patch(plt.Circle((0, 0), self.arena_radius, fill=False))

        plt.gca().set_aspect("equal", adjustable="box")
        plt.xlim(-self.obs_norm_radius, self.obs_norm_radius)
        plt.ylim(-self.obs_norm_radius, self.obs_norm_radius)
        plt.savefig(self._render_path)

    def _get_robot_state(self):
        x, y, theta, prox_data, *rest = self.robot.state()
        if self.robot_theta_tranformer:
            theta = self.robot_theta_tranformer(theta)
        return (x, y, theta, prox_data)
