from __future__ import annotations
import math
import time

from two_wheel_robots.two_wheel_robot_base import TwoWheelRobotBase
import queue
import threading
import matplotlib
import matplotlib.pyplot as plt


matplotlib.use("Agg")


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
    ):
        super().__init__()
        self.robot = robot
        self.goal = goal
        self.last_cmd = None
        self._traj = []
        self._render_q = queue.Queue(maxsize=1)
        self._render_stop = threading.Event()

        self._render_th = threading.Thread(
            target=self._render_worker,
            daemon=True,
        )
        self._render_th.start()

    def _obs(self):
        x1, y1, th, prox, scan, *_ = self.robot.state()

        x2, y2 = self.goal[:2]

        # Calculate Euclidean Distance
        dx = x2 - x1
        dy = y2 - y1

        print(
            (x2, y2),
            (x1, y1, th),
        )

        dist_err = math.sqrt(dx**2 + dy**2)

        heading_err = heading_error((x1, y1, th), (x2, y2))

        return [
            (x1, y1, th),
            prox,
            self.goal,
            dist_err,
            heading_err,
            self.last_cmd,
            scan,
        ]

    def close(self): ...

    def reset(
        self,
        goal=None,
    ):
        self.goal = self.goal if goal is None else goal
        self.last_cmd = None
        return self._obs(), {}

    def step(self, action):
        print(action)
        turn, move = action
        turn_vr, turn_vl, turn_dt, turn_start = turn
        move_vr, move_vl, move_dt, move_start = move

        vr, vl = 0, 0

        unit_step = 0.01

        if turn_dt != 0:
            if turn_start is None:
                turn_start = time.time()

            vr, vl = turn_vr, turn_vl
            next = (0, 0, 0, None)
            time_elapsed = time.time() - turn_start

            if time_elapsed < turn_dt:
                next = (turn_vr, turn_vl, turn_dt, turn_start)

                time_remaining = abs(turn_dt - time_elapsed)

                if time_remaining <= unit_step:
                    unit_step = time_remaining

            self.last_cmd = [next, move]

        elif move_dt != 0:
            if move_start is None:
                move_start = time.time()

            vr, vl = move_vr, move_vl
            next = (0, 0, 0, None)
            time_elapsed = time.time() - move_start

            if time_elapsed < move_dt:
                next = (move_vr, move_vl, move_dt, move_start)

                time_remaining = abs(move_dt - time_elapsed)

                if time_remaining <= unit_step:
                    unit_step = time_remaining

            self.last_cmd = [turn, next]

        self.robot.move_wheels(
            vr,
            vl,
        )

        self.robot.step(unit_step)

        obs = self._obs()

        pos, _, goal, dist_err, heading_err, _, scan, *_ = obs

        terminated = action[0] == (0, 0, 0, None) and action[1] == (0, 0, 0, None)

        self.render(obs=obs)

        info = {
            "time": time.time(),
            "x": pos[0],
            "y": pos[1],
            "th": pos[2],
            "gx": goal[0],
            "gy": goal[1],
            "is_at_goal": terminated,
            "vr": vr,
            "vl": vl,
            "lidar": scan,
            "env": "odom",
        }

        return obs, 0, terminated, False, info
    def _render_worker(self):

        while not self._render_stop.is_set():
            try:
                data = self._render_q.get(timeout=0.5)
            except queue.Empty:
                continue

            traj, goal, filename = data

            xs = [p[0] for p in traj]
            ys = [p[1] for p in traj]
            gx, gy = goal

            fig, ax = plt.subplots()

            ax.plot(xs, ys, label="trajectory")
            ax.scatter(xs[-1], ys[-1], label="robot")
            ax.scatter(gx, gy, marker="x", label="goal")

            ax.set_aspect("equal", adjustable="box")
            ax.grid(True)
            ax.legend()

            fig.savefig(filename)
            plt.close(fig)

    def render(self, obs=None):
        if obs is None:
            obs = self._obs()

        pos, _, goal, *_ = obs
        x, y, _ = pos
        gx, gy = goal[:2]

        self._traj.append((x, y))

        try:
            while True:
                self._render_q.get_nowait()
        except queue.Empty:
            pass

        try:
            filename="traj.png"
            self._render_q.put_nowait((list(self._traj), (gx, gy), filename))
        except queue.Full:
            pass