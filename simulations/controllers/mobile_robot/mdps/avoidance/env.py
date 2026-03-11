from __future__ import annotations


from two_wheel_robots.two_wheel_robot_base import TwoWheelRobotBase


class AvoidanceEnv:

    def __init__(
        self,
        robot: TwoWheelRobotBase,
    ):
        super().__init__()
        self.robot = robot
        self.prev_wheel_velocities = None
        self.fwd_seq = 0

    def _obs(self):
        robot_state = self.robot.state()
        x, y, th, scans, *_ = robot_state

        pos = (x, y, th)

        return [
            pos,
            scans,
            self.prev_wheel_velocities,
            self.fwd_seq,
        ]

    def close(self): ...

    def reset(
        self,
        *args,
        **kwargs,
    ):
        self.prev_wheel_velocities = None
        self.fwd_seq = 0
        return self._obs(), {}

    def step(self, action):
        (vr, vl), v_max, dt, fwd_seq = action

        terminated = False

        if (vr, vl) != (0, 0):

            print(self.prev_wheel_velocities, (vr, vl))

            if self.prev_wheel_velocities != (vr, vl):
                print("Changed vels")
                self.robot.move_wheels(
                    vr * v_max,
                    vl * v_max,
                )
                self.robot.step(dt)
                self.prev_wheel_velocities = (vr, vl)

            if (vr, vl) == (1, 1) and fwd_seq > 0:
                fwd_seq -= 1
                self.robot.step(1)

            terminated = fwd_seq == 0 and (vr, vl) == (1, 1)

            self.fwd_seq = fwd_seq

        else:
            if self.prev_wheel_velocities != (0, 0):
                self.robot.move_wheels(0, 0)
            self.robot.step(dt)
            self.prev_wheel_velocities = (0, 0)
            terminated = True

        obs = self._obs()

        info = {}

        return obs, 0, terminated, False, info

    def render(self, obs=None): ...
