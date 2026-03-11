class PID:
    def __init__(
        self,
        kp: float,
        ki: float = 0.0,
        kd: float = 0.0,
        output_limit: float | None = None,
        integral_limit: float | None = None,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit

        self.integral = 0.0
        self.prev_error = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0

        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = max(
                -self.integral_limit,
                min(self.integral, self.integral_limit),
            )

        derivative = 0.0
        if self.prev_error is not None:
            derivative = (error - self.prev_error) / dt

        self.prev_error = error

        u = self.kp * error + self.ki * self.integral + self.kd * derivative

        if self.output_limit is not None:
            u = max(-self.output_limit, min(u, self.output_limit))

        return u


class OdometryPIDAgent:
    def __init__(
        self,
        dt=0.05,
        dist_err_thres=0.05,
        heading_err_thres=5.0,
        max_forward_v=0.3,
        max_turn_v=0.2,
        #
        heading_pid: PID | None = None,
        dist_pid: PID | None = None,
        turn_only_angle_deg=25.0,
    ):
        self.dt = dt
        self.dist_err_thres = dist_err_thres
        self.heading_err_thres = heading_err_thres
        self.max_forward_v = max_forward_v
        self.max_turn_v = max_turn_v
        self.turn_only_angle_deg = turn_only_angle_deg

        self.heading_pid = heading_pid or PID(
            kp=0.02,
            ki=0.0,
            kd=0.002,
            output_limit=max_turn_v,
            integral_limit=100.0,
        )

        self.dist_pid = dist_pid or PID(
            kp=0.8,
            ki=0.0,
            kd=0.05,
            output_limit=max_forward_v,
            integral_limit=2.0,
        )

    def reset(self):
        self.heading_pid.reset()
        self.dist_pid.reset()

    def act(self, obs):
        pos, scans, goal, dist_err, heading_err, *_ = obs

        if dist_err <= self.dist_err_thres:
            self.reset()
            return (0, 0, 0)

        turn = self.heading_pid.update(heading_err, self.dt)

        # reduce or disable forward motion when badly misaligned
        if abs(heading_err) > self.turn_only_angle_deg:
            forward = 0.0
        else:
            forward = self.dist_pid.update(dist_err, self.dt)
            scale = max(0.0, 1.0 - abs(heading_err) / self.turn_only_angle_deg)
            forward *= scale

        vr = forward + turn
        vl = forward - turn

        vr = max(-self.max_forward_v, min(vr, self.max_forward_v))
        vl = max(-self.max_forward_v, min(vl, self.max_forward_v))

        return (vr, vl, self.dt)
