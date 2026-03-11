class OdometryBangBangAgent:
    def __init__(
        self,
        max_forward_v,
        max_turn_v,
        dist_err_thres=0,
        heading_err_thres=0,
        dt=0.01,
    ):
        self.max_forward_v = max_forward_v
        self.max_turn_v = max_turn_v
        self.dist_err_thres = dist_err_thres
        self.heading_err_thres = heading_err_thres
        self.dt = dt

    def act(self, obs):
        pos, scans, goal, dist_err, heading_err, *_ = obs

        if abs(heading_err) > self.heading_err_thres:
            vr = 1 if heading_err > 0 else -1
            vl = -vr
            return (
                vr * self.max_turn_v,
                vl * self.max_turn_v,
                self.dt,
            )

        if abs(dist_err) <= self.dist_err_thres:
            return (0, 0, 0)

        return (
            1 * self.max_forward_v,
            1 * self.max_forward_v,
            self.dt,
        )
