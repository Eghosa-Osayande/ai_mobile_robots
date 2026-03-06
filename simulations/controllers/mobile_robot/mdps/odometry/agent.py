import math


class OdometryAgent:
    def __init__(
        self,
        dt=0.01,
    ):
        self.dt = dt

    def create_action(
        self,
        *,
        v_r,
        v_l,
    ):
        return (
            v_r,
            v_l,
            self.dt,
        )

    def act(self, obs):
        pos, scans, goal, dist_err, heading_err, turning_complete = obs

        if not turning_complete:
            v = -1 if heading_err > 0 else 1
            return self.create_action(
                v_r=-v,
                v_l=v,
            )

        return self.create_action(
            v_r=1,
            v_l=1,
        )
