import math
import numpy as np
from stable_baselines3 import SAC


class OdometrySACAgent:
    def __init__(
        self,
        model_path,
        dt=0.05,
        dist_err_thres=0.05,
        heading_err_thres=5.0,
        max_forward_v=0.3,
        max_turn_v=0.2,
        obs_norm_radius=6,
    ):
        self.dt = dt
        self.dist_err_thres = dist_err_thres
        self.heading_err_thres = heading_err_thres
        self.max_forward_v = max_forward_v
        self.max_turn_v = max_turn_v
        self.obs_norm_radius = obs_norm_radius
        self.model = SAC.load(model_path)

    def _obs(
        self,
        x,
        y,
        theta,
        goal,
    ):

        dx = float(goal[0] - x)
        dy = float(goal[1] - y)

        c = math.cos(theta)
        s = math.sin(theta)
        gx_r = c * dx + s * dy
        gy_r = -s * dx + c * dy

        r = max(self.obs_norm_radius, 1e-6)
        gx_n = float(np.clip(gx_r / r, -1.0, 1.0))
        gy_n = float(np.clip(gy_r / r, -1.0, 1.0))

        return np.array([gx_n, gy_n, float(c), float(s)], dtype=np.float32)

    def act(self, obs):

        pos, scans, goal, dist_err, heading_err, *_ = obs

        obs2 = self._obs(
            pos[0],
            pos[1],
            math.radians(pos[2]),
            goal,
        )

        (vr, vl), _ = self.model.predict(obs2)

        vr = (vr / 0.6) * self.max_forward_v
        vl = (vl / 0.6) * self.max_forward_v

        if dist_err <= self.dist_err_thres:
            return (0, 0, 0)

        return (vr, vl, self.dt)
