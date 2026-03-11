import numpy as np


class AvoidanceBangBangAgent:
    def __init__(
        self,
        safe_distance_min,
        velocity_max,
        forward_seq,
        dt=0.01,
    ):
        self.safe_distance_min = safe_distance_min
        self.velocity_max = velocity_max
        self.forward_seq = forward_seq
        self.dt = dt

    def act(self, obs):
        pos, scans, prev_wheel_vels, fwd_seq = obs
       
        blockage = tuple((int(s < self.safe_distance_min) for s in scans))

        turn_left = (1, -1)
        turn_right = (-1, 1)
        do_nothing = (0, 0)


        wheel_vels = {
            (0, 0, 0): do_nothing,
            (0, 1, 0): turn_right,
            (0, 0, 1): turn_left,
            (0, 1, 1): turn_left,
            (1, 0, 0): turn_right,
            (1, 1, 0): turn_right,
            (1, 0, 1): turn_right,
            (1, 1, 1): turn_right,
        }.get(blockage, do_nothing)

        if wheel_vels != do_nothing:
            fwd_seq = self.forward_seq
        

        if wheel_vels == do_nothing and fwd_seq :
            wheel_vels = (1, 1)

        return (
            wheel_vels,
            self.velocity_max,
            self.dt,
            fwd_seq,
        )
