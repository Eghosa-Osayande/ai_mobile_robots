class TrackingAgent:
    def __init__(
        self,
        left_margin=0.4,
        right_margin=0.6,
        dt=0.01,
        max_forward_v=1,
        max_turn_v=1,
    ):
        self.dt = dt
        self.left_margin = left_margin
        self.right_margin = right_margin
        self.max_forward_v = max_forward_v
        self.max_turn_v = max_turn_v

    def _create_track_action(
        self,
        *,
        approach_v,
        turn_v,
        prev_action,
    ):

        stall = 0

        if prev_action is not None and (prev_action[:2]) != (approach_v, turn_v):
            stall = 1
            stall = 0
            # print("stall")

        vels = (0, 0)

        if approach_v:
            vr = self.max_forward_v * approach_v
            vl = vr
            vels = (vr, vl)
        elif turn_v:
            vr = self.max_turn_v * turn_v
            vl = -vr
            vels = (vr, vl)

        return (
            vels,
            stall,
            self.dt,
        )

    def act(self, obs):
        pos, frame, detected_color, image_size, prev_action, *_ = obs

        if detected_color and image_size:
            (x, y, w, h, cx, cy, poi) = detected_color

            img_w, img_h = image_size

            offset_frac_x = poi[0] / img_w

            if offset_frac_x < self.left_margin:
                return self._create_track_action(
                    approach_v=0,
                    turn_v=-1,
                    prev_action=prev_action,
                )

            if offset_frac_x > self.right_margin:
                return self._create_track_action(
                    approach_v=0,
                    turn_v=1,
                    prev_action=prev_action,
                )

            if offset_frac_x <= self.right_margin and offset_frac_x >= self.left_margin:
                return self._create_track_action(
                    approach_v=1,
                    turn_v=0,
                    prev_action=prev_action,
                )

        return self._create_track_action(
            approach_v=0,
            turn_v=0,
            prev_action=prev_action,
        )
