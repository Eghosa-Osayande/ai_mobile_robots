class AvoidAndTrackAgent:
    def __init__(
        self,
        left_margin=0.4,
        right_margin=0.6,
        dt=0.01,
    ):
        self.dt = dt
        self.left_margin = left_margin
        self.right_margin = right_margin

    def act(self, obs):
        avoid_obs, track_obs = obs

        avoid_action = self._act_avoid(avoid_obs)
        if avoid_action is not None:
            return [avoid_action, None]

        track_action = self._act_track(track_obs)

        return [None, track_action]

    def _act_avoid(self, obs):
        pos, blockage, prev_action, *_ = obs

        block_seq = tuple((1 if blocked else 0 for blocked in blockage))

        turn_left = "turn_left"
        turn_right = "turn_right"
        turn_right = turn_left
        do_nothing = None

        next_action = {
            (0, 0, 0): do_nothing,
            (0, 1, 0): turn_right,
            (0, 0, 1): turn_left,
            (0, 1, 1): turn_left,
            (1, 0, 0): turn_right,
            (1, 1, 0): turn_right,
            (1, 0, 1): turn_right,
            (1, 1, 1): turn_right,
        }.get(block_seq)

        if (
            prev_action is not None
            and prev_action != "move_forward"
            and next_action is None
        ):

            return "move_forward"

        if prev_action == "move_forward" and next_action is None:

            return next_action

        return next_action

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

        return (
            approach_v,
            turn_v,
            stall,
            self.dt,
        )

    def _act_track(self, obs):
        pos, frame, detected_color, image_size, prev_action, *_ = obs

        if detected_color and image_size:
            (x, y, w, h, cx, cy, poi) = detected_color

            img_w, img_h = image_size

            offset_frac_x = poi[0] / img_w

            if offset_frac_x < self.left_margin:
                return self._create_track_action(
                    approach_v=0,
                    turn_v=1,
                    prev_action=prev_action,
                )

            if offset_frac_x > self.right_margin:
                return self._create_track_action(
                    approach_v=0,
                    turn_v=-1,
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
