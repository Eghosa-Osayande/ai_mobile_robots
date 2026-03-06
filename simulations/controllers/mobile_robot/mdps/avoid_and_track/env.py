from __future__ import annotations
import time
import numpy as np

from two_wheel_robots.two_wheel_robot_base import TwoWheelRobotBase
from two_wheel_robots.integrations.camera_client import CamClient

import cv2 as cv2
import sys

H_TOL = 12
S_MIN = 60
V_MIN = 40
MORPH_KERNEL = (7, 7)


def hex_to_bgr(hex_code: str) -> tuple[int, int, int]:
    s = hex_code.strip().lstrip("#")
    if len(s) != 6:
        raise ValueError(f"Expected 6-digit hex like #RRGGBB, got: {hex_code!r}")
    r = int(s[0:2], 16)
    g = int(s[2:4], 16)
    b = int(s[4:6], 16)
    return (b, g, r)


def bgr_to_hsv(bgr: tuple[int, int, int]) -> tuple[int, int, int]:
    assert cv2 is not None
    px = np.uint8([[list(bgr)]])
    hsv = cv2.cvtColor(px, cv2.COLOR_BGR2HSV)[0, 0]
    return int(hsv[0]), int(hsv[1]), int(hsv[2])


def hue_ranges(h: int, h_tol: int) -> list[tuple[np.ndarray, np.ndarray]]:
    lo = h - h_tol
    hi = h + h_tol

    if lo < 0:
        return [
            (
                np.array([0, 0, 0], dtype=np.uint8),
                np.array([hi, 255, 255], dtype=np.uint8),
            ),
            (
                np.array([179 + lo, 0, 0], dtype=np.uint8),
                np.array([179, 255, 255], dtype=np.uint8),
            ),
        ]
    if hi > 179:
        return [
            (
                np.array([0, 0, 0], dtype=np.uint8),
                np.array([hi - 179, 255, 255], dtype=np.uint8),
            ),
            (
                np.array([lo, 0, 0], dtype=np.uint8),
                np.array([179, 255, 255], dtype=np.uint8),
            ),
        ]

    return [
        (np.array([lo, 0, 0], dtype=np.uint8), np.array([hi, 255, 255], dtype=np.uint8))
    ]


def build_colour_mask(
    hsv_frame: np.ndarray,
    target_hex: str,
    h_tol: int,
    s_min: int,
    v_min: int,
) -> np.ndarray:
    assert cv2 is not None
    bgr = hex_to_bgr(target_hex)
    h, _, _ = bgr_to_hsv(bgr)

    ranges = hue_ranges(h, h_tol)

    mask = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
    for lower, upper in ranges:
        lower = lower.copy()
        upper = upper.copy()

        lower[1] = np.uint8(s_min)
        lower[2] = np.uint8(v_min)
        upper[1] = np.uint8(255)
        upper[2] = np.uint8(255)

        mask = cv2.bitwise_or(mask, cv2.inRange(hsv_frame, lower, upper))

    return mask


class AvoidAndTrackingEnv:

    def __init__(
        self,
        robot: TwoWheelRobotBase,
        cam_client: CamClient,
        # avoidance
        front_idxs: list[float],
        left_idxs: list[float],
        right_idxs: list[float],
        avoid_dist_thres: float,
        avoid_velocity: float,
        # Track
        approach_v=1,
        turn_v=1,
        target_color_hex=None,
        area_constraint=None,
        cascade_classfier_path=cv2.data.haarcascades
        + "haarcascade_eye_tree_eyeglasses.xml",
        use_cascade=False,
    ):
        super().__init__()
        self.robot = robot
        self.cam_client = cam_client

        # Avoid
        self.left_idxs = left_idxs
        self.right_idxs = right_idxs
        self.front_idxs = front_idxs
        self.avoid_dist_thres = avoid_dist_thres
        self.avoid_velocity = avoid_velocity

        # Track

        self.max_velocity = approach_v
        self.area_constraint = area_constraint
        self.approach_v = approach_v
        self.turn_v = turn_v

        self.target_color_hex = target_color_hex

        self.cascade = None
        if cascade_classfier_path:
            cascade = cv2.CascadeClassifier(cascade_classfier_path)
            if not cascade.empty():
                self.cascade = cascade

        # detect
        self.frame = None
        self.detected_color = None
        self.image_size = None
        self.prev_track_action = None
        self.poi = None
        self.use_cascade = use_cascade

        # avoid
        self.prev_avoid_action = None

        # camera
        self.cam_client = cam_client
        self.cam_client.start()
        print("video capture started")

    def _avoid_obs(self, robot_state):
        x, y, th, *scans = robot_state

        pos = (x, y, th)
        blockage_l_front_r = (False, False, False)

        if scans is not None:
            left_d = np.min(np.array(scans)[self.left_idxs])
            right_d = np.min(np.array(scans)[self.right_idxs])
            front_d = np.min(np.array(scans)[self.front_idxs])

            is_left_blocked = left_d <= self.avoid_dist_thres

            is_front_blocked = front_d <= self.avoid_dist_thres

            is_right_blocked = right_d <= self.avoid_dist_thres

            blockage_l_front_r = (
                is_left_blocked,
                is_front_blocked,
                is_right_blocked,
            )

        return [
            pos,
            blockage_l_front_r,
            self.prev_avoid_action,
        ]

    def _track_obs(self, robot_state):
        x, y, th, *scans = robot_state

        return [
            (x, y, th),
            None if self.frame is None else self.frame.copy(),
            self.detected_color,
            self.image_size,
            self.prev_track_action,
            self.poi,
        ]

    def _obs(self):
        robot_state = self.robot.state()

        a_obs = self._avoid_obs(robot_state)

        t_obs = self._track_obs(robot_state)

        return [
            a_obs,
            t_obs,
        ]

    def close(self):
        try:
            self.cam_client.release()
        except Exception:
            pass

        cv2.destroyAllWindows()

    def reset(
        self,
        *args,
        **kwargs,
    ):
        self.robot.reset()

        # track
        self.frame = None
        self.image_size = None
        self.detected_color = None
        self.prev_track_action = None
        self.poi = None

        # avoid
        self.prev_avoid_action = None
        return self._obs(), {}

    def step(self, action):

        (
            avoid_action,
            track_action,
        ) = action

        if avoid_action is not None:
            self._avoid_step(avoid_action)

        if track_action is not None:
            self._track_step(track_action)

        self.frame = None
        self.image_size = None
        self.detected_color = None, None

        ok, frame = self.cam_client.read()

        if ok:
            frame = frame[:, ::-1]
            self.frame = frame
            h_img, w_img = frame.shape[:2]
            self.image_size = (w_img, h_img)

            if self.cascade and self.use_cascade:
                self.detected_color = self._cascade_detect(frame)
            else:
                self.detected_color = self._detect_colour(frame)

        obs = self._obs()
        (pos, *_), (_, frame, detection, image_size, *_) = obs

        terminated = False
        if detection is not None and image_size is not None:
            poi = detection[-1]
            (_, img_h) = image_size
            offset_frac_y = poi[1] / img_h

            # print(offset_frac_y)
            if offset_frac_y < 0.15:
                terminated = True
                self.robot.move_wheels(0, 0)

        info = {
            "time": time.time(),
            "x": pos[0],
            "y": pos[1],
            "th": pos[2],
            "env": "avoid_track",
            # "frame": frame,
            "detection": detection,
        }

        self.render(obs)

        return obs, 0, terminated, False, info

    def _avoid_step(self, a):
        if a is None:
            self.robot.move_wheels(0, 0)
            return

        action = {
            "turn_left": (1, -1, 0.1),
            "turn_right": (-1, 1, 0.1),
            "move_forward": (1, 1, 4),
        }[a]

        self.prev_avoid_action = a
        vr, vl, dt, *_ = action
        velocity = self.avoid_velocity

        if action == "move_forward":
            # move dist meters forward
            dist = 0.5
            dt = dist / velocity

        self.robot.move_wheels(vr * velocity, vl * velocity)
        self.robot.step(dt)

    def _track_step(self, action):

        approach, turn, stall, dt = action

        if stall:
            self.robot.move_wheels(0, 0)
            self.robot.step(stall)

        if approach and not stall:
            v = approach * self.approach_v
            self.robot.move_wheels(v, v)
            self.robot.step(dt)

        if turn and not stall:
            v = turn * self.turn_v
            self.robot.move_wheels(v, -v)
            self.robot.step(dt)

        self.prev_track_action = action

    def render(self, obs=None):
        if obs is None:
            obs = self._obs()
        _, t_obs = obs

        self._render_track(t_obs)

    def _render_track(self, obs):
        pos, frame, detected_color, image_size, prev_action, *_ = obs

        if frame is None:
            return
        if detected_color:
            (x, y, w, h, cx, cy, poi) = detected_color
            cv2.rectangle(
                frame,
                (x, y),
                (x + w, y + h),
                color=(0, 255, 0),
                thickness=2,
            )
            cv2.circle(
                frame,
                (cx, cy),
                4,
                (0, 0, 255),
                -1,
            )

        window = "Tracking"
        cv2.namedWindow(window)
        cv2.imshow(window, frame)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:  # ESC
            import sys

            sys.exit()

    def _cascade_detect(self, frame: cv2.typing.MatLike):
        detected = None

        cascade = self.cascade
        if cascade is not None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.equalizeHist(gray)

            items = cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(30, 30),
            )

            if len(items) > 0:
                # pick largest face by area
                x, y, w, h = max(
                    items,
                    key=lambda r: int(r[2]) * int(r[3]),
                )
                area = float(w * h)

                cx = x + w // 2
                cy = y + h // 2

                area_ok = True

                if self.area_constraint:
                    area_ok = self.area_constraint[0] <= area <= self.area_constraint[0]

                if area_ok:

                    if self.poi is None:
                        self.poi = [cx, cy]
                    else:
                        self.poi[0] = 0.7 * self.poi[0] + 0.3 * cx
                        self.poi[1] = 0.7 * self.poi[1] + 0.3 * cy

                    detected = (
                        x,
                        y,
                        w,
                        h,
                        cx,
                        cy,
                        [*self.poi],
                    )

            return detected

    def _detect_colour(self, frame: cv2.typing.MatLike):
        detected_color = None

        target_hex = self.target_color_hex
        if target_hex:

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            mask = build_colour_mask(hsv, target_hex, H_TOL, S_MIN, V_MIN)

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, MORPH_KERNEL)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            if contours:
                c = max(contours, key=cv2.contourArea)
                area = float(cv2.contourArea(c))

                area_ok = True
                if self.area_constraint:
                    area_ok = self.area_constraint[0] <= area <= self.area_constraint[0]

                if area_ok:
                    (x, y, w, h) = cv2.boundingRect(c)
                    cx = x + w // 2
                    cy = y + h // 2
                    if self.poi is None:
                        self.poi = [cx, cy]
                    else:
                        self.poi[0] = 0.7 * self.poi[0] + 0.3 * cx
                        self.poi[1] = 0.7 * self.poi[1] + 0.3 * cy

                    detected_color = (
                        x,
                        y,
                        w,
                        h,
                        cx,
                        cy,
                        [*self.poi],
                    )

        return detected_color
