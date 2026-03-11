import math

def corridor_distances_lfr(
    readings,
    robot_diameter,
    lookahead,
    margin=0.0,
    front_width_ratio=0.5,
):
    half_width = robot_diameter / 2.0 + margin
    front_half_width = front_width_ratio * half_width

    right_min = None
    front_min = None
    left_min = None

    for theta_deg, r in readings:
        if r is None:
            continue
        if not math.isfinite(r) or r <= 0.0:
            continue

        theta = math.radians(theta_deg)
        x = r * math.cos(theta)
        y = r * math.sin(theta)

        if not (0.0 < x <= lookahead and abs(y) <= half_width):
            continue

        if abs(y) <= front_half_width:
            if front_min is None or x < front_min:
                front_min = x
        elif y > 0.0:
            if right_min is None or x < right_min:
                right_min = x
        else:
            if left_min is None or x < left_min:
                left_min = x

    def normalize(d):
        if d is None:
            return 1.0
        return min(d / lookahead, 1.0)

    return normalize(left_min), normalize(front_min), normalize(right_min)

