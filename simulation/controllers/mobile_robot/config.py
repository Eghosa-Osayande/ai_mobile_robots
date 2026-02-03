def computePath(
    origin: tuple[float, float],
    steps: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    x, y = origin
    path = [(x, y)]

    for dx, dy in steps:
        x += dx
        y += dy
        path.append((x, y))

    return path


directions = [
    (0, 0),
    (1.98, 0),
    (0, -2.53),
    (2, 0),
    (0, -1.27),
    (-5.8, 0),
    (-5.8, 0),
    # (-5.23, 0),
]


nodes = computePath(
    (0, 0),
    directions,
)

nodes = nodes[::1]

isSerial = False
isSerial = True
wheelRadius = 0.095
wheelSeperation = 0.32125

# webot config
leftMotorDeviceName = "left wheel"
rightMotorDeviceName = "right wheel"

# serial config
port = "/dev/cu.usbserial-10"
