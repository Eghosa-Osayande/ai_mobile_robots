import helpers
import os


directions = [
    (0, 0),
    (1.98, 0),
    (0, -2.53),
    (2, 0),
    (0, -1.27),
    (-5.23, 0),
    (-5.23, 0),
]


nodes = helpers.computePath(
    (0, 0),
    directions,
)

nodes = nodes[::1]

isSerial = os.environ.get("IS_WEBOTS") != "true"
wheelRadius = 0.095
wheelSeperation = 0.32125

# webot config
leftMotorDeviceName = "left wheel"
rightMotorDeviceName = "right wheel"

# serial config
port = "/dev/cu.usbserial-10"
