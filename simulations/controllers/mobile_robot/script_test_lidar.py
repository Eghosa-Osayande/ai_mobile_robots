import time

from simulations.controllers.mobile_robot.two_wheel_robots.integrations.lidar_link import (
    LidarLink,
)


l = LidarLink(
    port="/dev/cu.usbserial-0001",
    resolution=8,
    fov_range=(-90, 90),
)

l.start()

while True:
    readings, _ = l.get_state()
    print("\n")
    print("\n")
    for a,d in readings:
        print(int(a)," => ",int(d))
    print("\n")
    print("\n")

    time.sleep(0.1)
