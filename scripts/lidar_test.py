import time

from simulations.controllers.mobile_robot.two_wheel_robots.integrations.lidar_link import (
    LidarLink,
)


l = LidarLink(
    port= ("localhost",5000),
    resolution=8,
    fov_range=(-90, 90),
)

l.start()

while True:
    readings, _ = l.get_state()
    print(readings)
    time.sleep(0.1)
    
