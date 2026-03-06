# Terminal-based Pioneer P3-DX control (no GUI, keyboard control)

import time


from simulations.controllers.mobile_robot.two_wheel_robots.pioneer3dx import (
    Pioneer3dx,
)


COM_PORT = "/dev/cu.usbserial-10"
BAUD_RATE = 9600
duration = 1

robot = Pioneer3dx(
    serial_port_baud=(COM_PORT, BAUD_RATE),
)

try:
    print("Terminal control active")

    while True:
        x = input("")

        if x:
            robot.move_wheels(0.1, 0.1)
            time.sleep(duration)
            robot.move_wheels(0, 0)


except Exception as e:
    print("Error:", e)

except KeyboardInterrupt as e:
    print("User cancelled")

finally:
    robot.shutdown()
