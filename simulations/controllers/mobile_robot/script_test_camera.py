import cv2
from simulations.controllers.mobile_robot.two_wheel_robots.integrations.camera_link import CameraLink

cam = CameraLink(src=0)
cam.start()

while True:
    ok, frame = cam.read()
    if not ok:
        continue

    cv2.imshow("cam", frame)
    if cv2.waitKey(1) == 27:
        break

cam.release()
cv2.destroyAllWindows()