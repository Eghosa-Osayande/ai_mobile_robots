import cv2
from simulations.controllers.mobile_robot.two_wheel_robots.integrations.camera_client import CamClient

# cam = CamClient(src=0)
cam = CamClient(tcp=("127.0.0.1", 8081))

while True:
    ok, frame = cam.read()
    if not ok:
        continue

    cv2.imshow("cam", frame)
    if cv2.waitKey(1) == 27:
        break

cam.release()
cv2.destroyAllWindows()