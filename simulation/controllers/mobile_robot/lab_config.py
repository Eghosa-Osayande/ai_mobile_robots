from dataclasses import dataclass
from dotenv import load_dotenv
import os
import argparse


@dataclass
class LabConfig:
    port: str
    isSerial: bool
    leftMotorDeviceName: str
    rightMotorDeviceName: str
    waypointsFilename: str
    nodes: list[tuple[float, float]]
    isRightHandCart: bool
    wheelRadius: float
    wheelSeperation: float

    def __init__(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("--env", default="env")
        args, _unknown = parser.parse_known_args()

        load_dotenv(args.env)

        self.port = os.getenv("port", "COM1")

        self.isSerial = os.getenv("is_serial", "false") == "true"
        self.isRightHandCart = os.getenv("use_right_hand_cartesian_system") != "true"

        self.leftMotorDeviceName = os.getenv("left_motor_device_name")

        self.rightMotorDeviceName = os.getenv("right_motor_device_name")

        try:
            wheelRadius = os.getenv("wheel_radius_meter")
            self.wheelRadius = float(wheelRadius)
        except ValueError as e:
            print("invalid wheel_radius_meter")
            raise e

        try:
            wheelSeperation = os.getenv("wheel_seperation")
            self.wheelSeperation = float(wheelSeperation)
        except ValueError as e:
            print("invalid wheel_seperation")
            raise e

        self.waypointsFilename = os.getenv("waypoints_filename")

        self.nodes = []
        contents = ""

        try:
            with open(self.waypointsFilename, "r") as fd:
                contents: str = fd.read()
                lines = contents.split("\n")
                linesSplit = []
                for line in lines:
                    line = line.strip()
                    if not line:
                        continue
                    linesSplit.append(line.split(","))

                self.nodes = [(float(line[0]), float(line[1])) for line in linesSplit]

        except FileNotFoundError:
            print(f"waypoint file not found, creating ... {self.waypointsFilename}")
            fd = open(self.waypointsFilename, "w")
            fd.write("0,0")
            fd.close()

        except Exception as e:
            print(f"Failed to parse waypoint file, error {e}")
            raise e
