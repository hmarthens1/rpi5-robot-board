import argparse
import sys

from PyQt5.QtWidgets import QApplication

from .app import MapperWindow


def main():
    ap = argparse.ArgumentParser(description="Drive a robot from its camera, run Hector SLAM, save maps.")
    ap.add_argument("--robot", default="robot04", help="ROS namespace of the robot (default robot04)")
    args = ap.parse_args()
    app = QApplication(sys.argv[:1])
    w = MapperWindow(args.robot)
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
