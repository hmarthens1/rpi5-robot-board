"""robot04's fleet nodes, in namespace /<hostname>:
   ros2 launch mentorpi_robot robot.launch.py   [lidar:=true] [camera:=false] [point_cloud:=true]

The Aurora 930 driver (deptrum-ros-driver-aurora930) is a vendor package, installed
separately (see README); the camera nodes start only when it is installed."""
import os
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import PackageNotFoundError
from launch_ros.actions import Node

CAMERA_PKG = "deptrum-ros-driver-aurora930"


def camera_installed():
    try:
        get_package_share_directory(CAMERA_PKG)
        return True
    except PackageNotFoundError:
        return False


def generate_launch_description():
    ns = socket.gethostname()
    config = os.path.join(get_package_share_directory("mentorpi_robot"), "config", "robot.yaml")
    lidar = LaunchConfiguration("lidar")
    return LaunchDescription([
        DeclareLaunchArgument("lidar", default_value="false"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/lidar"),
        DeclareLaunchArgument("camera", default_value="true" if camera_installed() else "false"),
        DeclareLaunchArgument("point_cloud", default_value="false"),
        Node(package="mentorpi_robot", executable="robot_node", namespace=ns, output="screen",
             parameters=[config]),
        Node(package="mentorpi_robot", executable="ms200_node", namespace=ns, output="screen",
             condition=IfCondition(lidar),
             parameters=[{"port": LaunchConfiguration("lidar_port"), "frame_id": ns + "/laser"}]),
        # Aurora 930: colour + depth, aligned, 15 fps; point cloud only on request (CPU)
        Node(package=CAMERA_PKG, executable="aurora930_node", namespace=ns + "/camera", output="log",
             condition=IfCondition(LaunchConfiguration("camera")),
             parameters=[{"rgb_enable": True, "depth_enable": True, "ir_enable": False,
                          "point_cloud_enable": LaunchConfiguration("point_cloud"),
                          "align_mode": True, "depth_correction": True, "rgb_fps": 15, "ir_fps": 15,
                          "boot_order": 1, "resolution_mode_index": 2, "log_dir": "/tmp/",
                          "minimum_filter_depth_value": 150, "maximum_filter_depth_value": 4000}]),
        Node(package="mentorpi_robot", executable="camera_bridge", namespace=ns, output="screen",
             condition=IfCondition(LaunchConfiguration("camera"))),
        # fixed frames (measure and correct for your robot)
        Node(package="tf2_ros", executable="static_transform_publisher", namespace=ns,
             arguments=["--x", "0.0", "--z", "0.12", "--frame-id", ns + "/base_footprint", "--child-frame-id", ns + "/laser"]),
        Node(package="tf2_ros", executable="static_transform_publisher", namespace=ns,
             arguments=["--x", "0.07", "--z", "0.09", "--frame-id", ns + "/base_footprint", "--child-frame-id", ns + "/depth_camera_link"]),
    ])
