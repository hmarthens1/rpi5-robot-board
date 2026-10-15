"""Hector SLAM on the laptop for a fleet robot's lidar.

The robot publishes /<robot>/scan (MS200, 10 Hz), TF <robot>/odom -> <robot>/base_footprint
(encoders) and <robot>/base_footprint -> <robot>/laser (static). hector_mapping matches each
scan against the map (no odometry needed) and publishes map -> <robot>/odom, so the encoder
odometry stays as it is and the robot appears at its SLAM pose in the map.

   ros2 launch robot04_slam hector.launch.py [robot:=robot04] [rviz:=true]
   ros2 topic pub --once /syscommand std_msgs/String "data: reset"     # start a new map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    robot = LaunchConfiguration("robot").perform(context)
    rviz_cfg = os.path.join(get_package_share_directory("robot04_slam"), "rviz", "hector.rviz")
    return [
        Node(package="hector_mapping", executable="hector_mapping", name="hector_mapping", output="screen",
             parameters=[{
                 "scan_topic": "/%s/scan" % robot,
                 "base_frame": "%s/base_footprint" % robot,
                 "odom_frame": "%s/odom" % robot,
                 "map_frame": "map",
                 "pub_map_odom_transform": True,
                 "map_resolution": 0.05,
                 "map_size": 1024,                  # 51 m square
                 "map_start_x": 0.5,
                 "map_start_y": 0.5,
                 "map_multi_res_levels": 2,
                 "update_factor_free": 0.4,
                 "update_factor_occupied": 0.9,
                 "map_update_distance_thresh": 0.2,
                 "map_update_angle_thresh": 0.06,
                 "laser_min_dist": 0.12,            # robot's own frame is inside 0.10 m
                 "laser_max_dist": 8.0,             # MS200 is rated 12 m; far returns are noisy
                 "laser_z_min_value": -1.0,
                 "laser_z_max_value": 1.0,
                 "use_tf_scan_transformation": True,
                 "use_tf_pose_start_estimate": False,
                 "map_pub_period": 1.0,
                 "advertise_map_service": True,
                 "scan_subscriber_queue_size": 5,
             }]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg], output="log",
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="robot04"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=nodes),
    ])
