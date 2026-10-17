"""Autonomous exploration on robot04: SLAM, navigation and the explorer, all on the Pi.

   ros2 launch mentorpi_nav explore.launch.py      (robot.launch.py must already be running)

  hector_mapping      /<robot>/scan -> /map, map -> <robot>/odom          (SLAM)
  Nav2                planner (NavFn on the map + live lidar), Regulated Pure Pursuit
                      controller (local costmap from the live lidar), recoveries, BT navigator,
                      velocity smoother                                    (navigation)
  cmd_vel_adapter     Nav2 m/s -> robot_node fractions, muted until the explorer starts
  explorer            frontiers -> NavigateToPose goals; start/pause/home from the PC app

Nav2 runs in the root namespace (one robot on this domain uses it); everything robot-
specific (frames, topics) is in config/nav2.yaml. The robot keeps navigating if Wi-Fi to
the PC drops: only the visualisation is remote.
"""
import os
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAV2_NODES = ["controller_server", "planner_server", "behavior_server", "velocity_smoother", "bt_navigator"]


def nodes(context):
    robot = LaunchConfiguration("robot").perform(context)
    params = os.path.join(get_package_share_directory("mentorpi_nav"), "config", "nav2.yaml")
    raw, smooth = "/%s/nav/cmd_vel_raw" % robot, "/%s/nav/cmd_vel" % robot
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
                 "laser_max_dist": 8.0,
                 "laser_z_min_value": -1.0,
                 "laser_z_max_value": 1.0,
                 "use_tf_scan_transformation": True,
                 "use_tf_pose_start_estimate": False,
                 "map_pub_period": 1.0,
                 "advertise_map_service": True,
                 "scan_subscriber_queue_size": 5,
             }]),
        Node(package="nav2_controller", executable="controller_server", output="screen",
             parameters=[params], remappings=[("cmd_vel", raw)]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen",
             parameters=[params]),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output="screen",
             parameters=[params], remappings=[("cmd_vel", raw)]),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother",
             output="screen", parameters=[params],
             remappings=[("cmd_vel", raw), ("cmd_vel_smoothed", smooth)]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="screen",
             parameters=[params]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output="screen", parameters=[{"autostart": True, "node_names": NAV2_NODES, "bond_timeout": 10.0}]),
        Node(package="mentorpi_nav", executable="cmd_vel_adapter", namespace=robot, output="screen"),
        Node(package="mentorpi_nav", executable="explorer", namespace=robot, output="screen",
             respawn=True, respawn_delay=2.0,       # comes back idle: the PC has to press Explore again
             parameters=[{"base_frame": "%s/base_footprint" % robot,
                          "max_minutes": float(LaunchConfiguration("max_minutes").perform(context))}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value=socket.gethostname()),
        DeclareLaunchArgument("max_minutes", default_value="45.0"),
        OpaqueFunction(function=nodes),
    ])
