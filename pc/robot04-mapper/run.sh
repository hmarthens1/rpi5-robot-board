#!/bin/bash
# Start the robot04 mapper. Options go to robot04_mapper (see --help).
# ROS settings match robot04 (Lab 01 / the 2026-10-13 DDS note): domain 17, Cyclone DDS.
# PYTHONNOUSERSITE: use Ubuntu's PyQt5/numpy and ROS's Python, not pip packages in ~/.local.
source /opt/ros/humble/setup.bash
source "$HOME/hector_ws/install/setup.bash"      # hector_mapping + robot04_slam
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-17}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_LOCALHOST_ONLY=0
cd "$(dirname "$0")"
PYTHONNOUSERSITE=1 exec python3 -m robot04_mapper "$@"
