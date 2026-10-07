#!/bin/bash
# =============================================================================
# rpi5-robot-board - start the robot's fleet nodes at boot (systemd unit robot-fleet-node)
# =============================================================================
# Runs `ros2 launch mentorpi_robot robot.launch.py` as the normal user, in the
# fleet's ROS 2 settings (ROS_DOMAIN_ID from ~/.bashrc, Fast DDS), restarted if
# it ever exits. Same idea as rpi4-robot-board's robot-status / robot-command.
#
#   sudo bash scripts/install_service.sh            # install + start
#   sudo bash scripts/install_service.sh --remove   # stop + remove
#   journalctl -u robot-fleet-node -f               # its log
#
# Extra launch arguments go in LAUNCH_ARGS below, e.g. "lidar:=true".
# =============================================================================
LAUNCH_ARGS=""

set -eu
[ "$(id -u)" -eq 0 ] || { echo "run with sudo"; exit 1; }
USER_NAME=${SUDO_USER:?run with sudo from the robot user}
HOME_DIR=$(getent passwd "$USER_NAME" | cut -d: -f6)
UNIT=/etc/systemd/system/robot-fleet-node.service

if [ "${1:-}" = "--remove" ]; then
  systemctl disable --now robot-fleet-node.service 2>/dev/null || true
  rm -f "$UNIT"
  systemctl daemon-reload
  echo "removed"
  exit 0
fi

DOMAIN=$(grep -oP '^export ROS_DOMAIN_ID=\K[0-9]+' "$HOME_DIR/.bashrc" | tail -1)
DOMAIN=${DOMAIN:-17}
SETUP="source /opt/ros/jazzy/setup.bash"
[ -f "$HOME_DIR/vendor_ws/install/setup.bash" ] && SETUP="$SETUP && source $HOME_DIR/vendor_ws/install/setup.bash"
SETUP="$SETUP && source $HOME_DIR/ros2_ws/install/setup.bash"

cat > "$UNIT" <<EOF
[Unit]
Description=Robot fleet node ($(hostname)): mentorpi_robot robot.launch.py
After=network-online.target
Wants=network-online.target

[Service]
User=$USER_NAME
Environment=ROS_DOMAIN_ID=$DOMAIN
Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp
Environment=ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ExecStart=/bin/bash -c '$SETUP && exec ros2 launch mentorpi_robot robot.launch.py $LAUNCH_ARGS'
KillSignal=SIGINT
TimeoutStopSec=10
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable --now robot-fleet-node.service
sleep 3
systemctl --no-pager --lines=0 status robot-fleet-node.service | head -4
echo "log: journalctl -u robot-fleet-node -f"
