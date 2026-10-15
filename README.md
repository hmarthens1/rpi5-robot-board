# rpi5-robot-board

ROS 2 software for **robot04**, a Hiwonder **MentorPi T1** in the robot fleet: a tracked
robot with a **Raspberry Pi 5**, the **Hiwonder RRC Lite** controller (STM32F407), an
**Oradar MS200** lidar and a **Deptrum Aurora 930** depth camera.

It speaks the same interface as the Pi 4 robots
([rpi4-robot-board](https://github.com/hmarthens1/rpi4-robot-board)), so the fleet dashboard
and voice agent (robot-fleet-gui) drive it without special cases, and it adds what this
robot has on top: wheel-encoder odometry, measured moves and turns, a depth camera and a lidar.

Set the Pi up first with [rpi5-ros2-rrclite](https://hmarthens1.github.io/rpi5-ros2-rrclite/):
Lab 01 (Ubuntu Server 24.04 + ROS 2 Jazzy), Lab 02 (the RRC Lite, `/dev/rrclite`) and
**Lab 03 (flash the Lab 03 firmware)** — this package needs its encoder report and watchdog.

## Layout

```
ros2/mentorpi_robot/                 ROS 2 package (Python, ament_python)
  mentorpi_robot/rrclite.py            RRC Lite serial driver: 0xAA 0x55 protocol at 1 Mbaud
  mentorpi_robot/robot_node.py         the fleet interface, odometry, safety
  mentorpi_robot/camera_bridge.py      depth camera -> the fleet's vision topics (+ stall watchdog)
  mentorpi_robot/ms200_node.py         Oradar MS200 lidar -> scan
  launch/robot.launch.py               everything, in namespace /<hostname>
  config/robot.yaml                    limits, motor profile, track geometry
scripts/install_service.sh           start it at boot (systemd unit robot-fleet-node)
udev/99-ms200-lidar.rules            /dev/lidar
pc/robot04_slam/                     (PC) Hector SLAM launch + RViz config for the robot's scan
pc/robot04-mapper/                   (PC) app: camera view, teleop, start SLAM, save maps
```

## Interface (namespace `/<hostname>`, e.g. `/robot04`)

Same as the Pi 4 fleet:

| Topic | Type | |
|---|---|---|
| `command` | `std_msgs/String` (JSON `{"id", "action", ...}`) | in |
| `command_result` | `std_msgs/String` (JSON `{"id", "action", "ok", "detail"}`) | out |
| `cmd_vel` | `geometry_msgs/Twist` — `linear.x` forward, `angular.z` turn, −1..1 of `max_speed` | in |
| `battery` | `sensor_msgs/BatteryState`, 1 Hz | out |
| `system` | `diagnostic_msgs/DiagnosticStatus`, 1 Hz — the fleet's keys plus `robot_type`, `encoders`, `lidar` | out |
| `vision/image/compressed` | `sensor_msgs/CompressedImage`, JPEG 320 px, 3 Hz while subscribed | out |
| `vision/state` | `std_msgs/String` JSON: `center_m`, `nearest_m`, `nearest_x` from the depth image | out |
| `vision/control` | `std_msgs/String` JSON `{"image": "rgb"\|"depth"}` | in |

New on this robot:

| Topic | Type | |
|---|---|---|
| `odom` (+ TF `robot04/odom → robot04/base_footprint`) | `nav_msgs/Odometry`, 20 Hz, from the track encoders | out |
| `imu` | `sensor_msgs/Imu` (RRC Lite QMI8658, no orientation) | out |
| `scan` | `sensor_msgs/LaserScan` — used for obstacle checks in the direction of travel | in |
| `camera/rgb/image_raw`, `camera/depth/image_raw` | from the Deptrum driver | out |

TF frames carry the robot's name (`robot04/...`) because TF is global across the fleet.

**Actions** (`command`): `status`, `stop`, `drive {vx, vy, turn, duration}` (`vx` is refused:
tracks can't slide), `nudge {direction, seconds, speed}`, `motor {motor, speed, duration}`,
`led {on}`, `buzzer {seconds, times}`, and closed-loop on the encoders:
`move {distance_m}` and `turn {degrees}` (positive = counter-clockwise). Arm, servo and RGB
actions are answered with `ok: false`.

**Safety**, enforced on the robot whatever the sender checked: speeds are fractions of
`max_speed` (50 %), timed motions stop after `max_duration` (5 s), `move`/`turn` after
`move_timeout` (15 s), `stop` always wins, `cmd_vel` stops 0.5 s after the last message, and
motion towards an obstacle closer than `min_clearance` on the lidar is refused or stopped.
Underneath, the RRC Lite firmware stops the motors by itself 300 ms after the Pi goes quiet,
and every stop sets the PWM to 0 (Lab 03).

## Install (on the robot)

```bash
# 1. this repo
git clone https://github.com/hmarthens1/rpi5-robot-board.git ~/rpi5-robot-board
ln -s ~/rpi5-robot-board/ros2/mentorpi_robot ~/ros2_ws/src/mentorpi_robot
source /opt/ros/jazzy/setup.bash && cd ~/ros2_ws && colcon build --symlink-install --packages-select mentorpi_robot

# 2. start it at boot (ROS_DOMAIN_ID is read from ~/.bashrc)
sudo bash ~/rpi5-robot-board/scripts/install_service.sh
journalctl -u robot-fleet-node -f
```

### The depth camera driver (not in this repo)

The Aurora 930 needs Deptrum's ROS 2 driver,
`deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz`, from Hiwonder's MentorPi
material (*11. Depth Camera Basics Course → 4 Appendix → 2 Source Code → ROS2*). It is
licensed for personal use, so it is **not** included here. It was written for ROS 2 Foxy;
on Jazzy it builds after four small changes:

```bash
sudo apt install -y ros-jazzy-cv-bridge ros-jazzy-tf2-geometry-msgs ros-jazzy-angles \
  libopencv-dev libgoogle-glog-dev libgflags-dev libboost-system-dev libboost-filesystem-dev
mkdir -p ~/vendor ~/vendor_ws/src
tar xzf deptrum-ros-driver-aurora930-aarch64-0.2.1001-source.tar.gz -C ~/vendor
D=~/vendor/deptrum-ros-driver-aurora930-0.2.1001
# 1. package name must match the CMake project on Jazzy
sed -i 's#<name>deptrum-ros-driver</name>#<name>deptrum-ros-driver-aurora930</name>#' $D/package.xml
# 2. cv_bridge and tf2_geometry_msgs headers are .hpp now
grep -rl -E 'cv_bridge/cv_bridge\.h>|tf2_geometry_msgs/tf2_geometry_msgs\.h>' $D/src $D/include $D/subscribe_node \
  | xargs sed -i -E 's#cv_bridge/cv_bridge\.h>#cv_bridge/cv_bridge.hpp>#; s#tf2_geometry_msgs/tf2_geometry_msgs\.h>#tf2_geometry_msgs/tf2_geometry_msgs.hpp>#'
# 3. USB permission for the camera (3251:1930)
sudo cp $D/ext/deptrum-stream-aurora900-linux-aarch64-v1.1.19-18.04/scripts/99-deptrum-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# 4. build without its lint tests
ln -s $D ~/vendor_ws/src/deptrum_ros_driver
cd ~/vendor_ws && colcon build --cmake-args -DSTREAM_SDK_TYPE=AURORA930 -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

`install_service.sh` sources `~/vendor_ws` when it exists, and the launch file starts the
camera only when the driver is installed (`camera:=false` to leave it out).

### Lidar (Oradar MS200)

The MS200 sits behind a CH340 USB-serial adapter (`1a86:7523`) and streams 47-byte packets
at 230400 baud (LD06 layout, CRC-8 poly 0x4D). `udev/99-ms200-lidar.rules` names it
`/dev/lidar`; it matches the USB port too, because the Hiwonder voice module is also a
CH340. Check the port with `udevadm info -q property /dev/ttyUSB0 | grep ID_PATH=` and edit
the rule if yours differs.

```bash
sudo cp udev/99-ms200-lidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=tty
ls -l /dev/lidar
```

The launch file starts `ms200_node` when `/dev/lidar` exists (`lidar:=false` to leave it out).
Returns under 0.10 m (`range_min`) are dropped: that is the robot's own frame.

**OpenCV:** use Ubuntu's 4.6 (`libopencv-dev`, `python3-opencv`), not a source build: Jazzy's
`cv_bridge` and the Deptrum driver link against 4.6, and two OpenCVs in one process clash.

## On the PC: mapping with Hector SLAM

The robot only publishes its scan; SLAM runs on the PC (ROS 2 Humble works with the
robot's Jazzy over DDS: same `ROS_DOMAIN_ID=17`). Hector SLAM has no official ROS 2
release; [RobotX-Workshops/hector_slam_ros2](https://github.com/RobotX-Workshops/hector_slam_ros2)
(BSD-2, a Jazzy port) builds on Humble unchanged.

```bash
mkdir -p ~/hector_ws/src && cd ~/hector_ws/src
git clone https://github.com/RobotX-Workshops/hector_slam_ros2.git
ln -s ~/Desktop/rpi5-robot-board/pc/robot04_slam .        # this repo's launch + RViz config
cd ~/hector_ws && source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

~/Desktop/rpi5-robot-board/pc/robot04-mapper/run.sh       # the app (see its README)
# or without the app:
ros2 launch robot04_slam hector.launch.py                  # hector + RViz
```

Hector matches each scan against the map (no odometry) and publishes
`map -> robot04/odom`, so the encoder odometry stays underneath. Maps are saved to
`~/Desktop/robot04_maps` as map_server `pgm`/`yaml` (Nav2 can load them).

Measured on robot04 over Wi-Fi: scan latency ≈ 6 ms on top of the 100 ms scan, ~20 KB/s;
`ms200_node` ≈ 9 % of one Pi core. The Pi 5 could run Hector too; SLAM is on the PC
for the RViz/tuning convenience. Move it onto the robot once it navigates by itself,
so a Wi-Fi dropout doesn't lose its localisation.

## Calibration

`config/robot.yaml`:

| Parameter | Value | How it was set |
|---|---|---|
| `motor_profile` | `JGB520` | JGB37-520, 1:90, 11-line encoder: 3960 counts per turn (Lab 03) |
| `left_motor`, `right_motor` | 1, 2 | the ports the tracks are plugged into |
| `left_sign`, `right_sign` | +1, −1 | checked on robot04: `drive vy=0.3` runs both tracks forward, `turn=0.3` turns left |
| `wheel_radius` | 0.0325 m | **estimate** — calibrate: `move` 0.5 m on the floor, measure, scale by measured/0.5 |
| `track_width` | 0.170 m | **estimate** — calibrate: `turn` 360°, scale by the angle actually turned |

## Status

| Part | |
|---|---|
| Fleet interface, odometry, IMU, safety, boot service | working on robot04 |
| Depth camera (Deptrum driver on Jazzy) → vision topics | working: RGB and depth 15 Hz, JPEG 10 Hz at 480 px (`camera_rate`, `camera_width` launch args). The driver sometimes stalls: `camera_bridge` kills it after 6 s without frames and launch respawns it (~12 s to recover) |
| Lidar (`ms200_node`, own MS200 driver) | working: `scan` 10 Hz, 450 rays, 0 CRC errors; starts when `/dev/lidar` exists. 0° = the robot's front and counter-clockwise, checked against the depth camera on robot04 |
| Mapping (PC: Hector SLAM + robot04 mapper app) | working; teleop not yet driven on the floor |
| `wheel_radius` / `track_width` | estimates, floor calibration to do |
