# robot04 mapper

Drive robot04 (Hiwonder MentorPi T1) from its camera view, run Hector SLAM on this PC,
and save the maps. Separate from robot-fleet-gui on purpose: one robot, one job.

```bash
./run.sh                    # --robot robot04 by default
```

| Panel | |
|---|---|
| Camera | `/robot04/vision/image/compressed`, 480 px, 10 Hz (camera_bridge); "depth view" switches to the colourised depth image |
| Drive | hold **W A S D** or the arrow keys (or the on-screen buttons); **Space** or STOP stops. Sends `cmd_vel` at 10 Hz only while held; speed/turn sliders are % of the robot's max speed |
| Map | Hector's `/map`, the robot (green), its path (blue), the live scan (red). Wheel zoom, drag pan, double-click follow |
| SLAM | **Start SLAM** runs `ros2 launch robot04_slam hector.launch.py` (from `~/hector_ws`); **New map** resets it; **Open RViz** for the full view |
| Save | to `~/Desktop/robot04_maps/<name>.{pgm,yaml,png,json}`: map_server format (Nav2 loads the yaml), a PNG with the path, and notes (size, explored area, path length). SLAM logs go to `logs/` there |

Safety lives on the robot, not here: `robot_node` stops 0.5 s after the last `cmd_vel`,
refuses to drive towards an obstacle closer than 0.25 m (lidar, ±25° front/back), and the
RRC Lite firmware stops the motors 300 ms after the Pi goes quiet. The app also stops
when its window loses focus.

Mapping tips: drive slowly and turn slowly (Hector has no odometry and loses track on
fast turns, especially at the MS200's 10 Hz); revisit the start before saving; long bare
corridors drift.

Needs: ROS 2 Humble, PyQt5, numpy (Ubuntu packages), `~/hector_ws` built (hector_mapping
port + robot04_slam), and on the robot `rpi5-robot-board` with the MS200 driver.
ROS settings: `ROS_DOMAIN_ID=17`, Cyclone DDS (set in run.sh).
