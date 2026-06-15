# URSULA

**U**ncrewed G**r**ound Vehicle for **S**ensorised **L**ocalisation and **A**utonomous **N**avigation

A GPS-free autonomous ground vehicle developed as a capstone project at the University of Liverpool. URSULA is a skid-steered platform built on four hoverboard (swegway) hub motors, carrying a Slamtec RPLIDAR A3 and a Luxonis OAK-D Pro camera. It maps its environment using LiDAR SLAM, localises within saved maps, and navigates autonomously using Nav2 — all without GPS.

**ROS 2 Humble · Ubuntu 22.04 · Gazebo Classic 11**

---

## Table of Contents

1. [Hardware](#1-hardware)
2. [Software Architecture](#2-software-architecture)
3. [Repository Structure](#3-repository-structure)
4. [Installation](#4-installation)
5. [Network Setup](#5-network-setup)
6. [Running the Robot](#6-running-the-robot)
7. [Running the Simulation](#7-running-the-simulation)
8. [Mapping and Localisation](#8-mapping-and-localisation)
9. [Object Detection](#9-object-detection)
10. [Configuration Reference](#10-configuration-reference)
11. [Recovery Procedure](#11-recovery-procedure)
12. [Troubleshooting](#12-troubleshooting)

---

## Important Notes for Future Developers

### Primary Launch Files (Supported)

| Launch File | Purpose |
|---|---|
| `jetson_hardware_launch.launch.py` | Main onboard robot launch (Jetson) |
| `dev_hardware_launch.launch.py` | Operator interface (Laptop) |
| `sim_full_test_launch.launch.py` | Full simulation environment |
| `sim_teleop_launch.launch.py` | Simulation teleoperation |

### Legacy / Development Launch Files

Several additional launch files remain in the repository from earlier development stages. These are retained for reference and experimentation but should not be assumed to be fully tested or maintained. Future development should use the primary launch files listed above.

### Maps

Maps are stored outside the repository:

```
~/ros2_ws/maps
```

Back up this directory before reflashing or replacing the Jetson.

### Project Status

**Implemented:**
- Teleoperation
- LiDAR SLAM
- Localisation
- Nav2 navigation
- RViz visualisation
- Foxglove integration
- Remote networking
- OAK-D integration
- Object detection pipeline

**Future Work:**
- Semantic mapping
- Detection integration into persistent maps
- Advanced mission planning
- Oxygen sensor integration

---

## 1. Hardware

| Component | Part | Notes |
|---|---|---|
| Chassis | Custom fabricated steel frame | 560 mm wide, 480 mm long |
| Drive motors | 4× hoverboard hub motors | Skid-steer, rear pair driven |
| Motor controller | Arduino Uno + custom H-bridge board | Firmware in `arduino/motorcontrol.ino` |
| Compute | NVIDIA Jetson Orin Nano (onboard) | Runs full ROS stack |
| LiDAR | Slamtec RPLIDAR A3 | `/dev/ttyUSB0`, 256000 baud |
| Camera | Luxonis OAK-D Pro | USB3, depthai_ros_driver |
| Dev laptop | Any Ubuntu 22.04 machine | Runs joystick + RViz |

**Serial connections on Jetson:**
- Arduino → `/dev/ttyACM0` (USB)
- RPLIDAR → `/dev/ttyUSB0` (USB)

Verify before launching:
```bash
ls -la /dev/ttyACM*   # Arduino
ls -la /dev/ttyUSB*   # LiDAR
```

---

## 2. Software Architecture

URSULA runs as two separate ROS domains connected over a local network. The Jetson runs all sensor and navigation nodes. The dev laptop runs the operator interface.

```
DEV LAPTOP                          JETSON (onboard)
──────────────────────              ──────────────────────────────────────
joy_node                            rplidar_composition  (/scan)
teleop_twist_joy  ──/cmd_vel_joy──▶ twist_mux ──/cmd_vel──▶ serial_command_bridge
rviz2             ◀── all topics ── robot_state_publisher      │
                                    rf2o_laser_odometry         ▼
                                    slam_toolbox           Arduino Uno
                                    nav2 ──/cmd_vel_nav──▶ twist_mux    │
                                    foxglove_bridge                      ▼
                                    ursula_manager                   Motors
                                    oak-d driver + detection
```

**Topic priority (twist_mux):**

| Topic | Source | Priority |
|---|---|---|
| `/cmd_vel_joy` | Joystick (laptop) | 100 — always overrides |
| `/cmd_vel_keyboard` | Keyboard teleop | 50 |
| `/cmd_vel_nav` | Nav2 autonomous | 10 — lowest |

The joystick LB button (button 4) acts as a physical emergency stop directly in `serial_command_bridge`. Publishing `True` to `/ursula/estop` triggers a software emergency stop from any device.

---

## 3. Repository Structure

```
ursula/
├── arduino/
│   └── motorcontrol.ino          Arduino motor controller firmware
├── config/
│   ├── controller_teleop.yaml    PS4 / F310 joystick config
│   ├── ps4_teleop.yaml           PS4-specific axis mapping
│   ├── nav2_params.yaml          Nav2 full stack parameters
│   ├── serial_bridge_params.yaml Serial bridge + velocity limits
│   ├── slam_toolbox_params.yaml  SLAM mapping mode
│   ├── slam_toolbox_localization.yaml  SLAM localisation mode
│   ├── steering_tuning.md        Skid-steer tuning reference
│   └── twist_mux.yaml            cmd_vel priority chain
├── launch/
│   ├── jetson_hardware_launch.launch.py   MAIN real-robot launch (Jetson)
│   ├── dev_hardware_launch.launch.py      Operator interface (laptop)
│   ├── sim_full_test_launch.launch.py     MAIN simulation launch
│   ├── sim_teleop_launch.launch.py        Sim joystick (separate terminal)
│   ├── robot_launch.launch.py             Legacy full launch (reference only)
│   ├── jetson_rtabmap.launch.py           Alternative: RTAB-Map 3D mapping
│   ├── camera.launch.py                   OAK-D camera driver
│   ├── detection.launch.py                YOLOv4 spatial detection
│   ├── nav2_launch.launch.py              Nav2 stack (called by others)
│   └── laptop_slam_test_launch.launch.py  LiDAR SLAM on laptop only
├── networking/
│   └── fastdds_super_client.xml  FastDDS discovery server config
├── scripts/
│   ├── serial_command_bridge.py  cmd_vel → Arduino serial (ROS node)
│   ├── ursula_manager.py         Map save service + status publisher
│   ├── detection_mapper.py       Detection → map markers (Foxglove)
│   ├── detection_visualiser.py   Annotated camera image publisher
│   ├── mock_detector.py          Sim stand-in for YOLO node
│   └── ursula_webpanel.py        Browser-based control panel (port 8080)
├── urdf/
│   ├── ursula.urdf.xacro         Main robot description
│   ├── lidar.xacro               RPLIDAR A3 link + joint
│   ├── oak_d.xacro               OAK-D camera links
│   └── gazebo_plugins.xacro      Gazebo diff-drive + sensor plugins
├── worlds/
│   ├── ursula_maze.world         Small maze (original test world)
│   └── ursula_open_lab.world     Large open lab (primary sim world)
├── rviz/
│   └── ursula.rviz               RViz2 configuration
├── CMakeLists.txt
└── package.xml
```

---

## 4. Installation

Complete setup from a fresh Ubuntu 22.04 installation. Run all commands on **both the Jetson and the dev laptop** unless stated otherwise.

### 4.1 Install ROS 2 Humble

### 4.2 Install ROS 2 dependencies

```bash
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-nav2-msgs \
  ros-humble-nav2-costmap-2d \
  ros-humble-nav2-controller \
  ros-humble-nav2-planner \
  ros-humble-nav2-behaviors \
  ros-humble-nav2-bt-navigator \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-teleop-twist-joy \
  ros-humble-teleop-twist-keyboard \
  ros-humble-joy \
  ros-humble-twist-mux \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-rplidar-ros \
  ros-humble-foxglove-bridge \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  python3-serial \
  python3-colcon-common-extensions
```

### 4.3 Install RF2O laser odometry

RF2O is not in the apt repositories and must be built from source:

```bash
cd ~/ros2_ws/src
git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git
```

### 4.4 Install OAK-D driver (Jetson only)

The OAK-D camera requires the Luxonis depthai-ros driver. For initial development, it was installed in a separate workspace to keep it isolated.

### 4.5 Clone and build URSULA

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/WillNewton129/ursula.git

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

# Make all Python scripts executable
chmod +x ~/ros2_ws/src/ursula/scripts/*.py
```

### 4.6 Shell setup

Add the following to `~/.bashrc` on both machines. The order of source lines matters — overlay workspaces must come after their dependencies.

```bash
# ROS 2 Humble
source /opt/ros/humble/setup.bash

# OAK-D driver (Jetson only — skip on laptop if not installed)
source ~/oak_camera_ws/install/setup.bash

# URSULA workspace
source ~/ros2_ws/install/setup.bash

# ROS domain — both machines must match
export ROS_DOMAIN_ID=42
```

Apply immediately:
```bash
source ~/.bashrc
```

> **Note:** Do not set `RMW_IMPLEMENTATION` or `ROS_DISCOVERY_SERVER` in `.bashrc`. These are mode-specific and are set by the networking scripts described in Section 5.

### 4.7 Create maps directory

```bash
mkdir -p ~/ros2_ws/maps
```

### 4.8 Flash the Arduino

Open `arduino/motorcontrol.ino` in the Arduino IDE. Select **Arduino Uno** as the board and the correct port (`/dev/ttyACM0` on the Jetson), then upload. No library dependencies are required beyond the Arduino standard library.

---

## 5. Network Setup

URSULA supports two operating modes depending on whether the Jetson and laptop are on the same network. Both modes require the same `ROS_DOMAIN_ID` in `~/.bashrc` (see section 4.6).

Switching between modes is done by sourcing the appropriate script rather than editing `~/.bashrc` directly. This keeps mode-specific settings isolated and avoids conflicts when multiple network interfaces are active.

### 5.1 Local Mode (Recommended)

Use when the Jetson and laptop are on the **same WiFi network, hotspot, or Ethernet connection**. No Fast DDS Discovery Server is required. This was the primary operating mode used throughout development.

On both machines, source the local mode script before launching:
> **Note:** Naming convention and file content needs confirming, files exist on all ROS devices.

```bash
source ~/local_mode.sh
```

`local_mode.sh` (place in `~` on both machines):

```bash
#!/bin/bash
unset RMW_IMPLEMENTATION
unset ROS_DISCOVERY_SERVER
unset FASTRTPS_DEFAULT_PROFILES_FILE
echo "Local mode active — standard DDS discovery"
```

Verify topics are visible across machines:

```bash
# On laptop — should show Jetson's topics
ros2 topic list
```

### 5.2 Remote Mode (Tailscale + Fast DDS Discovery Server)

Use when the Jetson and laptop are on **different networks**. Remote mode requires:

1. **Tailscale** — creates a VPN tunnel between devices
2. **Fast DDS Discovery Server** — running on the laptop, used by the Jetson to find ROS peers

#### Tailscale Setup

If you are setting up for the first time or migrating to a new account:

1. Create a new Tailscale account at [tailscale.com](https://tailscale.com)
2. Install Tailscale on both the Jetson and the dev laptop 
3. Authenticate both devices to the same account
4. Verify connectivity:

```bash
tailscale status
tailscale ping <device-name>
```

5. Update `networking/fastdds_super_client.xml` with the new Discovery Server Tailscale IP (the laptop's Tailscale IP):

```xml
<udpv4 ... address="<LAPTOP_TAILSCALE_IP>" port="11811"/>
```

#### FastDDS Configuration

FastDDS must be bound to the Tailscale network interface specifically. Binding to `0.0.0.0` causes instability when multiple network interfaces are active. The `networking/fastdds_super_client.xml` file handles this via `interfaceWhiteList`.

#### Remote Mode Scripts

`remote_mode.sh` (place in `~` on both machines):
> **Note:** Naming convention and file content needs confirming, files exist on all ROS devices.


```bash
#!/bin/bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="<LAPTOP_TAILSCALE_IP>:11811"
export FASTRTPS_DEFAULT_PROFILES_FILE=~/ros2_ws/src/ursula/networking/fastdds_super_client.xml
echo "Remote mode active — FastDDS discovery via Tailscale"
```

Replace `<LAPTOP_TAILSCALE_IP>` with the laptop's actual Tailscale IP address.

### 5.3 Foxglove Remote Monitoring

Foxglove Studio runs on any device (laptop, tablet, or phone) and provides remote monitoring, topic inspection, recording, and diagnostics.

Once `jetson_hardware_launch.launch.py` is running on the Jetson with `foxglove:=true`:

1. Open [Foxglove Studio](https://studio.foxglove.dev) or the desktop app
2. Connect via WebSocket:

| Mode | URL |
|---|---|
| Local | `ws://<jetson-ip>:8765` |
| Remote | `ws://<jetson-tailscale-ip>:8765` |

3. The URSULA web panel (if running) is accessible at `http://<jetson-ip>:8080`

---

## 6. Running the Robot

Two terminals are required — one on the Jetson, one on the laptop. Both must have completed the shell setup in section 4.6 and sourced the appropriate network mode script.

### 6.1 Quick Start

#### Local Mode

Jetson:

```bash
source ~/local_mode.sh
ros2 launch ursula jetson_hardware_launch.launch.py
```

Laptop:

```bash
source ~/local_mode.sh
ros2 launch ursula dev_hardware_launch.launch.py
```

#### Remote Mode

Laptop (start the discovery server first):

```bash
fastdds discovery --server-id 0 --udp-address <LAPTOP_TAILSCALE_IP> --udp-port 11811
source ~/remote_mode.sh
```

Jetson:

```bash
source ~/remote_mode.sh
ros2 launch ursula jetson_hardware_launch.launch.py
```

Laptop (in a new terminal, after sourcing remote_mode.sh):

```bash
ros2 launch ursula dev_hardware_launch.launch.py
```

### 6.2 Jetson — onboard stack options

```bash
# Full stack: SLAM + Nav2 + camera + detection + Foxglove
ros2 launch ursula jetson_hardware_launch.launch.py

# Manual teleop only (no Nav2 or camera, faster startup for testing)
ros2 launch ursula jetson_hardware_launch.launch.py nav2:=false camera:=false detection:=false

# Localisation mode (after a map has been saved)
ros2 launch ursula jetson_hardware_launch.launch.py slam_mode:=localization
```

**jetson_hardware_launch.launch.py arguments:**

| Argument | Default | Description |
|---|---|---|
| `slam_mode` | `mapping` | `mapping` builds a new map; `localization` loads the saved map |
| `nav2` | `true` | Launch Nav2 autonomous navigation |
| `foxglove` | `true` | Start Foxglove WebSocket bridge on port 8765 |
| `camera` | `true` | Launch OAK-D camera driver |
| `detection` | `true` | Launch YOLOv4 detection + detection_mapper |

### 6.3 Laptop — operator interface

In a new terminal on the laptop:

```bash
ros2 launch ursula dev_hardware_launch.launch.py
```

This starts the joystick driver, teleop node, and RViz.

### 6.4 Controller layout (PS4 / Logitech F310)

| Input | Action |
|---|---|
| Hold **RB** (button 5) | Deadman switch — must hold to move |
| Left stick up/down | Forward / backward |
| Right stick left/right | Turn left / right |
| **LB** (button 4) | Emergency stop |

If no controller is available, keyboard teleop can be used instead:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap /cmd_vel:=/cmd_vel_joy
```

Press `i` to move forward, `j`/`l` to turn, `k` to stop. Speed can be adjusted with `q`/`z`.

### 6.5 Emergency stop

Three methods are available:

1. **Physical:** Release RB on the controller (twist_mux stops publishing)
2. **Physical:** Press LB (button 4) — Arduino applies motor breaking

### 6.6 Recommended startup sequence for first run

Run in this order to verify each layer before adding the next:

```bash
# Step 1: motors + lidar + SLAM only
ros2 launch ursula jetson_hardware_launch.launch.py nav2:=false camera:=false detection:=false

# Step 2: add camera, check topics appear
ros2 launch ursula jetson_hardware_launch.launch.py nav2:=false detection:=false

# Step 3: add detection, check markers in Foxglove
ros2 launch ursula jetson_hardware_launch.launch.py nav2:=false

# Step 4: full stack
ros2 launch ursula jetson_hardware_launch.launch.py
```

---

## 7. Running the Simulation

Simulation does not require the Jetson or any hardware. Run everything on the dev laptop.

### 7.1 Full simulation stack

```bash
# Terminal 1 — full sim (Gazebo + SLAM + Nav2 + mock detection + Foxglove + RViz)
ros2 launch ursula sim_full_test_launch.launch.py

# Terminal 2 — joystick (run separately so it can be started/stopped independently)
ros2 launch ursula sim_teleop_launch.launch.py
```

Gazebo loads the `ursula_open_lab.world` by default (20 m × 16 m open lab). The robot spawns after 30 seconds to give Gazebo time to fully load.

### 7.2 Simulation launch arguments

| Argument | Default | Description |
|---|---|---|
| `world` | `open_lab` | `open_lab` (large) or `maze` (small original) |
| `slam_mode` | `mapping` | `mapping` or `localization` |
| `nav2` | `true` | Launch Nav2 |
| `foxglove` | `true` | Foxglove bridge on port 8765 |
| `camera` | `true` | Enable Gazebo camera + mock detector |
| `detection` | `true` | Enable detection_mapper |
| `use_rviz` | `true` | Launch RViz2 |

Examples:

```bash
# SLAM only, no Nav2 or detection (fastest startup for map building)
ros2 launch ursula sim_full_test_launch.launch.py nav2:=false detection:=false camera:=false

# Use the original maze world
ros2 launch ursula sim_full_test_launch.launch.py world:=maze

# Localisation mode (after saving a map)
ros2 launch ursula sim_full_test_launch.launch.py slam_mode:=localization

# No RViz (use Foxglove instead)
ros2 launch ursula sim_full_test_launch.launch.py use_rviz:=false
```

### 7.3 Simulation notes

- The Gazebo diff-drive plugin replaces the RPLidar, RF2O, serial bridge, and Arduino. Behaviour is equivalent but frictionless.
- `mock_detector.py` replaces the OAK-D YOLOv4 node. It publishes synthetic detections (random COCO classes) to verify the detection_mapper pipeline without camera hardware.
- Sim maps are saved to `~/ros2_ws/maps/sim/` and survive reboots.

---

## 8. Mapping and Localisation

### 8.1 Building a map

1. Launch in mapping mode (default): `slam_mode:=mapping`
2. Drive around the entire area using the joystick — cover all spaces you want on the map
3. When the map looks complete in RViz/Foxglove, save it:

```bash
ros2 service call /ursula/save_map std_srvs/srv/Trigger {}
```

This saves two copies:
- `~/ros2_ws/maps/ursula_map.yaml` / `.pgm` — the "latest" copy used by localisation
- `~/ros2_ws/maps/ursula_map_<timestamp>.yaml` / `.pgm` — timestamped archive
- A `.png` preview is auto-generated for visual inspection

View the map:
```bash
xdg-open ~/ros2_ws/maps/ursula_map.png
```

### 8.2 Localising on a saved map (Temperamental, Needs further testing)

After saving a map, relaunch with `slam_mode:=localization`:

```bash
ros2 launch ursula jetson_hardware_launch.launch.py slam_mode:=localization
```

Verify localisation loaded correctly:
```bash
# Should print: String value is: localization
ros2 param get /slam_toolbox mode

# Map should appear fully drawn before you drive anywhere
ros2 topic echo /map --once
```

### 8.3 Sending autonomous navigation goals

With Nav2 running and a map loaded, set a goal from RViz:

1. In RViz, select the **2D Goal Pose** tool (arrow icon in toolbar)
2. Click and drag on the map to set a target position and heading
3. Nav2 will plan a path and drive URSULA to the goal

From the command line:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}'
```

The joystick always overrides Nav2 (priority 100 vs 10). Pressing LB cancels autonomous motion immediately.

---

## 9. Object Detection

URSULA uses a YOLOv4-tiny model running on the OAK-D's onboard VPU. Detections are published to `/oak/nn/detections` and converted to labelled map markers by `detection_mapper.py`.

### 9.1 Viewing detections

**In Foxglove:**
- Add a **3D panel** and subscribe to `/ursula/detection_markers` to see coloured cylinders on the map at detection locations
- Add an **Image panel** and subscribe to `/detection/image` to see the annotated camera feed with bounding boxes

**In RViz:**
- Add a **MarkerArray** display and set the topic to `/ursula/detection_markers`

### 9.2 Blob file location

The YOLOv4-tiny blob must be present in the depthai_examples resources directory:

```bash
ls ~/oak_camera_ws/src/depthai-ros/depthai_examples/resources/
# Should include: yolov4_tiny_coco_416x416_openvino_2021.4_6shave_bgr.blob
```

### 9.3 Disabling detection

If the camera or blob is unavailable:

```bash
ros2 launch ursula jetson_hardware_launch.launch.py detection:=false camera:=false
```

---

## 10. Configuration Reference

### serial_bridge_params.yaml

| Parameter | Value | Description |
|---|---|---|
| `serial_port` | `/dev/ttyACM0` | Arduino USB port |
| `baud_rate` | `115200` | Must match Arduino firmware |
| `wheel_base` | `0.56` | Distance between left and right wheels (m) |
| `estop_button` | `4` | Controller button index for LB |
| `turn_multiplier` | `4.5` | Angular velocity amplification for skid-steer |
| `max_linear_vel` | `0.08` | Hard cap on linear velocity (m/s) |
| `max_angular_vel` | `0.08` | Hard cap on angular velocity (rad/s) |
| `min_angular_vel` | `0.05` | Minimum angular velocity to overcome motor deadband |
| `angular_deadband` | `0.005` | Commands below this threshold are treated as zero |

### controller_teleop.yaml

The PS4 right stick axis is `axes[3]`. The Logitech F310 uses `axes[0]`. To switch to F310, change `axis_angular.yaw` from `3` to `0` in `config/controller_teleop.yaml`.

### Skid-steer tuning

See `config/steering_tuning.md` for a full explanation of how to tune turning behaviour in both simulation and on real hardware. The key parameters are `turn_multiplier` (serial bridge), `scale_angular.yaw` (teleop config), and `mu2` (Gazebo friction coefficient).

---

## 11. Recovery Procedure

Complete recovery steps from a fresh Ubuntu 22.04 installation:

1. Install Ubuntu 22.04
2. Install ROS 2 Humble (section 4.1)
3. Clone the URSULA repository (section 4.5)
4. Install all ROS 2 dependencies (sections 4.2 – 4.3)
5. Install the OAK-D workspace on the Jetson (section 4.4)
6. Configure shell setup and create maps directory (sections 4.6 – 4.7)
7. Install and configure Tailscale (section 5.2)
8. Configure Fast DDS and update `networking/fastdds_super_client.xml` with new Tailscale IPs (section 5.2)
9. Flash Arduino firmware (section 4.8)
10. Restore maps from backup to `~/ros2_ws/maps`
11. Verify Local Mode (section 5.1)
12. Verify Remote Mode (section 5.2)

---

## 12. Troubleshooting

### LiDAR times out on startup

```
[rplidar_node] Error, operation time out. SL_RESULT_OPERATION_TIMEOUT!
```

The RPLIDAR A3 requires 256000 baud, not the default 115200. Confirm `serial_baudrate: 256000` is set in `jetson_hardware_launch.launch.py`. Also check the USB cable and port:

```bash
ls -la /dev/ttyUSB*
sudo dmesg | tail -20
```

### Arduino not found

```
Failed to connect to Arduino: [Errno 2] No such file or directory: '/dev/ttyACM0'
```

Check the port exists and add your user to the `dialout` group:

```bash
ls -la /dev/ttyACM*
sudo usermod -aG dialout $USER
# Log out and back in for the group change to take effect
```

### No topics visible on laptop

First confirm both machines have identical `ROS_DOMAIN_ID` values and are using the same network mode:

```bash
echo $ROS_DOMAIN_ID   # Must match on both machines
echo $RMW_IMPLEMENTATION  # Must match on both machines
```

Check network connectivity:
```bash
ping <jetson-ip>
ros2 topic list       # Should show Jetson's topics if DDS discovery is working
```

In remote mode, confirm the Fast DDS Discovery Server is running on the laptop before sourcing `remote_mode.sh` on either machine.

### FastDDS instability in remote mode

If topics drop out intermittently during remote operation, FastDDS may be binding to the wrong network interface. Confirm that `networking/fastdds_super_client.xml` uses `interfaceWhiteList` to restrict traffic to the Tailscale interface rather than `0.0.0.0`.

### SLAM map is distorted or drifting

RF2O odometry depends on the LiDAR scan being stable. Common causes of drift:

- Robot moving too fast — reduce `max_linear_vel` in `serial_bridge_params.yaml`
- Featureless environment (long empty corridor / flat outdoor area) — SLAM has nothing to match against; add visual landmarks or slow down
- LiDAR scan rate mismatch — confirm `freq: 15.0` in the RF2O node parameters matches the RPLIDAR A3 output rate

### Gazebo robot does not move

Confirm twist_mux is running and the joystick is publishing to `/cmd_vel_joy` (not `/cmd_vel` directly):

```bash
ros2 topic echo /cmd_vel_joy    # Should show values when joystick is moved
ros2 topic echo /cmd_vel        # Should mirror /cmd_vel_joy output
ros2 node list | grep twist_mux # twist_mux must be running
```

### OAK-D camera not detected

```bash
# Check USB3 connection
lsusb | grep MyriadX

# Check depthai permissions
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```
