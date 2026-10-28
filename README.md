# Rover_Go2

ROS 2 + Zenoh bridge for the Unitree Go2 robot, enabling wireless topic access
from a groundstation laptop via a Zenoh router-to-router link.

## Architecture

```
Go2 firmware (bare CycloneDDS)
  -> [go2-relay.service]         # makes Go2 topics visible as ROS2 nodes
  -> [zenoh-bridge.service]      # bridges ROS2 topics to Zenoh
  -> Jetson Zenoh router (:7447)
  -> (network link)
  -> Groundstation Zenoh router  (connects to Jetson, listens on :7447)
  -> Groundstation ROS2 nodes    (rmw_zenoh_cpp -> localhost:7447)
```

The Go2's firmware publishes topics as bare DDS without ROS2 discovery info
(`ros_discovery_info`). `zenoh-bridge-ros2dds` only bridges proper ROS2 nodes,
so `go2-relay.service` subscribes to the Go2 topics via CycloneDDS and
republishes them as a normal ROS2 node. The bridge then discovers and routes
those publishers over Zenoh.

## GO2 Setup
The go2 is jailbroken and updated to 1.11. SSH can be achieved for wired connections as specified below:
```bash
ssh root@192.168.123.161
```

## Jetson Orin setup

### Prerequisites

- ROS 2 Jazzy at `/opt/ros/jazzy`
- `unitree_ws` built at `/home/northjar/unitree_ws` (provides `unitree_go`,
  `unitree_api`, `unitree_hg` message types)
- `ros-jazzy-rmw-cyclonedds-cpp` and `ros-jazzy-rmw-zenoh-cpp` installed
- `zenoh-bridge-ros2dds` binary at `/usr/local/bin/zenoh-bridge-ros2dds`

### Install the Orin services

From the repo root, copy configs and services to the Jetson:

```bash
sudo install -d -m 0755 /etc/zenoh

# Zenoh configs
sudo install -m 0644 orin_systemd/router.json5   /etc/zenoh/router.json5
sudo install -m 0644 orin_systemd/bridge.json5   /etc/zenoh/bridge.json5
sudo install -m 0644 orin_systemd/cyclonedds.xml /etc/zenoh/cyclonedds.xml

# Relay script
sudo install -m 0755 orin_systemd/go2_relay.py   /etc/zenoh/go2_relay.py

# Systemd units
sudo install -m 0644 orin_systemd/zenoh-router.service  /etc/systemd/system/
sudo install -m 0644 orin_systemd/zenoh-bridge.service  /etc/systemd/system/
sudo install -m 0644 orin_systemd/go2-relay.service     /etc/systemd/system/

sudo systemctl daemon-reload
sudo systemctl enable --now zenoh-router.service
sudo systemctl enable --now zenoh-bridge.service
sudo systemctl enable --now go2-relay.service
```

### Check status

```bash
systemctl status zenoh-router.service zenoh-bridge.service go2-relay.service
journalctl -u zenoh-router.service -u zenoh-bridge.service -u go2-relay.service -f
```

The `go2-relay` log should show lines like:
```
Relaying /sportmodestate [SportModeState]
Relaying /lowstate [LowState]
...
```

The `zenoh-bridge` log should then show routes being created for those topics:
```
Discovered ROS Node /go2_relay
Route Publisher (ROS:/sportmodestate -> Zenoh:sportmodestate) created
```

### After editing configs

```bash
sudo install -m 0644 orin_systemd/bridge.json5 /etc/zenoh/bridge.json5
sudo install -m 0755 orin_systemd/go2_relay.py /etc/zenoh/go2_relay.py
sudo systemctl restart zenoh-bridge.service go2-relay.service
```

### Disable and remove

```bash
sudo systemctl disable --now zenoh-router.service zenoh-bridge.service go2-relay.service
sudo rm /etc/systemd/system/zenoh-router.service
sudo rm /etc/systemd/system/zenoh-bridge.service
sudo rm /etc/systemd/system/go2-relay.service
sudo rm /etc/zenoh/go2_relay.py
sudo systemctl daemon-reload
```

## Joystick control (PS4 / DualSense)

A PlayStation controller connected to the groundstation drives the Go2 over the
existing Zenoh ROS2 link.

```
Groundstation joy_node  →  /joy  →  (Zenoh)  →  Orin go2_joy_teleop  →  /api/sport/request
                                     Nav2  →  /cmd_vel  ↗
```

### Controller mapping

| Input | Action |
|-------|--------|
| Left stick — up / down | Forward / backward |
| Left stick — left / right | Strafe left / right |
| Right stick — left / right | Turn left / right |
| Right stick — up / down | Tilt body (look up / down) |
| **□ Square** | Toggle sit / stand |
| **△ Triangle** | Allow / block Nav2 `/cmd_vel` |

### Running joystick control

**Groundstation** — plug in the controller, then:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch /home/jn2-alt/Rover_Go2/groundstation/launch/joy_launch.py
```

**Orin** — in a sourced workspace terminal:

```bash
ros2 launch go2_control go2_joy_launch.py
```

Both sides must see each other's topics (Zenoh routers running, sourced with
`rmw_zenoh_cpp`).

### Tuning

All limits and axis assignments are ROS2 parameters. Override at launch:

```bash
ros2 launch go2_control go2_joy_launch.py max_linear:=1.2 max_angular:=1.5
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `axis_vx` | `1` | Axis index for forward / backward |
| `axis_vy` | `0` | Axis index for strafe |
| `axis_vyaw` | `2` | Axis index for yaw (turn) |
| `axis_pitch` | `3` | Axis index for body pitch |
| `btn_sit_stand` | `2` | Button index for sit / stand toggle |
| `btn_nav_toggle` | `3` | Button index for allowing / blocking Nav2 |
| `nav_enabled` | `true` | Whether Nav2 may drive at startup |
| `max_linear` | `0.8` | Max linear speed (m/s), joystick and Nav2 |
| `max_angular` | `1.0` | Max yaw rate (rad/s), joystick and Nav2 |
| `max_pitch` | `0.3` | Body pitch at full stick (rad) |
| `deadzone` | `0.08` | Stick deadzone |
| `joy_timeout` | `0.5` | `/joy` older than this (s) counts as sticks centred |
| `cmd_vel_timeout` | `0.5` | `/cmd_vel` older than this (s) is ignored |
| `nav_min_linear` | `0.15` | Nav2 walking speeds below this (m/s) are raised to it |
| `nav_min_angular` | `0.3` | Nav2 turn-in-place rates below this (rad/s) are raised to it |
| `nav_deadband_linear` | `0.03` | Nav2 speeds below this (m/s) are treated as stop |
| `nav_deadband_angular` | `0.05` | Nav2 yaw rates below this (rad/s) are treated as stop |

Axis indices follow the `joy_linux` driver defaults for DS4 / DualSense.
If your controller maps differently run `ros2 topic echo /joy` and check the
indices while moving each stick.

## Navigation (RTAB-Map + Nav2)

Two launch files bring up everything for mapping and autonomous navigation.

**Go2 / Jetson** — camera, EKF odometry, robot description, RTAB-Map SLAM,
lidar obstacle segmentation, Nav2 and the joystick / `cmd_vel` teleop:

```bash
ros2 launch go2_nav navigation.launch.py
```

**Groundstation** — RViz (map, costmaps, plan, Nav2 panel) and the joystick:

```bash
ros2 launch go2_nav groundstation_navigation.launch.py
```

The camera feed is not part of the groundstation launch; view it with
`groundstation/camera_gstreamer.sh`.

The groundstation launch also serves a **campus map** at http://localhost:8080
(Leaflet, OpenStreetMap or satellite, centered on Merryfield Hall). The robot and
its trail come from localization (TF `map → base_link`). GPS is only used to place
the map frame on the earth: fixes (±0.3 m or better) are paired with the
localized antenna position (`map → gps`) and a rotation + offset is fitted once
the robot has covered ~3 m. Disable with `use_map_ui:=false`, or run it alone with
`ros2 run go2_nav go2_map_ui.py`.

Each run starts a **new map** (`~/.ros/rtabmap.db` is deleted on startup). To keep
mapping into the previous database pass `delete_db_on_start:=false`. The same
default applies to `mapping.launch.py`.

### Driving

1. Press **□** twice (sit, then stand). The teleop only sends motion commands
   after its own stand-up / balance-stand sequence.
2. Drive with the sticks to build some map, or straight away use **Nav2 Goal**
   (or **2D Goal Pose**) in RViz. Goals may be in unexplored (unknown) space.
3. Any stick input overrides Nav2 while held. **△** blocks / re-allows Nav2
   commands entirely (the robot stops when blocked). If `/joy` stops arriving
   (link loss), the sticks count as centred.

### Data flow

```
/utlidar/cloud → cloud_restamp_node → /utlidar/cloud_restamped
    ├→ rtabmap (SLAM) → /map, map→odom TF
    └→ obstacles_detection → /utlidar/obstacles, /utlidar/ground → costmaps, collision_monitor
EKF → /odometry/filtered, odom→base_link TF
controller_server → cmd_vel_nav → velocity_smoother → cmd_vel_smoothed
    → collision_monitor → /cmd_vel → go2_joy_teleop → /api/sport/request (Move)
```

There is no AMCL or map_server: RTAB-Map localizes and its growing `/map` is the
global costmap's static layer. Nav2 starts `nav2_start_delay` (5 s) after the
rest so the TF tree is complete when the costmaps activate.

### Launch arguments (`navigation.launch.py`)

| Argument | Default | Description |
|----------|---------|-------------|
| `delete_db_on_start` | `true` | Fresh RTAB-Map database each run |
| `use_camera` | `true` | D435 driver, RGB-D in RTAB-Map and visual odometry in the EKF |
| `with_gps` | `false` | RTK GPS rover + RTK-fixed fixes as RTAB-Map position priors (`rtabmap_gps.yaml`) |
| `params_file` | `go2_nav/config/nav2_params.yaml` | Nav2 parameters |
| `nav2_start_delay` | `5.0` | Seconds before Nav2 starts |
| `lidar_min_range` | `0.5` | Drop lidar returns closer than this to the sensor (m); `0` disables |

Nav2 tuning lives in `src/go2_nav/config/nav2_params.yaml`: MPPI (omni motion
model, 0.5 m/s forward / 0.3 m/s lateral / 1.0 rad/s), footprint, inflation, and
collision monitor.

### Jetson dependencies

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-rtabmap-ros
```

After pulling, rebuild so the new launch and config files are installed:

```bash
colcon build --symlink-install --packages-select go2_nav go2_control
```

## Groundstation laptop setup

See [groundstation/README.md](groundstation/README.md) for full instructions.

### Quick start

```bash
# Install ROS2 Jazzy zenoh package if not already present
sudo apt install ros-jazzy-rmw-zenoh-cpp

# Terminal 1 — start groundstation router (connects to Jetson at 192.168.123.99:7447)
cd ~/Rover_Go2/groundstation
./run-zenoh-router.sh

# Terminal 2 — ROS2 commands
source ~/Rover_Go2/groundstation/ros2-local.sh
ros2 daemon stop
ros2 topic list
```

Ensure the laptop has the Unitree message packages built to subscribe to
custom Go2 topics (`unitree_go/msg/SportModeState`, etc.).

## Network

```
Jetson enP8p1s0  192.168.123.99   (Go2 internal Ethernet)
Laptop enx...    192.168.123.100  (direct Ethernet to Go2)
Zenoh port       7447             (TCP, both sides)
```

## Files

### orin_systemd/

| File | Purpose |
|------|---------|
| `router.json5` | Jetson Zenoh router config |
| `bridge.json5` | zenoh-bridge-ros2dds client config |
| `cyclonedds.xml` | CycloneDDS interface config (enP8p1s0) |
| `go2_relay.py` | Relay: bare DDS → ROS2 node |
| `zenoh-router.service` | Systemd unit for the Zenoh router |
| `zenoh-bridge.service` | Systemd unit for zenoh-bridge-ros2dds |
| `go2-relay.service` | Systemd unit for the Go2 relay |

### groundstation/

| File | Purpose |
|------|---------|
| `router.json5` | Groundstation Zenoh router config |
| `client.json5` | Zenoh client config for rmw_zenoh_cpp nodes |
| `zenoh-groundstation-router.service` | Systemd unit for groundstation router |
| `run-zenoh-router.sh` | Manual router launcher |
| `ros2-local.sh` | Sets RMW_IMPLEMENTATION=rmw_zenoh_cpp + ZENOH_ROUTER_CONFIG_URI |
| `launch/joy_launch.py` | Starts joy_node for PS4 / DualSense controller input |

### src/go2_control/

| File | Purpose |
|------|---------|
| `scripts/joy_teleop.py` | Translates `/joy` messages to Go2 sport API commands |
| `launch/go2_joy_launch.py` | Launches `joy_teleop.py` with default parameters |

### src/go2_nav/

| File | Purpose |
|------|---------|
| `launch/navigation.launch.py` | Go2: full stack (mapping + obstacles + Nav2 + teleop) |
| `launch/groundstation_navigation.launch.py` | Groundstation: RViz + joystick |
| `launch/mapping.launch.py` | Go2: camera, EKF, description, RTAB-Map only |
| `launch/groundstation_mapping.launch.py` | Groundstation: RViz for mapping only |
| `config/nav2_params.yaml` | Nav2 parameters |
| `config/rtabmap_mapping.yaml`, `config/rtabmap_camera.yaml` | RTAB-Map parameters |
| `config/groundstation_navigation.rviz` | RViz layout with Nav2 panel and goal tools |
