# Groundstation Zenoh Setup

The groundstation runs a Zenoh router that federates with the Jetson Orin's
router. ROS 2 nodes on the laptop use `rmw_zenoh_cpp` as their RMW and connect
as Zenoh clients to the local router. No separate ROS2DDS bridge is needed on
the groundstation.

```text
Go2 CycloneDDS
  -> Jetson zenoh-bridge-ros2dds (CycloneDDS -> Zenoh client)
  -> Jetson Zenoh router (192.168.123.99:7447)
  -> Groundstation Zenoh router (connects out to Jetson, listens on :7447)
  -> Groundstation ROS 2 nodes (rmw_zenoh_cpp -> localhost:7447)
```

The Jetson's `zenoh-bridge-ros2dds` is the only bridge in the chain. It reads
Go2 DDS topics via the `enP8p1s0` interface and injects them into the Zenoh
mesh. The groundstation sees those topics natively through `rmw_zenoh_cpp`.

## Prerequisites

- ROS 2 Jazzy at `/opt/ros/jazzy`
- `ros-jazzy-rmw-zenoh-cpp` (provides `rmw_zenohd`)
- TCP access to the Jetson at `192.168.123.99:7447`
- The same custom ROS message packages built on both machines

Install:

```bash
sudo apt update
sudo apt install ros-jazzy-rmw-zenoh-cpp
```

Verify network access before starting:

```bash
ping -c 3 192.168.123.99
nc -vz 192.168.123.99 7447
```

## Option 1: automatic startup with systemd

From this directory:

```bash
sudo install -d -m 0755 /etc/zenoh
sudo install -m 0644 router.json5 /etc/zenoh/groundstation-router.json5
sudo install -m 0644 zenoh-groundstation-router.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now zenoh-groundstation-router.service
```

Check status and logs:

```bash
systemctl status zenoh-groundstation-router.service
journalctl -u zenoh-groundstation-router.service -f
```

After editing `router.json5`:

```bash
sudo install -m 0644 router.json5 /etc/zenoh/groundstation-router.json5
sudo systemctl restart zenoh-groundstation-router.service
```

Disable and stop:

```bash
sudo systemctl disable --now zenoh-groundstation-router.service
sudo rm /etc/systemd/system/zenoh-groundstation-router.service
sudo rm /etc/zenoh/groundstation-router.json5
sudo systemctl daemon-reload
```

## Option 2: run manually

Start the router in a dedicated terminal:

```bash
./run-zenoh-router.sh
```

Press `Ctrl+C` to stop.

## Use ROS 2 on the groundstation

With the router running, open a new terminal and source the environment:

```bash
cd /home/jn2-alt/Rover_Go2/groundstation
source ./ros2-local.sh
ros2 daemon stop
ros2 topic list
```

The `ros2 daemon stop` is important if the daemon was previously started under a
different RMW. Use the same sourced environment for all commands:

```bash
ros2 topic echo /utlidar/imu --once
ros2 topic info /utlidar/imu --verbose
```

Source your workspace after `ros2-local.sh` when custom Go2 messages are needed:

```bash
source ./ros2-local.sh
source /path/to/your_workspace/install/setup.bash
```

## Troubleshooting

Confirm the router is connected to the Jetson:

```bash
sudo ss -ntp | grep ':7447'
```

You should see one connection to `192.168.123.99:7447` (the Jetson) and the
rmw_zenohd process listening on `0.0.0.0:7447`.

If `ros2 topic list` only shows `/parameter_events` and `/rosout`, confirm the
Jetson's `zenoh-bridge-ros2dds` is running and that it can see Go2 DDS topics
via CycloneDDS. A fresh `ros2 daemon stop` before listing is usually required
after sourcing a new environment.

## Joystick teleop

Plug in a PS4 or DualSense controller via USB or Bluetooth before launching.

```bash
# Verify the kernel sees the controller
ls /dev/input/js*

# Start the joy node (auto-detects the first controller)
source /opt/ros/jazzy/setup.bash
ros2 launch launch/joy_launch.py
```

The node publishes to `/joy`. The `go2_joy_teleop` node on the Orin subscribes
and translates stick positions into `/api/sport/request` sport API commands.

Controller axes are printed by running `ros2 topic echo /joy` and moving each
stick. If the Go2 moves in the wrong direction negate the relevant parameter on
the Orin launch side (`axis_vx`, `axis_vy`, `axis_vyaw`).

## Files

- `router.json5` — groundstation router; connects to Jetson, listens on :7447
- `client.json5` — Zenoh client config for `rmw_zenoh_cpp` ROS 2 nodes
- `zenoh-groundstation-router.service` — systemd unit for automatic router startup
- `run-zenoh-router.sh` — manual router launcher
- `ros2-local.sh` — sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and `ZENOH_ROUTER_CONFIG_URI`
- `launch/joy_launch.py` — starts `joy_node` for PS4 / DualSense controller input

### Legacy: groundstation ROS2DDS bridge

The files `bridge.json5`, `run-zenoh-bridge.sh`, and
`zenoh-groundstation-bridge.service` implement a double-bridge topology where
the groundstation also runs `zenoh-bridge-ros2dds` and local nodes use
CycloneDDS. This is no longer the recommended approach; use `rmw_zenoh_cpp`
directly as described above.
