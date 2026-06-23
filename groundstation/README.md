# Groundstation Zenoh and ROS2DDS bridge

The groundstation uses a paired ROS2DDS bridge. This is the matching endpoint
for `zenoh-bridge-ros2dds` on the Jetson Orin:

```text
Go2 CycloneDDS
  -> Jetson ROS2DDS bridge
  -> Jetson Zenoh router (192.168.123.99:7447)
  -> groundstation Zenoh router
  -> groundstation ROS2DDS bridge
  -> local CycloneDDS nodes
```

Local ROS 2 nodes use CycloneDDS with `ROS_LOCALHOST_ONLY=1`. DDS discovery
therefore remains on the laptop; only Zenoh traffic crosses the network link.
Do not use `rmw_zenoh_cpp` for the groundstation ROS nodes in this topology.

## Prerequisites

- ROS 2 Humble at `/opt/ros/humble`
- `ros-humble-rmw-zenoh-cpp` (provides the local `rmw_zenohd` router)
- `ros-humble-rmw-cyclonedds-cpp` (used by groundstation ROS nodes)
- `zenoh-bridge-ros2dds` version `1.9.0`, matching the Jetson bridge
- TCP access to the Jetson at `192.168.123.99:7447`
- The same custom ROS message packages on both machines

Install the ROS packages:

```bash
sudo apt update
sudo apt install ros-humble-rmw-zenoh-cpp ros-humble-rmw-cyclonedds-cpp unzip
```

Install the official x86_64 Linux ROS2DDS bridge package:

```bash
cd /tmp
curl -fLO https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/1.9.0/zenoh-plugin-ros2dds-1.9.0-x86_64-unknown-linux-gnu-debian.zip
unzip zenoh-plugin-ros2dds-1.9.0-x86_64-unknown-linux-gnu-debian.zip
sudo apt install ./zenoh-bridge-ros2dds_1.9.0_amd64.deb
/usr/bin/zenoh-bridge-ros2dds --version
```

The release package is from the official [Eclipse Zenoh v1.9.0
release](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/tag/1.9.0).

Check network access before starting:

```bash
ping -c 3 192.168.123.99
nc -vz 192.168.123.99 7447
```

## Option 1: automatic startup with systemd

From this directory, install both configurations and services:

```bash
sudo install -d -m 0755 /etc/zenoh
sudo install -m 0644 router.json5 /etc/zenoh/groundstation-router.json5
sudo install -m 0644 bridge.json5 /etc/zenoh/groundstation-bridge.json5
sudo install -m 0644 zenoh-groundstation-router.service /etc/systemd/system/
sudo install -m 0644 zenoh-groundstation-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now zenoh-groundstation-router.service
sudo systemctl enable --now zenoh-groundstation-bridge.service
```

The bridge unit requires the router unit, so systemd starts them in the correct
order. Check their status and logs with:

```bash
systemctl status zenoh-groundstation-router.service zenoh-groundstation-bridge.service
journalctl -u zenoh-groundstation-router.service -u zenoh-groundstation-bridge.service -f
```

After editing either JSON5 file, reinstall it and restart both services:

```bash
sudo install -m 0644 router.json5 /etc/zenoh/groundstation-router.json5
sudo install -m 0644 bridge.json5 /etc/zenoh/groundstation-bridge.json5
sudo systemctl restart zenoh-groundstation-router.service zenoh-groundstation-bridge.service
```

Disable automatic startup and stop both processes:

```bash
sudo systemctl disable --now zenoh-groundstation-bridge.service
sudo systemctl disable --now zenoh-groundstation-router.service
```

## Option 2: run manually

Stop the systemd services first to avoid duplicate processes:

```bash
sudo systemctl stop zenoh-groundstation-bridge.service zenoh-groundstation-router.service
```

Start the router in terminal 1:

```bash
./run-zenoh-router.sh
```

Start the bridge in terminal 2:

```bash
./run-zenoh-bridge.sh
```

Press `Ctrl+C` in each terminal to stop the manual processes.

## Use ROS 2 on the groundstation

With the router and bridge running, open a third terminal:

```bash
cd /home/jn2/Rover_Go2/groundstation
source ./ros2-local.sh
ros2 daemon stop
ros2 topic list
```

The daemon restart is important if `ros2` was previously run using another RMW
implementation or discovery configuration. Use the same sourced environment for
commands such as:

```bash
ros2 topic info /utlidar/imu --verbose
ros2 topic echo /utlidar/imu --once
```

Source your workspace after `ros2-local.sh` when custom Go2 messages are needed:

```bash
source ./ros2-local.sh
source /path/to/your_workspace/install/setup.bash
```

## Troubleshooting

Confirm all three TCP sessions on the groundstation:

```bash
sudo ss -ntp | grep ':7447'
```

You should see the groundstation router connected to `192.168.123.99:7447`
and the local bridge connected to `127.0.0.1:7447`.

If only `/parameter_events` and `/rosout` appear, first verify that the Jetson
can see native Go2 topics using CycloneDDS. A paired bridge cannot transport
topics that the Jetson bridge has not discovered.

Remove the installed groundstation units and configurations with:

```bash
sudo systemctl disable --now zenoh-groundstation-bridge.service zenoh-groundstation-router.service
sudo rm /etc/systemd/system/zenoh-groundstation-bridge.service
sudo rm /etc/systemd/system/zenoh-groundstation-router.service
sudo rm /etc/zenoh/groundstation-bridge.json5
sudo rm /etc/zenoh/groundstation-router.json5
sudo systemctl daemon-reload
```

## Files

- `router.json5`: connects the groundstation router to the Jetson
- `bridge.json5`: connects the local ROS2DDS bridge to the local router
- `zenoh-groundstation-router.service`: automatic router startup
- `zenoh-groundstation-bridge.service`: automatic bridge startup
- `run-zenoh-router.sh`: manual router launcher
- `run-zenoh-bridge.sh`: manual bridge launcher
- `ros2-local.sh`: environment for local Humble CycloneDDS commands
