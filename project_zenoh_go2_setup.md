---
name: project-zenoh-go2-setup
description: Zenoh + CycloneDDS bridge setup for Unitree Go2 over wireless link (Bullet AC / Nanostation M5), including groundstation router-to-router config and Jetson systemd autolaunch
metadata:
  node_type: memory
  type: project
  originSessionId: 38013856-b262-4bf3-8947-ce964fe42253
---

## Network Topology

```
[ Go2 Robot ]
    └── Jetson Orin Nano
          ├── enP8p1s0 (192.168.123.99) ←→ Go2 internal CycloneDDS
          └── [Bullet AC interface] ←→ ))) wireless ))) ←→ Nanostation M5
                                                                └── Groundstation Laptop
```

**Why:** Go2 speaks CycloneDDS internally; Zenoh provides efficient unicast transport over the Ubiquiti Bullet AC ↔ Nanostation M5 wireless link without broadcasting DDS multicast over the air. Router-to-router topology is used so local nodes on each side connect to their own local router — more resilient to link drops and avoids each node opening its own TCP connection over WiFi.

**How to apply:** Use this setup when helping debug or extend the Zenoh/ROS2 bridge.

## Key facts
- ROS2 distro: Jazzy (at /opt/ros/jazzy)
- Go2 network interface on Jetson: `enP8p1s0` at 192.168.123.99/24
- Current WiFi (dev/test): `wlP1p1s0` at 192.168.8.226
- Bullet AC interface IP on Jetson: TBD (needed for groundstation router config)
- unitree_ws already built at /home/northjar/unitree_ws (has unitree_go, unitree_api, unitree_hg)
- ros-jazzy-rmw-cyclonedds-cpp: installed (0.10.5)
- ros-jazzy-rmw-zenoh-cpp: installed (0.2.9)
- zenoh-bridge-ros2dds: installed (binary)
- Configs: /etc/zenoh/router.json5 and /etc/zenoh/bridge.json5

## Jetson Commands (on Go2)

**Terminal 1 — Start Zenoh router** (listens on tcp/[::]:7447 for groundstation to connect):
```bash
ZENOH_ROUTER_CONFIG_URI=file:///etc/zenoh/router.json5 ros2 run rmw_zenoh_cpp rmw_zenohd
```

**Terminal 2 — Start CycloneDDS→Zenoh bridge** (reads Go2 ROS topics via enP8p1s0):
```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="enP8p1s0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'

RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 zenoh-bridge-ros2dds --config /etc/zenoh/bridge.json5
```

The bridge is a Zenoh client connecting to the local router at 127.0.0.1:7447 (per bridge.json5).

## Launching ROS2 nodes directly on the Jetson

The router and bridge are standalone processes — they don't set a default RMW for other nodes. A node's RMW depends on the env vars in its own shell:

**To see Go2's native DDS topics** (same domain/interface as the bridge):
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="enP8p1s0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'
ros2 run your_pkg your_node
```

**To talk only through the Zenoh mesh** (e.g. to reach the groundstation without touching Go2's DDS domain):
```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CONFIG_URI=file:///path/to/jetson-client.json5  # connect: tcp/127.0.0.1:7447
ros2 run your_pkg your_node
```
A fresh shell with no exports set falls back to the system default RMW (typically `rmw_fastrtps_cpp`), which sees neither Go2's DDS topics nor the Zenoh mesh.

## Jetson Autolaunch (systemd)

Drafted (not yet applied) in `~/zenoh_setup/systemd/`:
- `cyclonedds.xml` — the CYCLONEDDS_URI XML as a file (avoids quoting issues in systemd `Environment=`), installs to `/etc/zenoh/cyclonedds.xml`
- `zenoh-router.service` — runs `rmw_zenohd` as user `northjar`, `Restart=on-failure`
- `zenoh-bridge.service` — runs `zenoh-bridge-ros2dds`, `After=`/`Requires=zenoh-router.service` so it always starts second, `Restart=on-failure`

Apply with:
```bash
sudo cp /home/northjar/zenoh_setup/systemd/cyclonedds.xml /etc/zenoh/cyclonedds.xml
sudo cp /home/northjar/zenoh_setup/systemd/zenoh-router.service /etc/systemd/system/
sudo cp /home/northjar/zenoh_setup/systemd/zenoh-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now zenoh-router.service
sudo systemctl enable --now zenoh-bridge.service
```
Check status: `systemctl status zenoh-router.service zenoh-bridge.service`
Check logs: `journalctl -u zenoh-router.service -u zenoh-bridge.service -f`

## Groundstation Laptop Commands

**Step 1 — Create router config** (`~/zenoh/router.json5`):
```json5
{
  mode: "router",
  connect: {
    endpoints: ["tcp/<JETSON_BULLET_IP>:7447"]  // Jetson's IP on Bullet AC side — TBD
  },
  listen: {
    endpoints: ["tcp/[::]:7447"]
  },
  scouting: {
    multicast: { enabled: false }
  }
}
```

**Step 2 — Create client config** (`~/zenoh/client.json5`):
```json5
{
  mode: "client",
  connect: {
    endpoints: ["tcp/127.0.0.1:7447"]
  },
  scouting: {
    multicast: { enabled: false }
  }
}
```

**Terminal 1 — Start groundstation Zenoh router:**
```bash
ZENOH_ROUTER_CONFIG_URI=file:///home/you/zenoh/router.json5 ros2 run rmw_zenoh_cpp rmw_zenohd
```

**Terminal 2+ — Run ROS2 nodes:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/your_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CONFIG_URI=file:///home/you/zenoh/client.json5
ros2 topic list
```

## Data Flow

```
Go2 DDS topics
  → [enP8p1s0] → zenoh-bridge-ros2dds (CycloneDDS client)
  → Zenoh router on Jetson (:7447)
  → Bullet AC ))) Nanostation M5
  → Zenoh router on groundstation (:7447)
  → ROS2 nodes (rmw_zenoh_cpp client → localhost:7447)
```

## Config Summary
- router.json5 (Jetson): mode=router, listens tcp/[::]:7447, multicast disabled
- bridge.json5 (Jetson): mode=client, connects to localhost:7447, ros2dds plugin domain 0
- router.json5 (groundstation): mode=router, connects to Jetson Bullet AC IP:7447, listens on :7447
- client.json5 (groundstation): mode=client, connects to localhost:7447

## Outstanding
- Confirm Jetson's IP on the Bullet AC-facing interface — plug into groundstation router.json5 connect endpoint
- Apply the drafted systemd units on the Jetson (currently just files in `~/zenoh_setup/systemd/`, not yet copied to `/etc/systemd/system/` or enabled)
