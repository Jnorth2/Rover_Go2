# Zenoh Bridge Investigation Notes

## Problem

`ros2 topic list` with `RMW_IMPLEMENTATION=rmw_zenoh_cpp` on the laptop (or Jetson)
shows only `/parameter_events` and `/rosout` — no Go2 topics.

## Architecture (what IS working)

```
Go2 firmware (bare CycloneDDS, no ros_discovery_info)
  -> go2-relay.service   (rmw_cyclonedds_cpp on enP8p1s0, rebroadcasts as proper ROS2 node)
  -> zenoh-bridge.service (zenoh-bridge-ros2dds v1.9.0 client, connected to localhost:7447)
     creates Zenoh routes: rt/sportmodestate, rt/lowstate, etc.
  -> Jetson rmw_zenohd router (port 7447, rmw_zenoh_cpp 0.2.9, zenoh-c 1.6.2)
  -> (federation) Laptop rmw_zenohd router (port 7447)
  -> Laptop ROS2 peer sessions (rmw_zenoh_cpp 0.2.9, connects to localhost:7447)
```

The relay correctly republishes Go2 DDS topics as a proper ROS2 node.
The bridge confirms routes are created (`Route Publisher (ROS:/sportmodestate -> Zenoh:sportmodestate) created`).

## Root Cause: Liveliness Token Format Mismatch

rmw_zenoh_cpp uses **liveliness tokens** for topic discovery (`ros2 topic list`).

### What rmw_zenoh_cpp (0.2.9 AND 0.6.6) subscribes to:
```
@ros2_lv/0/**
```
(confirmed by: `RUST_LOG=zenoh::net::routing::dispatcher::interests=debug ros2 topic list`)

### What zenoh-bridge-ros2dds v1.x creates for bridged topics:
```
@/<bridge_zid>/@ros2_lv/MS/sportmodestate/unitree_go§msg§SportModeState/...
@/<bridge_zid>/@ros2_lv/MS/lowstate/unitree_go§msg§LowState/...
```
(confirmed by: `RUST_LOG=zenoh=debug timeout 8 /usr/local/bin/zenoh-bridge-ros2dds --config /etc/zenoh/bridge.json5`)

### Why they don't match:
- `@ros2_lv/0/**` matches keys starting with `@ros2_lv/0/`
- `@/<zid>/@ros2_lv/...` does NOT start with `@ros2_lv/0/`
- These two namespaces are incompatible

## What Was Tested

| Test | Result |
|------|--------|
| `ros2 topic list` (Jazzy, laptop, via local router → Jetson federation) | ❌ 2 self-published topics only |
| `ros2 topic list` (Jazzy, laptop, direct peer to Jetson router) | ❌ same |
| `ros2 topic list` (Kilted 0.6.6, laptop, direct peer to Jetson router) | ❌ same |
| `ros2 topic list` (Jazzy, Jetson, connecting to localhost:7447) | ❌ same |
| Bridge v1.6.2 token format check | ❌ still uses `@/<zid>/@ros2_lv/...` |
| Bridge v1.9.0 token format check | ❌ uses `@/<zid>/@ros2_lv/...` |

All v1.x bridge releases (v1.0.4 through v1.9.0) use the new namespace format.

## Key Version Facts

| Component | Package Version | Zenoh Version |
|-----------|----------------|---------------|
| rmw_zenoh_cpp (Jazzy) | 0.2.9 | zenoh-c 1.6.2 |
| rmw_zenoh_cpp (Kilted) | 0.6.6 | zenoh-c 1.6.2 |
| zenoh-bridge-ros2dds (installed) | 1.9.0 | zenoh 1.9.0 |
| zenoh-bridge-ros2dds (tested) | 1.6.2 | zenoh 1.6.2 |

## Next Steps to Try (in priority order)

### Option A: Install ROS-packaged bridge (highest chance of working)

Check if `ros-jazzy-zenoh-bridge-ros2dds` or similar package exists in the ROS apt repo.
A bridge packaged alongside rmw_zenoh_cpp 0.2.9 would use the matching format.

```bash
# On Jetson:
apt-cache search zenoh
# OR
apt-cache search ros2dds
```

### Option B: Find a v0.x bridge release

The v1.x releases all use the new `@/<zid>/@ros2_lv/...` format. There may be
v0.x releases that use the old `@ros2_lv/0/<zid>/...` format compatible with
rmw_zenoh_cpp 0.2.9.

Check GitHub releases page 2+:
```
https://api.github.com/repos/eclipse-zenoh/zenoh-plugin-ros2dds/releases?per_page=20&page=2
```

### Option C: No-bridge architecture

Instead of using zenoh-bridge-ros2dds, run a second relay process that bridges
from the CycloneDDS local network to Zenoh directly using rmw_zenoh_cpp.

Architecture:
```
go2_relay (rmw_cyclonedds_cpp) publishes on ROS_DOMAIN_ID=0 CycloneDDS loopback
  -> zenoh_relay (rmw_zenoh_cpp, subscribes via ROS_LOCALHOST_ONLY=1 CycloneDDS loopback,
                  republishes via rmw_zenoh_cpp to Zenoh network)
```

This requires two separate Python nodes. The zenoh_relay would use
`RMW_IMPLEMENTATION=rmw_zenoh_cpp` and have full graph visibility.

Caveat: zenoh_relay can't subscribe to BOTH rmw_cyclonedds topics AND rmw_zenoh topics
in the same process. It needs rmw_cyclonedds to subscribe and rmw_zenoh to publish.
This means a 3-node chain or a transport bridge.

**SIMPLEST IMPLEMENTATION of Option C:**
Run go2_relay with `rmw_zenoh_cpp` directly, but change it to subscribe to Go2's
bare DDS topics. The problem is that `rmw_zenoh_cpp` won't discover Go2 bare DDS.

Therefore, split into TWO processes:
1. `go2_dds_relay.py` - `rmw_cyclonedds_cpp`, subscribes to Go2 DDS on `enP8p1s0`,
   republishes to LOCAL loopback (different domain or localhost only)
2. `go2_zenoh_relay.py` - `rmw_zenoh_cpp`, subscribes to local loopback topics,
   publishes to Zenoh network → laptop can discover via `@ros2_lv/0/**`

This avoids the bridge entirely. Services would start `go2_dds_relay` first,
then `go2_zenoh_relay`.

## Environment Variables for rmw_zenoh_cpp (for reference)

- `ZENOH_SESSION_CONFIG_URI` - overrides the Zenoh session config for ROS2 nodes
- `ZENOH_ROUTER_CONFIG_URI` - overrides the config used by `rmw_zenohd` (router daemon)
- `ZENOH_CONFIG_OVERRIDE` - per-key JSON config overrides

The `ros2-local.sh` script was previously setting `ZENOH_ROUTER_CONFIG_URI` which
only affects the router daemon, NOT the node sessions. It should set
`ZENOH_SESSION_CONFIG_URI` instead (or both).

## Other Confirmed Facts

- The Jetson router (ZID `8ca168b0`) is running correctly
- The bridge v1.9.0 IS creating Zenoh routes for all 11 relay topics
- Both Jazzy (0.2.9) and Kilted (0.6.6) use the SAME zenoh-c 1.6.2
- The laptop router federation with Jetson works at the TCP layer
- Liveliness tokens from the bridge DON'T cross the federation
  (because the format mismatch means rmw_zenoh_cpp nodes don't subscribe to them)
