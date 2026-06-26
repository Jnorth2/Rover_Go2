#!/usr/bin/env python3
"""
Relay bare-DDS Go2 topics into the ROS2 graph so zenoh-bridge-ros2dds
can discover and forward them.

The Go2 firmware runs plain CycloneDDS without publishing ros_discovery_info,
so zenoh-bridge-ros2dds never sees its publishers.  This node subscribes to
the Go2 topics as a normal ROS2 node (using rmw_cyclonedds_cpp on enP8p1s0),
which makes ros_discovery_info visible.  The bridge then creates Zenoh routes
for each publisher here and the laptop can receive them via rmw_zenoh_cpp.

Run with:
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  CYCLONEDDS_URI=file:///etc/zenoh/cyclonedds.xml
  ROS_DOMAIN_ID=0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from unitree_go.msg import (
    SportModeState,
    LowState,
    WirelessController,
    LidarState,
    HeightMap,
    IMUState,
)
from unitree_api.msg import Request, Response


# (topic_name, message_type, qos_reliability)
# RELIABLE matches the Go2's publisher QoS; BEST_EFFORT for high-rate streams.
RELAY_TOPICS = [
    # State
    ("/sportmodestate",              SportModeState,      "reliable"),
    ("/lf/sportmodestate",           SportModeState,      "reliable"),
    ("/lowstate",                    LowState,            "reliable"),
    ("/lf/lowstate",                 LowState,            "reliable"),
    # Controller
    ("/wirelesscontroller",          WirelessController,  "reliable"),
    ("/wirelesscontroller_unprocessed", WirelessController, "reliable"),
    # LiDAR / IMU
    ("/utlidar/imu",                 Imu,                 "best_effort"),
    ("/utlidar/lidar_state",         LidarState,          "reliable"),
    # API responses (Go2 → laptop)
    ("/api/sport/response",          Response,            "reliable"),
    ("/api/motion_switcher/response",Response,            "reliable"),
    ("/api/robot_state/response",    Response,            "reliable"),
]


def make_qos(reliability: str) -> QoSProfile:
    rel = (
        ReliabilityPolicy.RELIABLE
        if reliability == "reliable"
        else ReliabilityPolicy.BEST_EFFORT
    )
    return QoSProfile(
        reliability=rel,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class Go2Relay(Node):
    def __init__(self):
        super().__init__("go2_relay")
        self._relay_pubs: dict = {}

        for topic, msg_type, reliability in RELAY_TOPICS:
            qos = make_qos(reliability)
            pub = self.create_publisher(msg_type, topic, qos)
            self._relay_pubs[topic] = pub
            self.create_subscription(
                msg_type,
                topic,
                lambda msg, t=topic: self._relay_pubs[t].publish(msg),
                qos,
            )
            self.get_logger().info(f"Relaying {topic} [{msg_type.__name__}]")


def main():
    rclpy.init()
    node = Go2Relay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
