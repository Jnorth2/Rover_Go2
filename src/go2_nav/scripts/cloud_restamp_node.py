#!/usr/bin/env python3
"""
Re-timestamps point cloud messages to the current ROS clock.

The Go2's UTlidar publishes /utlidar/cloud and /utlidar/cloud_deskewed with the
Go2's internal system time, which is offset from the Jetson's clock (observed
~113 s). RTAB-Map's approx_sync cannot match the cloud (Go2 time) with the EKF
odometry output (Jetson time). This node re-stamps to Jetson time so all inputs
share the same clock reference.

The point data itself is captured at the correct time relative to the Go2's
hardware — only the header timestamp is wrong due to the clock offset.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class CloudRestampNode(Node):
    def __init__(self):
        super().__init__('cloud_restamp_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub = self.create_publisher(PointCloud2, 'cloud_out', qos)
        self._sub = self.create_subscription(
            PointCloud2, 'cloud_in', self._cb, qos,
        )

    def _cb(self, msg: PointCloud2):
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CloudRestampNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
