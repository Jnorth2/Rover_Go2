#!/usr/bin/env python3
"""
Re-timestamps point cloud messages to the current ROS clock.

The Go2's UTlidar publishes /utlidar/cloud_deskewed with the Go2's internal system
time, which is offset from the Jetson's clock (observed ~113 s). RTAB-Map's
approx_sync cannot match the cloud (Go2 time) with the EKF odometry output
(Jetson time). This node re-stamps to Jetson time so all inputs share the same
clock reference.

QoS: UTlidar publishes BEST_EFFORT, so we subscribe BEST_EFFORT. RTAB-Map
subscribes RELIABLE, so we publish RELIABLE. A RELIABLE publisher can serve
both RELIABLE and BEST_EFFORT subscribers, so this is the correct bridge.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class CloudRestampNode(Node):
    def __init__(self):
        super().__init__('cloud_restamp_node')

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub = self.create_publisher(PointCloud2, 'cloud_out', pub_qos)
        self._sub = self.create_subscription(
            PointCloud2, 'cloud_in', self._cb, sub_qos,
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
