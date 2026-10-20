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

Also drops returns closer than min_range to the sensor. The UTlidar produces
spurious points 5-25 cm above the floor just ahead of the head (within ~0.5 m of
the sensor), which the costmaps mark as an obstacle touching the footprint.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class CloudRestampNode(Node):
    def __init__(self):
        super().__init__('cloud_restamp_node')

        self.declare_parameter('min_range', 0.5)  # m from the sensor; 0 disables

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
        min_range = self.get_parameter('min_range').value
        if min_range > 0.0 and msg.width * msg.height > 0:
            self._filter_min_range(msg, min_range)
        self._pub.publish(msg)

    @staticmethod
    def _filter_min_range(msg: PointCloud2, min_range: float):
        """Remove points within min_range of the sensor origin, in place."""
        step = msg.point_step
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.row_step)
        points = rows[:, :msg.width * step].reshape(-1, step)
        offsets = {f.name: f.offset for f in msg.fields}
        xyz = np.stack([
            np.ascontiguousarray(points[:, offsets[a]:offsets[a] + 4]).view(np.float32)[:, 0]
            for a in ('x', 'y', 'z')
        ], axis=1)
        # NaN points compare False and are kept, as before
        keep = ~(np.einsum('ij,ij->i', xyz, xyz) < min_range * min_range)
        kept = points[keep]
        msg.height = 1
        msg.width = len(kept)
        msg.row_step = msg.width * step
        msg.data = kept.tobytes()


def main(args=None):
    rclpy.init(args=args)
    node = CloudRestampNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
