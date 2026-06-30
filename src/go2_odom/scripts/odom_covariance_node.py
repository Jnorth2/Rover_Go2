#!/usr/bin/env python3
"""
Injects diagonal pose/twist covariance matrices into nav_msgs/Odometry messages.

/utlidar/robot_odom from the Unitree Go2 publishes with zero covariance matrices,
which robot_localization silently ignores. This node injects physically reasonable
diagonal values so the EKF can properly weight each measurement.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomCovarianceNode(Node):
    def __init__(self):
        super().__init__('odom_covariance_node')

        # 6-element diagonal for pose [x, y, z, roll, pitch, yaw] in m² and rad²
        self.declare_parameter(
            'pose_covariance_diagonal',
            [0.01, 0.01, 0.1, 0.1, 0.1, 0.05],
        )
        # 6-element diagonal for twist [vx, vy, vz, vroll, vpitch, vyaw]
        self.declare_parameter(
            'twist_covariance_diagonal',
            [0.01, 0.05, 0.1, 0.1, 0.1, 0.02],
        )

        pose_diag = self.get_parameter('pose_covariance_diagonal').value
        twist_diag = self.get_parameter('twist_covariance_diagonal').value

        # Build flat 36-element covariance matrices from diagonal values
        self._pose_cov = [0.0] * 36
        self._twist_cov = [0.0] * 36
        for i, v in enumerate(pose_diag):
            self._pose_cov[i * 7] = float(v)
        for i, v in enumerate(twist_diag):
            self._twist_cov[i * 7] = float(v)

        self._pub = self.create_publisher(Odometry, 'odom_out', 10)
        self._sub = self.create_subscription(Odometry, 'odom_in', self._cb, 10)

    def _cb(self, msg: Odometry):
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose = msg.pose.pose
        out.twist.twist = msg.twist.twist
        out.pose.covariance = self._pose_cov
        out.twist.covariance = self._twist_cov
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomCovarianceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
