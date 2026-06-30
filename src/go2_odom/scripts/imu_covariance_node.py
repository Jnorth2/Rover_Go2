#!/usr/bin/env python3
"""
Republishes IMU messages with populated covariance matrices and corrected timestamps.

The Go2's UTlidar publishes sensor_msgs/Imu on /utlidar/imu with:
  - Zero covariances (robot_localization ignores zero-covariance data)
  - Go2 system clock timestamps (offset from the Jetson's clock by a fixed delta)

This node injects diagonal covariance matrices and re-stamps to Jetson time so the
EKF can receive all sensor inputs on a common clock.

Orientation covariance_diag < 0 signals "orientation not provided" per REP-145,
which tells robot_localization to skip orientation fusion from this source.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuCovarianceNode(Node):
    def __init__(self):
        super().__init__('imu_covariance_node')

        self.declare_parameter('orientation_covariance_diag', -1.0)
        self.declare_parameter('angular_velocity_covariance_diag', 0.001)
        self.declare_parameter('linear_acceleration_covariance_diag', 0.01)

        ori_diag = self.get_parameter('orientation_covariance_diag').value
        ang_diag = self.get_parameter('angular_velocity_covariance_diag').value
        lin_diag = self.get_parameter('linear_acceleration_covariance_diag').value

        # REP-145: orientation_covariance[0] = -1 means orientation unavailable
        if ori_diag < 0:
            self._ori_cov = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        else:
            self._ori_cov = [ori_diag, 0.0, 0.0,
                             0.0, ori_diag, 0.0,
                             0.0, 0.0, ori_diag]

        self._ang_cov = [ang_diag, 0.0, 0.0,
                         0.0, ang_diag, 0.0,
                         0.0, 0.0, ang_diag]
        self._lin_cov = [lin_diag, 0.0, 0.0,
                         0.0, lin_diag, 0.0,
                         0.0, 0.0, lin_diag]

        self._pub = self.create_publisher(Imu, 'imu_out', 10)
        self._sub = self.create_subscription(Imu, 'imu_in', self._cb, 10)

    def _cb(self, msg: Imu):
        out = Imu()
        # Re-stamp to Jetson clock so EKF inputs share a common time reference.
        # The Go2's clock is offset from the Jetson's; without this the EKF
        # cannot sync odom and IMU measurements correctly.
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        out.orientation = msg.orientation
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration
        out.orientation_covariance = self._ori_cov
        out.angular_velocity_covariance = self._ang_cov
        out.linear_acceleration_covariance = self._lin_cov
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuCovarianceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
