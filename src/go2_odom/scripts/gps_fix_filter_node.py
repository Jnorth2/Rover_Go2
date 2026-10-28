#!/usr/bin/env python3
"""
Forwards only high-quality GPS fixes, for use as RTAB-Map position priors.

RTAB-Map weights each fix by its covariance, but a float or single-point fix
(0.5-5 m, often biased near buildings and trees) still drags the map if it gets
in. This node passes a fix only when the u-blox reports an RTK fixed solution
(carrier phase fixed → NavSatStatus.STATUS_GBAS_FIX in ublox_gps) and its
horizontal standard deviation is under max_horizontal_std.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GpsFixFilterNode(Node):
    def __init__(self):
        super().__init__('gps_fix_filter_node')

        # true = RTK fixed only; false = any fix that passes the accuracy check
        self.declare_parameter('require_rtk_fixed', True)
        self.declare_parameter('max_horizontal_std', 0.10)  # m

        self._pub = self.create_publisher(NavSatFix, 'fix_out', 10)
        self._sub = self.create_subscription(NavSatFix, 'fix_in', self._cb, 10)
        self._passing = None

    def _cb(self, msg: NavSatFix):
        min_status = (NavSatStatus.STATUS_GBAS_FIX
                      if self.get_parameter('require_rtk_fixed').value
                      else NavSatStatus.STATUS_FIX)
        std_h = math.sqrt(max(msg.position_covariance[0], msg.position_covariance[4]))
        ok = (msg.status.status >= min_status
              and msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN
              and std_h <= self.get_parameter('max_horizontal_std').value)

        if ok != self._passing:
            if ok:
                self.get_logger().info(f'Forwarding GPS fixes (horizontal std {std_h:.3f} m)')
            else:
                self.get_logger().warn(
                    f'Holding back GPS fixes: status {msg.status.status}, '
                    f'horizontal std {std_h:.3f} m')
            self._passing = ok

        if ok:
            self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GpsFixFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
