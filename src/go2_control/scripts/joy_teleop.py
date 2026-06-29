#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from unitree_api.msg import Request

# API IDs from ros2_sport_client.h
_API_MOVE = 1008
_API_STOPMOVE = 1003
_API_STANDUP = 1004
_API_STANDDOWN = 1005

# PS4/5 axis indices (joy_linux defaults)
#   axes[0]  left stick X  (+1=left,  -1=right)
#   axes[1]  left stick Y  (+1=up,    -1=down)   → vx
#   axes[2]  L2 trigger
#   axes[3]  right stick X (+1=left,  -1=right)  → vyaw
#   axes[4]  right stick Y
# Button indices
#   0=Cross  1=Circle  2=Triangle  3=Square  4=L1  5=R1

_DEADZONE = 0.08


class Go2JoyTeleop(Node):
    def __init__(self):
        super().__init__('go2_joy_teleop')

        self.declare_parameter('axis_vx', 1)        # left stick Y
        self.declare_parameter('axis_vy', 0)        # left stick X
        self.declare_parameter('axis_vyaw', 3)      # right stick X
        self.declare_parameter('btn_sit_stand', 3)  # Square
        self.declare_parameter('max_linear', 0.8)   # m/s
        self.declare_parameter('max_angular', 1.0)  # rad/s
        self.declare_parameter('deadzone', _DEADZONE)

        self._pub = self.create_publisher(Request, '/api/sport/request', 10)
        self._sub = self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self._timer = self.create_timer(0.05, self._tick)  # 20 Hz

        self._joy: Joy | None = None
        self._is_sitting = False
        self._prev_btn = 0

    def _joy_cb(self, msg: Joy):
        self._joy = msg

    def _tick(self):
        if self._joy is None:
            return

        joy = self._joy
        btn_idx = self.get_parameter('btn_sit_stand').value

        btn = joy.buttons[btn_idx] if btn_idx < len(joy.buttons) else 0
        if btn and not self._prev_btn:
            self._toggle_sit_stand()
        self._prev_btn = btn

        if not self._is_sitting:
            ax_vx = self.get_parameter('axis_vx').value
            ax_vy = self.get_parameter('axis_vy').value
            ax_vyaw = self.get_parameter('axis_vyaw').value
            max_lin = self.get_parameter('max_linear').value
            max_ang = self.get_parameter('max_angular').value
            dz = self.get_parameter('deadzone').value

            vx = self._dz(self._axis(joy, ax_vx), dz) * max_lin
            vy = self._dz(self._axis(joy, ax_vy), dz) * max_lin
            vyaw = self._dz(self._axis(joy, ax_vyaw), dz) * max_ang

            self._send_move(vx, vy, vyaw)

    def _toggle_sit_stand(self):
        req = Request()
        if self._is_sitting:
            req.header.identity.api_id = _API_STANDUP
            self._is_sitting = False
            self.get_logger().info('Standing up')
        else:
            req.header.identity.api_id = _API_STANDDOWN
            self._is_sitting = True
            self.get_logger().info('Standing down')
        self._pub.publish(req)

    def _send_move(self, vx: float, vy: float, vyaw: float):
        req = Request()
        req.header.identity.api_id = _API_MOVE
        req.parameter = json.dumps({'x': vx, 'y': vy, 'z': vyaw})
        self._pub.publish(req)

    @staticmethod
    def _axis(joy: Joy, idx: int) -> float:
        return float(joy.axes[idx]) if idx < len(joy.axes) else 0.0

    @staticmethod
    def _dz(val: float, dz: float) -> float:
        return 0.0 if abs(val) < dz else val


def main():
    rclpy.init()
    node = Go2JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
