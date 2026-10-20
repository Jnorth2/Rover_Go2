#!/usr/bin/env python3
import json
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from unitree_api.msg import Request

# API IDs from ros2_sport_client.h
_API_BALANCE_STAND = 1002
_API_EULER = 1007
_API_MOVE = 1008
_API_STANDUP = 1004
_API_STANDDOWN = 1005

# PS4/5 axis indices (joy/SDL GameController layout)
#   axes[0]  left stick X  (+1=left,  -1=right)
#   axes[1]  left stick Y  (+1=up,    -1=down)   → vx
#   axes[2]  right stick X (+1=left,  -1=right)  → vyaw
#   axes[3]  right stick Y (+1=up,    -1=down)   → body pitch
#   axes[4]  L2 trigger
#   axes[5]  R2 trigger
# Button indices
#   0=Cross  1=Circle  2=Square  3=Triangle  4=Share  5=PS  6=Options
#
# /cmd_vel (Nav2) is forwarded as Move when the sticks are idle and Nav2 is
# enabled (Triangle toggles). Stick input always takes priority over Nav2.

_DEADZONE = 0.08


class Go2JoyTeleop(Node):
    def __init__(self):
        super().__init__('go2_joy_teleop')

        self.declare_parameter('axis_vx', 1)        # left stick Y
        self.declare_parameter('axis_vy', 0)        # left stick X
        self.declare_parameter('axis_vyaw', 2)      # right stick X
        self.declare_parameter('axis_pitch', 3)     # right stick Y
        self.declare_parameter('btn_sit_stand', 2)  # Square
        self.declare_parameter('btn_nav_toggle', 3)  # Triangle
        self.declare_parameter('nav_enabled', True)  # Nav2 may drive at startup
        self.declare_parameter('max_linear', 0.8)   # m/s
        self.declare_parameter('max_angular', 1.0)  # rad/s
        self.declare_parameter('max_pitch', 0.3)    # rad (Go2 limit ~0.75)
        self.declare_parameter('deadzone', _DEADZONE)
        # Inputs older than this are treated as zero, so a lost link stops the robot
        self.declare_parameter('joy_timeout', 0.5)      # s (joy_node autorepeats at 20 Hz)
        self.declare_parameter('cmd_vel_timeout', 0.5)  # s
        # The Go2 walks in discrete steps and barely advances on small Move
        # commands, so Nav2 speeds below these floors are raised to them. Commands
        # below the deadbands are noise around a stop and are zeroed.
        self.declare_parameter('nav_min_linear', 0.15)       # m/s
        self.declare_parameter('nav_min_angular', 0.3)       # rad/s
        self.declare_parameter('nav_deadband_linear', 0.03)  # m/s
        self.declare_parameter('nav_deadband_angular', 0.05)  # rad/s

        self._pub = self.create_publisher(Request, '/api/sport/request', 10)
        self._joy_sub = self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self._cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self._timer = self.create_timer(0.05, self._tick)  # 20 Hz

        self._joy: Joy | None = None
        self._joy_time = None
        self._cmd_vel: Twist | None = None
        self._cmd_vel_time = None
        self._nav_enabled = self.get_parameter('nav_enabled').value
        self._prev_nav_btn = 0
        self._is_sitting = False
        self._ready_to_move = False
        self._prev_btn = 0
        self._balance_stand_timer = None
        self._was_moving = False
        self._prev_pitch = 0.0

    def _joy_cb(self, msg: Joy):
        self._joy = msg
        self._joy_time = self.get_clock().now()

    def _cmd_vel_cb(self, msg: Twist):
        self._cmd_vel = msg
        self._cmd_vel_time = self.get_clock().now()

    def _is_fresh(self, stamp, timeout_param: str) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return age < self.get_parameter(timeout_param).value

    def _tick(self):
        joy = self._joy if self._is_fresh(self._joy_time, 'joy_timeout') else None

        if joy is not None:
            btn_idx = self.get_parameter('btn_sit_stand').value
            btn = joy.buttons[btn_idx] if btn_idx < len(joy.buttons) else 0
            if btn and not self._prev_btn:
                self._toggle_sit_stand()
            self._prev_btn = btn

            nav_idx = self.get_parameter('btn_nav_toggle').value
            nav_btn = joy.buttons[nav_idx] if nav_idx < len(joy.buttons) else 0
            if nav_btn and not self._prev_nav_btn:
                self._nav_enabled = not self._nav_enabled
                self.get_logger().info(
                    'Nav2 control ' + ('enabled' if self._nav_enabled else 'disabled'))
            self._prev_nav_btn = nav_btn

        if self._ready_to_move:
            max_lin = self.get_parameter('max_linear').value
            max_ang = self.get_parameter('max_angular').value
            vx = vy = vyaw = pitch = 0.0

            if joy is not None:
                ax_vx = self.get_parameter('axis_vx').value
                ax_vy = self.get_parameter('axis_vy').value
                ax_vyaw = self.get_parameter('axis_vyaw').value
                ax_pitch = self.get_parameter('axis_pitch').value
                max_pitch = self.get_parameter('max_pitch').value
                dz = self.get_parameter('deadzone').value

                vx = self._dz(self._axis(joy, ax_vx), dz) * max_lin
                vy = self._dz(self._axis(joy, ax_vy), dz) * max_lin
                vyaw = self._dz(self._axis(joy, ax_vyaw), dz) * max_ang
                # Stick up = look up; Go2 positive pitch is nose down
                pitch = -self._dz(self._axis(joy, ax_pitch), dz) * max_pitch

            # Sticks idle: let Nav2 drive, within the same speed limits
            if (not (vx or vy or vyaw) and self._nav_enabled
                    and self._is_fresh(self._cmd_vel_time, 'cmd_vel_timeout')):
                cmd = self._cmd_vel
                vx, vy, vyaw = self._nav_floor(cmd.linear.x, cmd.linear.y, cmd.angular.z)
                vx = self._clamp(vx, max_lin)
                vy = self._clamp(vy, max_lin)
                vyaw = self._clamp(vyaw, max_ang)

            if vx or vy or vyaw:
                self._send_move(vx, vy, vyaw)
                self._was_moving = True
                self._prev_pitch = 0.0
                return

            if self._was_moving:
                # Stop, then return to balance stand so Euler takes effect
                self._send_move(0.0, 0.0, 0.0)
                self._send_api(_API_BALANCE_STAND)
                self._was_moving = False

            # Euler holds while stick is deflected; one zero on release levels the body
            if pitch or self._prev_pitch:
                self._send_euler(0.0, pitch, 0.0)
            self._prev_pitch = pitch

    def _nav_floor(self, vx: float, vy: float, vyaw: float):
        """Raise small Nav2 commands to speeds the Go2 actually walks at."""
        min_lin = self.get_parameter('nav_min_linear').value
        min_ang = self.get_parameter('nav_min_angular').value
        speed = math.hypot(vx, vy)
        if speed < self.get_parameter('nav_deadband_linear').value:
            vx = vy = 0.0
        elif speed < min_lin:
            # Keep the direction, raise the magnitude
            vx, vy = vx * min_lin / speed, vy * min_lin / speed
        if abs(vyaw) < self.get_parameter('nav_deadband_angular').value:
            vyaw = 0.0
        elif abs(vyaw) < min_ang and not (vx or vy):
            # Only for turning in place; while walking, small yaw rates steer fine
            vyaw = math.copysign(min_ang, vyaw)
        return vx, vy, vyaw

    def _toggle_sit_stand(self):
        if self._is_sitting:
            self._send_api(_API_STANDUP)
            self._is_sitting = False
            self._ready_to_move = False
            self.get_logger().info('Standing up')
            if self._balance_stand_timer is not None:
                self._balance_stand_timer.cancel()
            self._balance_stand_timer = self.create_timer(2.0, self._on_balance_stand_timer)
        else:
            if self._balance_stand_timer is not None:
                self._balance_stand_timer.cancel()
                self._balance_stand_timer = None
            self._send_api(_API_STANDDOWN)
            self._is_sitting = True
            self._ready_to_move = False
            self.get_logger().info('Standing down')

    def _on_balance_stand_timer(self):
        self._balance_stand_timer.cancel()
        self._balance_stand_timer = None
        self._send_api(_API_BALANCE_STAND)
        self._ready_to_move = True
        self.get_logger().info('Balance stand sent — ready to move')

    def _send_move(self, vx: float, vy: float, vyaw: float):
        req = Request()
        req.header.identity.api_id = _API_MOVE
        req.parameter = json.dumps({'x': vx, 'y': vy, 'z': vyaw})
        self._pub.publish(req)

    def _send_euler(self, roll: float, pitch: float, yaw: float):
        req = Request()
        req.header.identity.api_id = _API_EULER
        req.parameter = json.dumps({'x': roll, 'y': pitch, 'z': yaw})
        self._pub.publish(req)

    def _send_api(self, api_id: int):
        req = Request()
        req.header.identity.api_id = api_id
        self._pub.publish(req)

    @staticmethod
    def _axis(joy: Joy, idx: int) -> float:
        return float(joy.axes[idx]) if idx < len(joy.axes) else 0.0

    @staticmethod
    def _dz(val: float, dz: float) -> float:
        return 0.0 if abs(val) < dz else val

    @staticmethod
    def _clamp(val: float, limit: float) -> float:
        return max(-limit, min(limit, val))


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
