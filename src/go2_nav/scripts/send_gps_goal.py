#!/usr/bin/env python3
"""
Send the Go2 to a GPS waypoint through Nav2.

    ros2 run go2_nav send_gps_goal.py                      # 10 m straight ahead
    ros2 run go2_nav send_gps_goal.py --forward 5          # 5 m straight ahead
    ros2 run go2_nav send_gps_goal.py --lat 44.56740 --lon -123.27420

The waypoint is converted to the map frame with the map ↔ GPS alignment that
go2_map_ui maintains (http://localhost:8080/georef), so the map UI must be
running and show "Localized" (≈3 m of travel with GPS fixes). With --forward,
the waypoint's lat/lon is computed from the current localized pose and heading.

The robot only moves if go2_joy_teleop is ready (□ sit → stand done) and Nav2
control is enabled (△). Ctrl+C cancels the goal.
"""
import argparse
import json
import math
import urllib.error
import urllib.request

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

EARTH_R = 6378137.0


class Georef:
    """east/north = R(theta) * map_xy + t, on a local plane around (lat0, lon0)."""

    def __init__(self, g):
        self.lat0, self.lon0 = g['origin_lat'], g['origin_lon']
        self.th, self.tx, self.ty = g['theta'], g['tx'], g['ty']
        self.rms = g['rms']

    def map_to_ll(self, x, y):
        c, s = math.cos(self.th), math.sin(self.th)
        e, n = c * x - s * y + self.tx, s * x + c * y + self.ty
        lat = self.lat0 + math.degrees(n / EARTH_R)
        lon = self.lon0 + math.degrees(e / (EARTH_R * math.cos(math.radians(self.lat0))))
        return lat, lon

    def ll_to_map(self, lat, lon):
        e = math.radians(lon - self.lon0) * EARTH_R * math.cos(math.radians(self.lat0))
        n = math.radians(lat - self.lat0) * EARTH_R
        e, n = e - self.tx, n - self.ty
        c, s = math.cos(-self.th), math.sin(-self.th)
        return c * e - s * n, s * e + c * n


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--forward', type=float, default=10.0, help='metres straight ahead (default 10)')
    ap.add_argument('--lat', type=float, help='waypoint latitude (overrides --forward)')
    ap.add_argument('--lon', type=float, help='waypoint longitude')
    ap.add_argument('--ui', default='http://localhost:8080', help='go2_map_ui address')
    ap.add_argument('--yes', action='store_true', help="don't ask for confirmation")
    args, ros_args = ap.parse_known_args()
    if (args.lat is None) != (args.lon is None):
        ap.error('--lat and --lon go together')

    try:
        with urllib.request.urlopen(args.ui.rstrip('/') + '/georef', timeout=3) as r:
            g = json.load(r)
    except urllib.error.HTTPError as e:
        raise SystemExit(f'Map UI at {args.ui} has no /georef (HTTP {e.code}): it was started '
                         'before that was added. Restart go2_map_ui (or the groundstation launch).')
    except urllib.error.URLError as e:
        raise SystemExit(f'Map UI not reachable at {args.ui} ({e.reason}). Start the groundstation '
                         'launch or: ros2 run go2_nav go2_map_ui.py')
    if not g.get('aligned'):
        raise SystemExit(f"Map ↔ GPS alignment not ready ({g.get('pairs', 0)} fixes). "
                         'Drive ~3 m with GPS until the map UI shows "Localized".')
    geo = Georef(g)

    rclpy.init(args=ros_args)
    node = Node('send_gps_goal')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    map_frame = g.get('map_frame', 'map')

    # Current localized pose
    tf = None
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            tf = buf.lookup_transform(map_frame, 'base_link', Time())
            break
        except tf2_ros.TransformException:
            pass
    if tf is None:
        raise SystemExit(f'No TF {map_frame} → base_link; is the robot stack running?')
    x0, y0 = tf.transform.translation.x, tf.transform.translation.y
    q = tf.transform.rotation
    yaw0 = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    if args.lat is not None:
        lat, lon = args.lat, args.lon
        gx, gy = geo.ll_to_map(lat, lon)
        goal_yaw = math.atan2(gy - y0, gx - x0)  # face the direction of travel
    else:
        fx, fy = x0 + args.forward * math.cos(yaw0), y0 + args.forward * math.sin(yaw0)
        lat, lon = geo.map_to_ll(fx, fy)          # the GPS waypoint
        gx, gy = geo.ll_to_map(lat, lon)          # back to the map frame for Nav2
        goal_yaw = yaw0

    here_lat, here_lon = geo.map_to_ll(x0, y0)
    dist = math.hypot(gx - x0, gy - y0)
    bearing = (90.0 - math.degrees(math.atan2(*reversed(_enu_delta(geo, x0, y0, gx, gy))))) % 360
    print(f'Robot   : {here_lat:.7f}, {here_lon:.7f}')
    print(f'Waypoint: {lat:.7f}, {lon:.7f}  ({dist:.1f} m, bearing {bearing:.0f}° from north)')
    print(f'Map goal: x={gx:.2f} y={gy:.2f} in {map_frame}  (alignment fit {geo.rms * 100:.0f} cm rms)')
    if not args.yes and input('Send this goal? [y/N] ').strip().lower() != 'y':
        raise SystemExit('Not sent.')

    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    if not client.wait_for_server(timeout_sec=5.0):
        raise SystemExit('Nav2 navigate_to_pose action not available.')

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = map_frame
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
    goal.pose.pose.orientation.z = math.sin(goal_yaw / 2)
    goal.pose.pose.orientation.w = math.cos(goal_yaw / 2)

    def feedback(fb):
        f = fb.feedback
        print(f'\r  {f.distance_remaining:5.1f} m remaining, {f.number_of_recoveries} recoveries   ', end='', flush=True)

    send = client.send_goal_async(goal, feedback_callback=feedback)
    rclpy.spin_until_future_complete(node, send)
    handle = send.result()
    if not handle.accepted:
        raise SystemExit('Goal rejected by Nav2.')
    print('Goal accepted. Ctrl+C to cancel.')
    result = handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, result)
        status = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(result.result().status, result.result().status)
        print(f'\nResult: {status}')
    except KeyboardInterrupt:
        print('\nCancelling…')
        rclpy.spin_until_future_complete(node, handle.cancel_goal_async(), timeout_sec=3.0)
    node.destroy_node()
    rclpy.try_shutdown()


def _enu_delta(geo, x0, y0, x1, y1):
    c, s = math.cos(geo.th), math.sin(geo.th)
    dx, dy = x1 - x0, y1 - y0
    return c * dx - s * dy, s * dx + c * dy   # (east, north)


if __name__ == '__main__':
    main()
