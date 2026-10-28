#!/usr/bin/env python3
"""
Web map of the Go2 on the OSU campus (Leaflet), served from the groundstation.

The robot marker and trail come from localization: TF map → base_link, i.e.
RTAB-Map + EKF. GPS is only used to georeference the map frame: each good fix
is paired with where localization says the antenna (TF map → gps) was at that
moment, and a 2D rotation + translation from map frame to east/north is fitted
to all pairs. Raw GPS positions are never drawn.

Open http://localhost:8080 (port and bind address are parameters).
"""
import json
import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix

EARTH_R = 6378137.0

# Merryfield Hall, Oregon State University (OpenStreetMap)
MERRYFIELD = (44.5672596, -123.2743300)


class MapUiNode(Node):
    def __init__(self):
        super().__init__('go2_map_ui')
        self.declare_parameter('port', 8080)
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('gps_topic', '/ublox_gps_node/fix')
        # Fixes worse than this don't go into the georeference fit (m, 1-sigma)
        self.declare_parameter('max_horizontal_std', 0.3)
        # Travel needed before the rotation is trusted (m between furthest pairs)
        self.declare_parameter('min_baseline', 3.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('gps_frame', 'gps')
        self.declare_parameter('trail_spacing', 0.10)  # m between trail points

        self._lock = threading.Lock()
        self._buf = tf2_ros.Buffer()
        self._tl = tf2_ros.TransformListener(self._buf, self)

        self._origin = None          # (lat0, lon0) of the local east/north plane
        self._pairs_map = []         # antenna xy in map frame
        self._pairs_enu = []         # antenna east/north from GPS
        self._pairs_w = []           # 1 / variance
        self._fit = None             # (theta, tx, ty, rms)
        self._trail = []             # base_link xy in map frame
        self._pose = None            # (x, y, yaw) in map frame
        self._gps = {}

        self.create_subscription(
            NavSatFix, self.get_parameter('gps_topic').value, self._gps_cb, 10)
        self.create_timer(0.1, self._pose_tick)

    # ---------------------------------------------------------------- geodesy
    def _to_enu(self, lat, lon):
        lat0, lon0 = self._origin
        e = math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0))
        n = math.radians(lat - lat0) * EARTH_R
        return e, n

    def _to_ll(self, e, n):
        lat0, lon0 = self._origin
        lat = lat0 + math.degrees(n / EARTH_R)
        lon = lon0 + math.degrees(e / (EARTH_R * math.cos(math.radians(lat0))))
        return lat, lon

    # ---------------------------------------------------------------- ROS
    def _lookup_xy(self, child):
        t = self._buf.lookup_transform(
            self.get_parameter('map_frame').value, child, Time())
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def _pose_tick(self):
        try:
            x, y, yaw = self._lookup_xy(self.get_parameter('base_frame').value)
        except tf2_ros.TransformException:
            return
        with self._lock:
            self._pose = (x, y, yaw)
            spacing = self.get_parameter('trail_spacing').value
            if not self._trail or math.hypot(x - self._trail[-1][0], y - self._trail[-1][1]) >= spacing:
                self._trail.append((x, y))

    def _gps_cb(self, msg: NavSatFix):
        std = math.sqrt(max(msg.position_covariance[0], msg.position_covariance[4]))
        good = (msg.status.status >= 0 and std <= self.get_parameter('max_horizontal_std').value
                and msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN)
        with self._lock:
            self._gps = {'status': msg.status.status, 'std': std, 'used': good}
        if not good:
            return
        try:
            ax, ay, _ = self._lookup_xy(self.get_parameter('gps_frame').value)
        except tf2_ros.TransformException:
            return
        with self._lock:
            if self._origin is None:
                self._origin = (msg.latitude, msg.longitude)
            e, n = self._to_enu(msg.latitude, msg.longitude)
            # One pair per 0.2 m of travel keeps the fit from being dominated by
            # long stops in one place
            if self._pairs_map and math.hypot(ax - self._pairs_map[-1][0], ay - self._pairs_map[-1][1]) < 0.2:
                return
            self._pairs_map.append((ax, ay))
            self._pairs_enu.append((e, n))
            self._pairs_w.append(1.0 / max(std * std, 1e-4))
            self._refit()

    def _refit(self):
        """Weighted 2D rigid fit enu = R(theta) * map + t, with outlier rejection."""
        P = np.array(self._pairs_map); Q = np.array(self._pairs_enu); w = np.array(self._pairs_w)
        if len(P) < 3:
            return
        span = np.max(np.linalg.norm(P - P.mean(0), axis=1)) * 2
        if span < self.get_parameter('min_baseline').value:
            return
        keep = np.ones(len(P), bool)
        for _ in range(3):
            ww = w[keep]; Pk = P[keep]; Qk = Q[keep]
            pc = (ww[:, None] * Pk).sum(0) / ww.sum(); qc = (ww[:, None] * Qk).sum(0) / ww.sum()
            A, B = Pk - pc, Qk - qc
            s = (ww * (A[:, 0] * B[:, 1] - A[:, 1] * B[:, 0])).sum()
            c = (ww * (A[:, 0] * B[:, 0] + A[:, 1] * B[:, 1])).sum()
            th = math.atan2(s, c)
            Rm = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
            t = qc - Rm @ pc
            res = np.linalg.norm(Q - (P @ Rm.T + t), axis=1)
            thr = max(0.5, 3.0 * np.median(res[keep]))
            new_keep = res < thr
            if new_keep.sum() < 3 or (new_keep == keep).all():
                break
            keep = new_keep
        rms = float(np.sqrt(np.mean(res[keep] ** 2)))
        self._fit = (th, float(t[0]), float(t[1]), rms, int(keep.sum()))

    # ---------------------------------------------------------------- state for the page
    def state(self):
        with self._lock:
            out = {'gps': self._gps, 'pairs': len(self._pairs_map), 'aligned': False,
                   'have_pose': self._pose is not None, 'trail_len': len(self._trail)}
            if self._fit is None or self._pose is None:
                if self._pairs_map:
                    P = np.array(self._pairs_map)
                    out['baseline'] = float(np.max(np.linalg.norm(P - P.mean(0), axis=1)) * 2)
                return out
            th, tx, ty, rms, used = self._fit
            c, s = math.cos(th), math.sin(th)

            def ll(x, y):
                return self._to_ll(c * x - s * y + tx, s * x + c * y + ty)
            x, y, yaw = self._pose
            lat, lon = ll(x, y)
            # Heading in degrees clockwise from north, for the marker
            heading = (90.0 - math.degrees(yaw + th)) % 360.0
            out.update({
                'aligned': True, 'lat': lat, 'lon': lon, 'heading': heading,
                'trail': [ll(px, py) for px, py in self._trail],
                'fit_rms': rms, 'fit_used': used,
            })
            return out

    def georef(self):
        """Map-frame ↔ GPS alignment: east/north = R(theta) * map_xy + (tx, ty),
        east/north in metres on a local plane around origin (lat, lon)."""
        with self._lock:
            if self._fit is None:
                return {'aligned': False, 'pairs': len(self._pairs_map)}
            th, tx, ty, rms, used = self._fit
            return {'aligned': True, 'origin_lat': self._origin[0], 'origin_lon': self._origin[1],
                    'theta': th, 'tx': tx, 'ty': ty, 'rms': rms, 'fixes_used': used,
                    'map_frame': self.get_parameter('map_frame').value}

    def clear_trail(self):
        with self._lock:
            self._trail = self._trail[-1:]


PAGE = r"""<!doctype html>
<html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Go2 Campus Map</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
  :root { --bg:#ffffff; --fg:#1d1d1f; --muted:#6b6b70; --panel:rgba(255,255,255,.94); --accent:#d73f09; --ok:#1a7f37; --warn:#b35900; --line:#e2e2e6; }
  @media (prefers-color-scheme: dark) { :root { --bg:#141416; --fg:#ececef; --muted:#9a9aa2; --panel:rgba(28,28,31,.94); --ok:#4cc26b; --warn:#f0a23b; --line:#333338; } }
  html,body { margin:0; height:100%; background:var(--bg); color:var(--fg); font:14px/1.4 system-ui,-apple-system,"Segoe UI",sans-serif; }
  #map { position:absolute; inset:0; }
  #panel { position:absolute; top:12px; left:56px; z-index:1000; background:var(--panel); border:1px solid var(--line);
           border-radius:10px; padding:10px 12px; min-width:230px; max-width:calc(100vw - 88px); box-shadow:0 2px 10px rgba(0,0,0,.15); }
  #panel h1 { font-size:15px; margin:0 0 6px; }
  .row { display:flex; justify-content:space-between; gap:12px; color:var(--muted); }
  .row b { color:var(--fg); font-weight:600; font-variant-numeric:tabular-nums; }
  #status { margin:6px 0; font-weight:600; }
  .ok { color:var(--ok); } .warn { color:var(--warn); }
  .btns { display:flex; gap:6px; margin-top:8px; flex-wrap:wrap; }
  button { font:inherit; padding:4px 10px; border-radius:6px; border:1px solid var(--line); background:var(--bg); color:var(--fg); cursor:pointer; }
  button:hover { border-color:var(--accent); }
  .robot { width:28px; height:28px; }
  .robot svg { width:28px; height:28px; overflow:visible; }
</style></head><body>
<div id="map"></div>
<div id="panel">
  <h1>Go2 position</h1>
  <div id="status" class="warn">Connecting…</div>
  <div class="row"><span>GPS</span><b id="gps">–</b></div>
  <div class="row"><span>Alignment</span><b id="fit">–</b></div>
  <div class="row"><span>Trail</span><b id="trail">–</b></div>
  <div class="btns">
    <button id="follow">Follow: on</button>
    <button id="center">Merryfield</button>
    <button id="clear">Clear trail</button>
  </div>
</div>
<script>
const MERRYFIELD = [__LAT__, __LON__];
const map = L.map('map', { zoomControl: true, maxZoom: 21 }).setView(MERRYFIELD, 18);
const streets = L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png',
  { maxZoom: 21, maxNativeZoom: 19, attribution: '&copy; OpenStreetMap contributors' }).addTo(map);
const satellite = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  { maxZoom: 21, maxNativeZoom: 19, attribution: 'Imagery &copy; Esri' });
L.control.layers({ 'Streets': streets, 'Satellite': satellite }).addTo(map);
L.marker(MERRYFIELD, { title: 'Merryfield Hall' }).addTo(map).bindTooltip('Merryfield Hall');

const trail = L.polyline([], { color: '#d73f09', weight: 4, opacity: .9 }).addTo(map);
const robotIcon = L.divIcon({ className: 'robot', iconSize: [28, 28], iconAnchor: [14, 14],
  html: '<svg viewBox="-14 -14 28 28"><g id="arrow"><circle r="9" fill="#d73f09" stroke="#fff" stroke-width="2.5"/>' +
        '<path d="M0,-13 L5,-5 L-5,-5 Z" fill="#d73f09" stroke="#fff" stroke-width="1.5"/></g></svg>' });
let robot = null, follow = true;

document.getElementById('follow').onclick = e => { follow = !follow; e.target.textContent = 'Follow: ' + (follow ? 'on' : 'off'); };
document.getElementById('center').onclick = () => { follow = false; document.getElementById('follow').textContent = 'Follow: off'; map.setView(MERRYFIELD, 18); };
document.getElementById('clear').onclick = () => fetch('/clear', { method: 'POST' });
map.on('dragstart', () => { follow = false; document.getElementById('follow').textContent = 'Follow: off'; });

const GPS_STATUS = { '-1': 'no fix', '0': 'fix', '1': 'SBAS', '2': 'RTK fixed' };
function set(id, text, cls) { const el = document.getElementById(id); el.textContent = text; if (cls !== undefined) el.className = cls; }

async function tick() {
  try {
    const s = await (await fetch('/state', { cache: 'no-store' })).json();
    const g = s.gps || {};
    set('gps', g.status === undefined ? 'no messages' :
        `${GPS_STATUS[g.status] ?? g.status}, ±${(g.std * 100).toFixed(1)} cm${g.used ? '' : ' (not used)'}`);
    set('trail', `${s.trail_len} pts`);
    if (!s.have_pose) set('status', 'Waiting for map → base_link TF', 'warn');
    else if (!s.aligned) {
      const b = s.baseline !== undefined ? `, ${s.baseline.toFixed(1)} m travelled` : '';
      set('status', `Aligning to GPS: ${s.pairs} fixes${b} (need 3 m)`, 'warn');
      set('fit', '–');
    } else {
      set('status', 'Localized', 'ok');
      set('fit', `${(s.fit_rms * 100).toFixed(0)} cm rms, ${s.fit_used} fixes`);
      trail.setLatLngs(s.trail);
      const ll = [s.lat, s.lon];
      if (!robot) robot = L.marker(ll, { icon: robotIcon, zIndexOffset: 1000 }).addTo(map);
      robot.setLatLng(ll);
      const g2 = robot.getElement() && robot.getElement().querySelector('#arrow');
      if (g2) g2.setAttribute('transform', `rotate(${s.heading})`);
      if (follow) map.panTo(ll, { animate: true, duration: 0.25 });
    }
  } catch (e) { set('status', 'UI server not reachable', 'warn'); }
  setTimeout(tick, 300);
}
tick();
</script></body></html>
""".replace('__LAT__', str(MERRYFIELD[0])).replace('__LON__', str(MERRYFIELD[1]))


def make_handler(node: MapUiNode):
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, body, ctype):
            data = body.encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                self._send(200, PAGE, 'text/html; charset=utf-8')
            elif self.path == '/state':
                self._send(200, json.dumps(node.state()), 'application/json')
            elif self.path == '/georef':
                self._send(200, json.dumps(node.georef()), 'application/json')
            else:
                self._send(404, 'not found', 'text/plain')

        def do_POST(self):
            if self.path == '/clear':
                node.clear_trail()
                self._send(200, '{}', 'application/json')
            else:
                self._send(404, 'not found', 'text/plain')

        def log_message(self, *args):
            pass
    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = MapUiNode()
    host, port = node.get_parameter('host').value, node.get_parameter('port').value
    server = ThreadingHTTPServer((host, port), make_handler(node))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f'Go2 map UI at http://{"localhost" if host in ("127.0.0.1", "0.0.0.0") else host}:{port}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
