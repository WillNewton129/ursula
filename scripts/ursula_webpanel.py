#!/usr/bin/env python3
"""
ursula_webpanel.py

Lightweight web panel that runs on the Jetson and lets you control
URSULA's launch configuration from any browser or Foxglove Custom Panel
on any device — no SSH required.

Serves a single HTML page on port 8080 that provides:
  - Launch / Stop buttons for jetson_hardware_launch, robot_launch, rtabmap
  - slam_mode selector (mapping / localization)
  - nav2, foxglove, camera, detection toggles
  - Map save (calls /ursula/save_map service via ros2 cli)
  - Live log streaming (last 50 lines of the running process)
  - Emergency stop (publishes True to /ursula/estop)

Usage (on Jetson):
  python3 ursula_webpanel.py
  # Or add to jetson startup:
  # ros2 run ursula ursula_webpanel.py

Then open from any device on the same network (or Tailscale):
  http://<jetson-ip>:8080
  http://ursula-jetson:8080   (if Tailscale hostname is set)

The panel is intentionally self-contained (no npm, no framework).
"""

import subprocess
import threading
import os
import signal
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs
import json

# ── State ──────────────────────────────────────────────────────────────────
_process: subprocess.Popen | None = None
_log_lines: list[str] = []
_log_lock = threading.Lock()
_status = "idle"

HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>URSULA Control Panel</title>
<style>
  :root {
    --bg: #0f1117; --surface: #1a1d27; --border: #2e3348;
    --accent: #4f8ef7; --danger: #e05252; --ok: #4caf7d;
    --warn: #e0a030; --text: #d4d8f0; --muted: #6b7280;
    --radius: 8px; --font: 'Segoe UI', system-ui, sans-serif;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: var(--bg); color: var(--text); font-family: var(--font);
         padding: 20px; min-height: 100vh; }
  h1 { font-size: 1.4rem; font-weight: 700; color: var(--accent); margin-bottom: 4px; }
  .subtitle { color: var(--muted); font-size: 0.85rem; margin-bottom: 20px; }
  .card { background: var(--surface); border: 1px solid var(--border);
          border-radius: var(--radius); padding: 16px; margin-bottom: 16px; }
  .card h2 { font-size: 0.9rem; font-weight: 600; color: var(--muted);
             text-transform: uppercase; letter-spacing: 0.05em; margin-bottom: 12px; }
  .row { display: flex; flex-wrap: wrap; gap: 10px; align-items: center; margin-bottom: 10px; }
  label { font-size: 0.88rem; color: var(--text); }
  select, input[type=text] {
    background: var(--bg); border: 1px solid var(--border); color: var(--text);
    padding: 6px 10px; border-radius: 5px; font-size: 0.88rem; min-width: 160px;
  }
  .toggle-row { display: flex; flex-wrap: wrap; gap: 14px; }
  .toggle { display: flex; align-items: center; gap: 6px; font-size: 0.88rem; cursor: pointer; }
  .toggle input { width: 16px; height: 16px; accent-color: var(--accent); cursor: pointer; }
  button {
    padding: 8px 18px; border: none; border-radius: 6px; font-size: 0.88rem;
    font-weight: 600; cursor: pointer; transition: opacity .15s;
  }
  button:hover { opacity: 0.85; }
  button:disabled { opacity: 0.4; cursor: not-allowed; }
  .btn-primary  { background: var(--accent); color: #fff; }
  .btn-danger   { background: var(--danger); color: #fff; }
  .btn-warn     { background: var(--warn);   color: #111; }
  .btn-ok       { background: var(--ok);     color: #111; }
  .btn-neutral  { background: var(--border); color: var(--text); }
  #status-badge {
    display: inline-block; padding: 4px 12px; border-radius: 20px;
    font-size: 0.82rem; font-weight: 700; margin-left: 10px;
  }
  .badge-idle    { background: var(--border); color: var(--muted); }
  .badge-running { background: var(--ok);     color: #111; }
  .badge-error   { background: var(--danger); color: #fff; }
  #log {
    background: #0a0c10; border: 1px solid var(--border); border-radius: 6px;
    padding: 10px; font-family: 'Courier New', monospace; font-size: 0.78rem;
    color: #8db4e0; height: 280px; overflow-y: auto; white-space: pre-wrap;
    word-break: break-all;
  }
  .estop-bar {
    background: #2a0a0a; border: 2px solid var(--danger);
    border-radius: var(--radius); padding: 14px; margin-bottom: 16px;
    display: flex; align-items: center; gap: 16px;
  }
  .estop-bar span { flex: 1; font-weight: 600; color: var(--danger); }
</style>
</head>
<body>

<h1>🤖 URSULA Control Panel</h1>
<p class="subtitle">Remote launch control · No SSH required</p>

<!-- E-STOP -->
<div class="estop-bar">
  <span>⚠ EMERGENCY STOP</span>
  <button class="btn-danger" onclick="estop()">STOP ALL MOTORS</button>
  <button class="btn-ok" onclick="estopRelease()" style="margin-left:4px">Release E-Stop</button>
</div>

<!-- LAUNCH CONFIG -->
<div class="card">
  <h2>Launch Configuration</h2>

  <div class="row">
    <label>Launch file:</label>
    <select id="launch_file">
      <option value="jetson_hardware_launch">jetson_hardware_launch (SLAM only)</option>
      <option value="robot_launch">robot_launch (+ Camera + YOLO detection)</option>
      <option value="jetson_rtabmap">jetson_rtabmap (RTAB-Map 3D)</option>
    </select>
  </div>

  <div class="row">
    <label>SLAM mode:</label>
    <select id="slam_mode">
      <option value="mapping">mapping (build new map)</option>
      <option value="localization">localization (load saved map)</option>
    </select>
  </div>

  <div class="toggle-row" style="margin-bottom:14px">
    <label class="toggle"><input type="checkbox" id="arg_nav2" checked> Nav2</label>
    <label class="toggle"><input type="checkbox" id="arg_foxglove" checked> Foxglove bridge</label>
    <label class="toggle"><input type="checkbox" id="arg_camera" checked> Camera</label>
    <label class="toggle"><input type="checkbox" id="arg_detection" checked> Detection</label>
  </div>

  <div class="row">
    <button class="btn-primary" id="btn_launch" onclick="launchRobot()">▶ Launch</button>
    <button class="btn-danger"  id="btn_stop"   onclick="stopRobot()" disabled>■ Stop Launch</button>
    <span id="status-badge" class="badge-idle">IDLE</span>
  </div>
</div>

<!-- MAP TOOLS -->
<div class="card">
  <h2>Map Tools</h2>
  <div class="row">
    <button class="btn-ok" onclick="saveMap()">💾 Save Map Now</button>
    <span id="map-status" style="font-size:0.85rem;color:var(--muted)"></span>
  </div>
  <p style="font-size:0.8rem;color:var(--muted);margin-top:8px">
    Saves to /home/uoljetson/maps/ursula_map (and a timestamped copy).<br>
    After saving, relaunch with <em>localization</em> mode to navigate on the saved map.
  </p>
</div>

<!-- LOG -->
<div class="card">
  <h2>Live Log (last 50 lines)</h2>
  <div id="log">Waiting for launch...</div>
  <div class="row" style="margin-top:8px">
    <button class="btn-neutral" onclick="clearLog()">Clear</button>
    <button class="btn-neutral" onclick="fetchLog()">Refresh</button>
    <label class="toggle" style="margin-left:auto">
      <input type="checkbox" id="auto_refresh" checked> Auto-refresh (2s)
    </label>
  </div>
</div>

<script>
let refreshTimer = null;

function badgeSet(state) {
  const b = document.getElementById('status-badge');
  b.textContent = state.toUpperCase();
  b.className = 'badge-' + (state === 'running' ? 'running' : state === 'error' ? 'error' : 'idle');
}

async function api(path, params={}) {
  const qs = new URLSearchParams(params).toString();
  const r = await fetch('/api/' + path + (qs ? '?' + qs : ''));
  return r.json();
}

async function launchRobot() {
  const args = {
    launch_file: document.getElementById('launch_file').value,
    slam_mode:   document.getElementById('slam_mode').value,
    nav2:        document.getElementById('arg_nav2').checked ? 'true' : 'false',
    foxglove:    document.getElementById('arg_foxglove').checked ? 'true' : 'false',
    camera:      document.getElementById('arg_camera').checked ? 'true' : 'false',
    detection:   document.getElementById('arg_detection').checked ? 'true' : 'false',
  };
  const r = await api('launch', args);
  appendLog('>> Launch requested: ' + JSON.stringify(args));
  document.getElementById('btn_launch').disabled = true;
  document.getElementById('btn_stop').disabled = false;
  badgeSet('running');
}

async function stopRobot() {
  const r = await api('stop');
  appendLog('>> Stop requested');
  document.getElementById('btn_launch').disabled = false;
  document.getElementById('btn_stop').disabled = true;
  badgeSet('idle');
}

async function saveMap() {
  document.getElementById('map-status').textContent = 'Saving...';
  const r = await api('save_map');
  document.getElementById('map-status').textContent = r.message || 'Done';
}

async function estop() {
  await api('estop', {state: '1'});
  appendLog('!! EMERGENCY STOP sent');
}

async function estopRelease() {
  await api('estop', {state: '0'});
  appendLog('>> E-Stop released');
}

async function fetchLog() {
  const r = await api('log');
  const el = document.getElementById('log');
  el.textContent = (r.lines || []).join('');
  el.scrollTop = el.scrollHeight;
  if (r.status) badgeSet(r.status);
}

function appendLog(line) {
  const el = document.getElementById('log');
  el.textContent += line + '\n';
  el.scrollTop = el.scrollHeight;
}

function clearLog() {
  document.getElementById('log').textContent = '';
}

function startAutoRefresh() {
  if (refreshTimer) clearInterval(refreshTimer);
  refreshTimer = setInterval(() => {
    if (document.getElementById('auto_refresh').checked) fetchLog();
  }, 2000);
}

fetchLog();
startAutoRefresh();
</script>
</body>
</html>"""


# ── Log streamer ────────────────────────────────────────────────────────────
def _stream_output(proc: subprocess.Popen):
    global _status
    for line in iter(proc.stdout.readline, b''):
        decoded = line.decode('utf-8', errors='replace')
        with _log_lock:
            _log_lines.append(decoded)
            if len(_log_lines) > 200:
                _log_lines.pop(0)
    _status = 'idle'


# ── HTTP handler ────────────────────────────────────────────────────────────
class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # Suppress default access log spam

    def _json(self, data: dict, code: int = 200):
        body = json.dumps(data).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)

    def _html(self, body: str):
        encoded = body.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def do_GET(self):
        global _process, _status, _log_lines

        parsed = urlparse(self.path)
        params = {k: v[0] for k, v in parse_qs(parsed.query).items()}

        if parsed.path == '/':
            self._html(HTML)
            return

        if not parsed.path.startswith('/api/'):
            self.send_response(404)
            self.end_headers()
            return

        action = parsed.path[5:]  # strip /api/

        # ── launch ──────────────────────────────────────────────────────
        if action == 'launch':
            if _process and _process.poll() is None:
                self._json({'ok': False, 'message': 'Already running'})
                return

            launch_file = params.get('launch_file', 'jetson_hardware_launch')
            slam_mode   = params.get('slam_mode',   'mapping')
            nav2        = params.get('nav2',        'true')
            foxglove    = params.get('foxglove',    'true')
            camera      = params.get('camera',      'true')
            detection   = params.get('detection',   'true')

            cmd = [
                'ros2', 'launch', 'ursula', f'{launch_file}.launch.py',
                f'slam_mode:={slam_mode}',
                f'nav2:={nav2}',
                f'foxglove:={foxglove}',
            ]
            # camera/detection only valid for robot_launch and rtabmap
            if launch_file in ('robot_launch', 'jetson_rtabmap'):
                cmd += [f'camera:={camera}', f'detection:={detection}']

            try:
                _process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid,
                )
                _status = 'running'
                with _log_lock:
                    _log_lines.clear()
                    _log_lines.append(f'>> Starting: {" ".join(cmd)}\n')
                threading.Thread(
                    target=_stream_output, args=(_process,), daemon=True
                ).start()
                self._json({'ok': True, 'cmd': ' '.join(cmd)})
            except Exception as e:
                _status = 'error'
                self._json({'ok': False, 'message': str(e)}, 500)

        # ── stop ────────────────────────────────────────────────────────
        elif action == 'stop':
            if _process and _process.poll() is None:
                os.killpg(os.getpgid(_process.pid), signal.SIGTERM)
                _process = None
                _status = 'idle'
                self._json({'ok': True})
            else:
                self._json({'ok': False, 'message': 'Nothing running'})

        # ── log ─────────────────────────────────────────────────────────
        elif action == 'log':
            with _log_lock:
                lines = list(_log_lines[-50:])
            self._json({'lines': lines, 'status': _status})

        # ── save_map ────────────────────────────────────────────────────
        elif action == 'save_map':
            try:
                result = subprocess.run(
                    ['ros2', 'service', 'call',
                     '/ursula/save_map', 'std_srvs/srv/Trigger', '{}'],
                    capture_output=True, text=True, timeout=20
                )
                msg = result.stdout[:300] if result.returncode == 0 else result.stderr[:300]
                self._json({'ok': result.returncode == 0, 'message': msg})
            except Exception as e:
                self._json({'ok': False, 'message': str(e)})

        # ── estop ───────────────────────────────────────────────────────
        elif action == 'estop':
            state = params.get('state', '1')
            val   = 'True' if state == '1' else 'False'
            try:
                subprocess.Popen(
                    ['ros2', 'topic', 'pub', '--once',
                     '/ursula/estop', 'std_msgs/msg/Bool', f'{{data: {val}}}']
                )
                self._json({'ok': True, 'state': val})
            except Exception as e:
                self._json({'ok': False, 'message': str(e)})

        else:
            self._json({'error': 'unknown action'}, 404)


def main():
    port = 8080
    server = HTTPServer(('0.0.0.0', port), Handler)
    print(f'URSULA Web Panel running on http://0.0.0.0:{port}')
    print(f'Open from any device: http://<jetson-ip>:{port}')
    print('Press Ctrl+C to stop.')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nShutting down.')


if __name__ == '__main__':
    main()