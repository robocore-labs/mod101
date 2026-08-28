#!/usr/bin/env python3
"""mod101 configurator backend.

Serves configurator/ as static files. Endpoints:
  GET  /load              - read the four build args (shoulder/elbow rail
                            length + shoulder/elbow mount) from
                            src/mod101_description/urdf/mod101_config.xacro
  POST /save              - write those four values back in-place (fast; does
                            NOT rebuild the collision matrices)
  GET  /masses            - return src/mod101_description/link_masses.json
                            (Part C output) so the page uses real printed masses
  GET  /tool, POST /tool  - active end-effector package + discovery
  GET  /collisions        - self-collision matrix status: running, last result,
                            and `stale` (do the generated matrices still match
                            the args on disk?). mod101 only; consumers run their
                            own — see DOWNSTREAM_NOTE
  POST /collisions/regen  - rebuild the matrices. Slow (minutes at
                            REGEN_TRIALS); its own button in the UI
  GET  /urdf              - run `xacro` and return the expanded URDF, with
                            mesh URIs rewritten to /pkg/<pkg>/meshes/<file>
  GET  /pkg/<pkg>/meshes/<file> - serve binary mesh files from any package
  GET  /calibration       - read back the generated calibration artifacts
  POST /calibration       - write calibration.yaml (ROS) + lerobot_calibration
                            .json from calibrate.html's sweep results

Servo bus (see motors.py — talks to the Feetech chain over USB-TTL directly):
  GET  /bus               - connection status + known servo IDs
  GET  /bus/ports         - candidate serial devices
  POST /bus/connect       - {port} open the bus and scan it
  POST /bus/disconnect    - disarm + close
  POST /bus/scan          - re-ping the bus
  POST /bus/disarm        - torque off everything (also the unload beacon)
  GET  /servo/<id>        - one servo's telemetry
  POST /servo/<id>/torque - {on}
  POST /servo/<id>/move   - {pos, speed?, accel?}
  POST /servo/<id>/profile- {speed, accel}
  POST /servo/<id>/id     - {to} reassign servo ID

Stdlib only. Run from project root:
    python3 configurator/server.py
"""

import atexit
import http.server
import json
import threading
import pathlib
import os
import re
import shutil
import socketserver
import subprocess
import sys
import tempfile
from pathlib import Path

# Run as `python3 configurator/server.py` this is already sys.path[0], but be
# explicit so importing server.py as a module (tests) resolves motors too.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from motors import BUS, BusError  # noqa: E402

PORT = 8001
HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
SRC  = ROOT / 'src'
DESC = SRC / 'mod101_description'
# What we render for the live preview...
XACRO = DESC / 'urdf' / 'mod101.xacro'
# ...and what we actually edit. The four build args + tool live in their own
# file so every consumer of the mod101_arm macro (the standalone arm, base101,
# anything else) includes the same source of truth instead of only mod101.xacro.
CONFIG = DESC / 'urdf' / 'mod101_config.xacro'

MASSES = DESC / 'link_masses.json'

# Calibration artifacts written by configurator/calibrate.html. Both are
# generated files — the wizard is the source of truth, not hand edits.
CAL_DIR = SRC / 'mod101_control' / 'config'
CAL_YAML = CAL_DIR / 'calibration.yaml'
CAL_LEROBOT = CAL_DIR / 'lerobot_calibration.json'

# The two rail-length build args: <xacro:arg name="shoulder_ext_length" default="0.130"/>
LEN_ARG_RE = re.compile(
    r'(<xacro:arg\s+name="(?P<name>shoulder_ext_length|elbow_ext_length)"\s+default=")'
    r'(?P<val>[^"]+)'
    r'(")'
)

# The two mount build args: <xacro:arg name="shoulder_mount" default="small"/>
MOUNT_ARG_RE = re.compile(
    r'(<xacro:arg\s+name="(?P<name>shoulder_mount|elbow_mount)"\s+default=")'
    r'(?P<val>[^"]+)'
    r'(")'
)

# `<xacro:arg name="tool" default="<value>"/>` — selects the end-effector pkg.
TOOL_ARG_RE = re.compile(
    r'(<xacro:arg\s+name="tool"\s+default=")'
    r'(?P<val>[^"]+)'
    r'(")'
)


def list_tools() -> list[str]:
    """Discover available tool packages on disk (sibling mod101_tool_* dirs)."""
    out = []
    for child in sorted((ROOT / 'src').iterdir()):
        if child.is_dir() and child.name.startswith('mod101_tool_'):
            out.append(child.name[len('mod101_tool_'):])
    return out


def read_tool() -> str:
    m = TOOL_ARG_RE.search(CONFIG.read_text())
    if not m:
        raise RuntimeError(f'Could not find <xacro:arg name="tool"> in {CONFIG}.')
    return m['val']


def write_tool(name: str) -> None:
    available = list_tools()
    if name not in available:
        raise ValueError(f'Unknown tool {name!r}. Available: {available}')

    def sub(m: re.Match) -> str:
        return f'{m.group(1)}{name}{m.group(3)}'

    new_text, n = TOOL_ARG_RE.subn(sub, CONFIG.read_text())
    if n != 1:
        raise RuntimeError(f'Expected 1 replacement for tool arg, did {n}')
    CONFIG.write_text(new_text)


MOUNTS = ('small', 'big')


def read_props() -> dict:
    """Return the four build args: two lengths (m) + two mounts."""
    text = CONFIG.read_text()
    lens: dict[str, float] = {}
    for m in LEN_ARG_RE.finditer(text):
        lens[m['name']] = float(m['val'])
    mounts: dict[str, str] = {}
    for m in MOUNT_ARG_RE.finditer(text):
        mounts[m['name']] = m['val']
    if {'shoulder_ext_length', 'elbow_ext_length'} - lens.keys():
        raise RuntimeError(f'Both extrusion length args not found in {CONFIG}.')
    if {'shoulder_mount', 'elbow_mount'} - mounts.keys():
        raise RuntimeError(f'Both mount args not found in {CONFIG}.')
    return {
        'shoulder': lens['shoulder_ext_length'],
        'elbow': lens['elbow_ext_length'],
        'shoulder_mount': mounts['shoulder_mount'],
        'elbow_mount': mounts['elbow_mount'],
    }


def write_props(shoulder_m: float, elbow_m: float,
                shoulder_mount: str, elbow_mount: str) -> None:
    for label, v in (('shoulder', shoulder_m), ('elbow', elbow_m)):
        if not (0.05 <= v <= 0.40):
            raise ValueError(f'{label} length {v} m outside [0.05, 0.40]')
    for label, mnt in (('shoulder', shoulder_mount), ('elbow', elbow_mount)):
        if mnt not in MOUNTS:
            raise ValueError(f'{label} mount {mnt!r} not in {MOUNTS}')

    lengths = {'shoulder_ext_length': shoulder_m, 'elbow_ext_length': elbow_m}
    mounts = {'shoulder_mount': shoulder_mount, 'elbow_mount': elbow_mount}

    text = CONFIG.read_text()
    text, n = LEN_ARG_RE.subn(
        lambda m: f'{m.group(1)}{lengths[m["name"]]:.4f}{m.group(4)}', text)
    if n != 2:
        raise RuntimeError(f'Expected 2 length replacements, did {n}')
    text, n = MOUNT_ARG_RE.subn(
        lambda m: f'{m.group(1)}{mounts[m["name"]]}{m.group(4)}', text)
    if n != 2:
        raise RuntimeError(f'Expected 2 mount replacements, did {n}')
    CONFIG.write_text(text)



# ---------------------------------------------------------------------------
# Self-collision matrix
# ---------------------------------------------------------------------------
# Changing a rail length or a mount moves the arm's reach, which changes which
# link pairs can never touch. mod101's own matrix derives from the build params,
# so a Save invalidates it — but does NOT rebuild it. Writing the xacro is
# instant and rebuilding takes minutes, so they are separate actions with
# separate buttons, and `stale` tells the UI when the second one is owed.
#
# THIS REGENERATES mod101 AND NOTHING ELSE. It used to reach into a base101
# workspace as well, which inverted the dependency: the arm knew about a robot
# that embeds it. A consumer can live anywhere, be absent, be unbuilt, or not be
# base101 at all — so each consumer owns its own regeneration and the
# configurator only tells you to run it. See DOWNSTREAM_NOTE.
REGEN_TIMEOUT = 900

# Surfaced in /save, /tool and /collisions responses so the page can tell you
# what still needs doing. A pointer, deliberately not an integration.
DOWNSTREAM_NOTE = (
    'Robots that embed the mod101_arm macro keep their own self-collision '
    'matrices and are NOT regenerated from here. Re-run that workspace\'s sync '
    'script after this change. For base101 that is '
    '`base101_arm_moveit_config/scripts/sync_arm_change.sh`.'
)

_regen_lock = threading.Lock()
_regen_state = {'running': False, 'last': None}


def _regen_targets():
    """(label, script, workspace-setup) for each matrix generator that exists."""
    return [('mod101', ROOT / 'tools' / 'gen_collision_matrix.py', ROOT)]


# The configurator asks for a thorough run, not the generator's quick default.
# See MATRIX_TRIALS in gen_collision_matrix.py's docstring for the convergence
# data: at 10,000 the matrix wrongly disables pairs that really can collide,
# which is the dangerous direction. A Save is rare and now has its own button,
# so it can afford to be slow and right.
REGEN_TRIALS = 1_000_000


def _regen_worker(trials=REGEN_TRIALS):
    results = {}
    for label, script, ws in _regen_targets():
        setup = ws / 'install' / 'setup.bash'
        if not setup.is_file():
            results[label] = 'skipped: workspace not built'
            print(f'[collisions] {label}: {results[label]}')
            continue
        # A login shell so the generator sees ROS and this workspace's overlay.
        cmd = (f'source /opt/ros/$ROS_DISTRO/setup.bash && '
               f'source {setup} && python3 {script} --trials {trials}')
        try:
            proc = subprocess.run(['bash', '-lc', cmd], capture_output=True,
                                  text=True, timeout=REGEN_TIMEOUT)
            results[label] = 'ok' if proc.returncode == 0 else \
                f'failed ({proc.returncode})'
            if proc.returncode != 0:
                print(f'[collisions] {label} failed:\n{proc.stderr[-2000:]}')
            else:
                print(f'[collisions] {label}: regenerated')
        except subprocess.TimeoutExpired:
            results[label] = 'timed out'
            print(f'[collisions] {label}: timed out after {REGEN_TIMEOUT}s')
    with _regen_lock:
        _regen_state['running'] = False
        _regen_state['last'] = results


MATRIX_DIR = SRC / 'mod101_moveit_config' / 'config' / 'collisions'
STAMP_RE = re.compile(
    r'shoulder_ext_length=(?P<sl>[\d.]+)\s+elbow_ext_length=(?P<el>[\d.]+)\s+'
    r'shoulder_mount=(?P<sm>\w+)\s+elbow_mount=(?P<em>\w+)')


def matrices_stale() -> bool | None:
    """Do the generated matrices still match the args on disk?

    Read from the generated files' own build stamp rather than remembering
    whether a Save happened, so it survives a server restart and is still right
    if someone edits the xacro by hand. None = can't tell (no matrices yet).
    """
    try:
        cur = read_props()
    except Exception:
        return None
    for f in sorted(MATRIX_DIR.glob('*.srdf.xacro')):
        m = STAMP_RE.search(f.read_text())
        if not m:
            return None
        if (abs(float(m['sl']) - cur['shoulder']) > 1e-6
                or abs(float(m['el']) - cur['elbow']) > 1e-6
                or m['sm'] != cur['shoulder_mount']
                or m['em'] != cur['elbow_mount']):
            return True
    return False


def regen_collisions_async(trials: int = REGEN_TRIALS):
    """Kick off regeneration of mod101's own matrices unless one is in flight.

    Consumers are never touched — `downstream` is what you have to run yourself.
    """
    with _regen_lock:
        if _regen_state['running']:
            return {'running': True, 'note': 'already regenerating',
                    'downstream': DOWNSTREAM_NOTE, 'trials': trials}
        _regen_state['running'] = True
    threading.Thread(target=_regen_worker, args=(trials,), daemon=True).start()
    return {'running': True,
            'targets': [label for label, _, _ in _regen_targets()],
            'downstream': DOWNSTREAM_NOTE, 'trials': trials}


def regen_status():
    with _regen_lock:
        state = dict(_regen_state)
    return {**state, 'downstream': DOWNSTREAM_NOTE,
            'stale': matrices_stale(), 'trials': REGEN_TRIALS}


def read_masses() -> dict:
    """Part C output. {'masses': {...}} or {'masses': None} if not generated."""
    if not MASSES.is_file():
        return {'masses': None}
    return {'masses': json.loads(MASSES.read_text())}


def read_calibration() -> dict:
    """Return whatever calibration artifacts exist on disk (None when absent)."""
    return {
        'yaml': CAL_YAML.read_text() if CAL_YAML.is_file() else None,
        'lerobot': (json.loads(CAL_LEROBOT.read_text())
                    if CAL_LEROBOT.is_file() else None),
    }


def write_calibration(yaml_text: str, lerobot: dict) -> list[str]:
    """Persist the wizard's output. `yaml_text` is rendered client-side so the
    page's preview and the file on disk can't drift; we only sanity-check it."""
    if not isinstance(yaml_text, str) or 'mod101_calibration:' not in yaml_text:
        raise ValueError('yaml payload missing the mod101_calibration root key')
    if not isinstance(lerobot, dict) or not lerobot:
        raise ValueError('lerobot payload must be a non-empty object')
    for name, entry in lerobot.items():
        missing = {'id', 'drive_mode', 'homing_offset',
                   'range_min', 'range_max'} - set(entry)
        if missing:
            raise ValueError(f'{name}: missing {sorted(missing)}')

    CAL_DIR.mkdir(parents=True, exist_ok=True)
    CAL_YAML.write_text(yaml_text)
    CAL_LEROBOT.write_text(json.dumps(lerobot, indent=2) + '\n')
    return [str(p.relative_to(ROOT)) for p in (CAL_YAML, CAL_LEROBOT)]


_SRC_OVERLAY: str | None = None


def src_ament_overlay() -> str:
    """Build (once) a temp ament prefix whose share/<pkg> symlinks to each src
    package, so `$(find <pkg>)` in xacro resolves to LIVE src/ — independent of
    whether the colcon workspace is built, fresh, or even sourced. This is what
    makes the configurator always render the current parameterized URDF instead
    of a stale install/ copy.

    A src package laid out as src/<pkg>/{urdf,meshes,config} matches the share/
    layout ament expects, so the symlink Just Works for includes and meshes.
    """
    global _SRC_OVERLAY
    if _SRC_OVERLAY is not None:
        return _SRC_OVERLAY

    prefix = Path(tempfile.mkdtemp(prefix='mod101-cfg-overlay-'))
    share = prefix / 'share'
    marker_dir = share / 'ament_index' / 'resource_index' / 'packages'
    marker_dir.mkdir(parents=True, exist_ok=True)
    for pkg_dir in sorted(SRC.iterdir()):
        if not (pkg_dir / 'package.xml').is_file():
            continue
        pkg = pkg_dir.name
        (share / pkg).symlink_to(pkg_dir.resolve())  # share/<pkg> -> src/<pkg>
        (marker_dir / pkg).write_text('')            # ament package marker
    atexit.register(shutil.rmtree, prefix, ignore_errors=True)
    _SRC_OVERLAY = str(prefix)
    return _SRC_OVERLAY


ALLOWED_ARGS = ('shoulder_ext_length', 'elbow_ext_length',
                'shoulder_mount', 'elbow_mount')


def expand_urdf(mappings: dict | None = None) -> str:
    """Run `xacro` (resolving packages from src/) and rewrite mesh URIs to web
    paths. `mappings` overrides the build args for a LIVE preview without
    writing the file (only the four ALLOWED_ARGS are honoured)."""
    xacro_bin = shutil.which('xacro')
    if not xacro_bin:
        raise RuntimeError(
            "xacro CLI not on PATH. Source ROS first:\n"
            "    source /opt/ros/jazzy/setup.bash"
        )
    # Prepend the src overlay so $(find <pkg>) -> src/<pkg>, ahead of any built
    # install space. No `source install/setup.bash` needed.
    env = dict(os.environ)
    overlay = src_ament_overlay()
    existing = env.get('AMENT_PREFIX_PATH', '')
    env['AMENT_PREFIX_PATH'] = overlay + (os.pathsep + existing if existing else '')

    arg_pairs = []
    for k, v in (mappings or {}).items():
        if k in ALLOWED_ARGS and v not in (None, ''):
            arg_pairs.append(f'{k}:={v}')

    proc = subprocess.run(
        [xacro_bin, str(XACRO), *arg_pairs],
        capture_output=True, text=True, env=env,
    )
    if proc.returncode != 0:
        raise RuntimeError(f'xacro failed: {proc.stderr.strip()}')
    # Mesh URI styles to rewrite to web paths, all -> /pkg/<pkg>/meshes/foo.stl:
    #   file:///.../share/<pkg>/meshes/foo.stl   (install space OR src overlay —
    #                                             both end in /share/<pkg>/meshes)
    #   file:///.../src/<pkg>/meshes/foo.stl      (if $(find) resolves to src)
    #   package://<pkg>/meshes/foo.stl
    urdf = proc.stdout
    urdf = re.sub(
        r'file://[^"]*/share/([^/]+)/meshes/([^"]+)',
        r'/pkg/\1/meshes/\2', urdf)
    urdf = re.sub(
        r'file://[^"]*/src/([^/]+)/meshes/([^"]+)',
        r'/pkg/\1/meshes/\2', urdf)
    urdf = re.sub(
        r'package://([^/]+)/meshes/',
        r'/pkg/\1/meshes/', urdf)
    return urdf


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(HERE), **kwargs)

    def end_headers(self):
        # Every response, including the static ones. SimpleHTTPRequestHandler
        # sends Last-Modified but no Cache-Control, and Chrome then heuristically
        # caches the page and viewer.js — so an edit to either lands in the repo
        # and not in the browser, which reads as "my change did nothing".
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def _json(self, status: int, payload: dict) -> None:
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _text(self, status: int, body: str, ctype: str) -> None:
        b = body.encode()
        self.send_response(status)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def _file(self, path: Path, ctype: str) -> None:
        data = path.read_bytes()
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path.rstrip('/') == '/load':
            try:    return self._json(200, read_props())
            except Exception as e: return self._json(500, {'error': str(e)})

        if path.rstrip('/') == '/masses':
            try:    return self._json(200, read_masses())
            except Exception as e: return self._json(500, {'error': str(e)})

        if path.rstrip('/') == '/collisions':
            # Regeneration is fire-and-forget from /save and /tool; this lets
            # the page show progress without blocking the save.
            return self._json(200, regen_status())

        if path.rstrip('/') == '/tool':
            try:    return self._json(200, {'tool': read_tool(), 'available': list_tools()})
            except Exception as e: return self._json(500, {'error': str(e)})

        if path.rstrip('/') == '/calibration':
            try:    return self._json(200, read_calibration())
            except Exception as e: return self._json(500, {'error': str(e)})

        if path.rstrip('/') == '/bus':
            return self._json(200, BUS.status())

        if path.rstrip('/') == '/bus/ports':
            try:    return self._json(200, {'ports': BUS.list_ports()})
            except BusError as e:   return self._json(409, {'error': str(e)})
            except Exception as e:  return self._json(500, {'error': str(e)})

        if path.startswith('/servo/'):
            parts = path[len('/servo/'):].strip('/').split('/')
            if len(parts) == 1 and parts[0].isdigit():
                try:    return self._json(200, BUS.telemetry(int(parts[0])))
                except BusError as e:   return self._json(409, {'error': str(e)})
                except Exception as e:  return self._json(500, {'error': str(e)})
            return self._json(400, {'error': 'expected GET /servo/<id>'})

        if path.rstrip('/') == '/urdf':
            try:
                from urllib.parse import parse_qs
                q = parse_qs(self.path.split('?', 1)[1]) if '?' in self.path else {}
                mappings = {k: v[0] for k, v in q.items()}
                return self._text(200, expand_urdf(mappings), 'application/xml')
            except Exception as e: return self._json(500, {'error': str(e)})

        if path.startswith('/pkg/'):
            parts = path[len('/pkg/'):].split('/', 2)
            if len(parts) == 3 and parts[1] == 'meshes':
                pkg, _, name = parts
                base = SRC / pkg / 'meshes'
                target = (base / name).resolve()
                if base.resolve() not in target.parents and target != base.resolve():
                    return self._json(403, {'error': 'path traversal blocked'})
                if not target.is_file():
                    return self._json(404, {'error': f'{pkg}/meshes/{name} not found'})
                ctype = 'model/stl' if name.endswith('.stl') else 'application/octet-stream'
                return self._file(target, ctype)
            return self._json(400, {'error': 'expected /pkg/<pkgname>/meshes/<file>'})

        return super().do_GET()

    def do_POST(self):
        path = self.path.rstrip('/')
        try:
            length = int(self.headers.get('Content-Length', '0'))
            # A bodyless POST is legitimate here: the page's unload handler
            # fires /bus/disarm through navigator.sendBeacon.
            data = json.loads(self.rfile.read(length)) if length else {}
        except Exception as e:
            return self._json(400, {'error': f'bad json: {e}'})

        if path == '/save':
            try:
                # mounts default to whatever's on disk if the client omits them
                # (lets a length-only save not disturb the motor selection).
                cur = read_props()
                write_props(
                    float(data['shoulder']), float(data['elbow']),
                    str(data.get('shoulder_mount', cur['shoulder_mount'])),
                    str(data.get('elbow_mount', cur['elbow_mount'])))
                # Deliberately does NOT regenerate. Writing the xacro is
                # instant; rebuilding the matrices takes minutes at
                # REGEN_TRIALS. They are separate buttons so the cost is
                # visible instead of hiding inside a Save that looks cheap.
                return self._json(200, {'ok': True, **read_props(),
                                        'collisions': regen_status()})
            except Exception as e:
                return self._json(400, {'error': str(e)})

        if path == '/tool':
            try:
                write_tool(str(data['tool']))
                # No regeneration: the generator writes one matrix per tool on
                # every run, all four stamped with the current rails/mounts.
                # Switching tool selects an already-current file.
                return self._json(200, {'ok': True, 'tool': read_tool(),
                                        'available': list_tools(),
                                        'collisions': regen_status()})
            except Exception as e:
                return self._json(400, {'error': str(e)})

        if path == '/collisions/regen':
            try:
                trials = int(data.get('trials', REGEN_TRIALS))
                return self._json(200, regen_collisions_async(trials))
            except Exception as e:
                return self._json(400, {'error': str(e)})

        if path == '/calibration':
            try:
                written = write_calibration(data.get('yaml'), data.get('lerobot'))
                return self._json(200, {'ok': True, 'written': written})
            except Exception as e:
                return self._json(400, {'error': str(e)})

        # ---- servo bus ----------------------------------------------------
        # BusError is the "you asked for something the bus can't do right now"
        # case (not connected, servo didn't ack) -> 409, not a 500.
        if path.startswith('/bus') or path.startswith('/servo/'):
            try:
                return self._bus_post(path, data)
            except BusError as e:
                return self._json(409, {'error': str(e)})
            except Exception as e:
                return self._json(500, {'error': str(e)})

        return self._json(404, {'error': 'unknown endpoint'})

    def _bus_post(self, path: str, data: dict):
        if path == '/bus/connect':
            port = str(data.get('port') or '').strip()
            if not port:
                raise BusError('port is required')
            return self._json(200, {'ok': True, **BUS.connect(port)})

        if path == '/bus/disconnect':
            BUS.disconnect()
            return self._json(200, {'ok': True, **BUS.status()})

        if path == '/bus/scan':
            return self._json(200, {'ok': True, **BUS.scan()})

        if path == '/bus/disarm':
            return self._json(200, {'ok': True, **BUS.disarm_all()})

        if path.startswith('/servo/'):
            parts = path[len('/servo/'):].strip('/').split('/')
            if len(parts) == 2 and parts[0].isdigit():
                sid, action = int(parts[0]), parts[1]
                if action == 'torque':
                    BUS.torque(sid, bool(data.get('on')))
                    return self._json(200, {'ok': True})
                if action == 'move':
                    BUS.move(sid, int(data['pos']),
                             data.get('speed'), data.get('accel'))
                    return self._json(200, {'ok': True})
                if action == 'profile':
                    BUS.profile(sid, int(data['speed']), int(data['accel']))
                    return self._json(200, {'ok': True})
                if action == 'id':
                    BUS.change_id(sid, int(data['to']))
                    return self._json(200, {'ok': True, **BUS.status()})

        return self._json(404, {'error': f'unknown bus endpoint {path}'})


class Server(socketserver.TCPServer):
    # Without SO_REUSEADDR the listening socket sits in TIME_WAIT for ~60 s
    # after a Ctrl-C, and an immediate restart dies with "Address already in
    # use" — which reads like a second server is running when none is.
    allow_reuse_address = True


def main() -> None:
    for required in (XACRO, CONFIG):
        if not required.exists():
            sys.exit(f'{required} not found')
    with Server(('', PORT), Handler) as httpd:
        print(f'mod101 configurator: http://localhost:{PORT}/')
        print(f'editing  {CONFIG}')
        print(f'tools    {list_tools()}')
        try:    httpd.serve_forever()
        except KeyboardInterrupt: print()


if __name__ == '__main__':
    main()
