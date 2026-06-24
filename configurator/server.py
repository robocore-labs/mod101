#!/usr/bin/env python3
"""mod101 configurator backend.

Serves configurator/ as static files. Endpoints:
  GET  /load              - read the four build args (shoulder/elbow rail
                            length + shoulder/elbow mount) from
                            src/mod101_description/urdf/mod101.xacro
  POST /save              - write those four values back in-place
  GET  /masses            - return src/mod101_description/link_masses.json
                            (Part C output) so the page uses real printed masses
  GET  /tool, POST /tool  - active end-effector package + discovery
  GET  /urdf              - run `xacro` and return the expanded URDF, with
                            mesh URIs rewritten to /pkg/<pkg>/meshes/<file>
  GET  /pkg/<pkg>/meshes/<file> - serve binary mesh files from any package

Stdlib only. Run from project root:
    python3 configurator/server.py
"""

import atexit
import http.server
import json
import os
import re
import shutil
import socketserver
import subprocess
import sys
import tempfile
from pathlib import Path

PORT = 8000
HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
SRC  = ROOT / 'src'
DESC = SRC / 'mod101_description'
XACRO = DESC / 'urdf' / 'mod101.xacro'

MASSES = DESC / 'link_masses.json'

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
    m = TOOL_ARG_RE.search(XACRO.read_text())
    if not m:
        raise RuntimeError(f'Could not find <xacro:arg name="tool"> in {XACRO}.')
    return m['val']


def write_tool(name: str) -> None:
    available = list_tools()
    if name not in available:
        raise ValueError(f'Unknown tool {name!r}. Available: {available}')

    def sub(m: re.Match) -> str:
        return f'{m.group(1)}{name}{m.group(3)}'

    new_text, n = TOOL_ARG_RE.subn(sub, XACRO.read_text())
    if n != 1:
        raise RuntimeError(f'Expected 1 replacement for tool arg, did {n}')
    XACRO.write_text(new_text)


MOUNTS = ('small', 'big')


def read_props() -> dict:
    """Return the four build args: two lengths (m) + two mounts."""
    text = XACRO.read_text()
    lens: dict[str, float] = {}
    for m in LEN_ARG_RE.finditer(text):
        lens[m['name']] = float(m['val'])
    mounts: dict[str, str] = {}
    for m in MOUNT_ARG_RE.finditer(text):
        mounts[m['name']] = m['val']
    if {'shoulder_ext_length', 'elbow_ext_length'} - lens.keys():
        raise RuntimeError(f'Both extrusion length args not found in {XACRO}.')
    if {'shoulder_mount', 'elbow_mount'} - mounts.keys():
        raise RuntimeError(f'Both mount args not found in {XACRO}.')
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

    text = XACRO.read_text()
    text, n = LEN_ARG_RE.subn(
        lambda m: f'{m.group(1)}{lengths[m["name"]]:.4f}{m.group(4)}', text)
    if n != 2:
        raise RuntimeError(f'Expected 2 length replacements, did {n}')
    text, n = MOUNT_ARG_RE.subn(
        lambda m: f'{m.group(1)}{mounts[m["name"]]}{m.group(4)}', text)
    if n != 2:
        raise RuntimeError(f'Expected 2 mount replacements, did {n}')
    XACRO.write_text(text)


def read_masses() -> dict:
    """Part C output. {'masses': {...}} or {'masses': None} if not generated."""
    if not MASSES.is_file():
        return {'masses': None}
    return {'masses': json.loads(MASSES.read_text())}


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


def expand_urdf() -> str:
    """Run `xacro` (resolving packages from src/) and rewrite mesh URIs to web
    paths."""
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

    proc = subprocess.run(
        [xacro_bin, str(XACRO)],
        capture_output=True, text=True, env=env,
    )
    if proc.returncode != 0:
        raise RuntimeError(f'xacro failed: {proc.stderr.strip()}')
    # Mesh URI styles to rewrite to web paths (any of):
    #   file:///abs/.../<pkg>/share/<pkg>/meshes/foo.stl  (built install space)
    #   file:///abs/.../src/<pkg>/meshes/foo.stl          (src overlay)
    #   package://<pkg>/meshes/foo.stl
    # all -> /pkg/<pkg>/meshes/foo.stl
    urdf = proc.stdout
    urdf = re.sub(
        r'file://[^"]*/([^/]+)/share/\1/meshes/([^"]+)',
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

    def _json(self, status: int, payload: dict) -> None:
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _text(self, status: int, body: str, ctype: str) -> None:
        b = body.encode()
        self.send_response(status)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(b)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(b)

    def _file(self, path: Path, ctype: str) -> None:
        data = path.read_bytes()
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
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

        if path.rstrip('/') == '/tool':
            try:    return self._json(200, {'tool': read_tool(), 'available': list_tools()})
            except Exception as e: return self._json(500, {'error': str(e)})

        if path.rstrip('/') == '/urdf':
            try:    return self._text(200, expand_urdf(), 'application/xml')
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
            data = json.loads(self.rfile.read(length))
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
                return self._json(200, {'ok': True, **read_props()})
            except Exception as e:
                return self._json(400, {'error': str(e)})

        if path == '/tool':
            try:
                write_tool(str(data['tool']))
                return self._json(200, {'ok': True, 'tool': read_tool(),
                                        'available': list_tools()})
            except Exception as e:
                return self._json(400, {'error': str(e)})

        return self._json(404, {'error': 'unknown endpoint'})


def main() -> None:
    if not XACRO.exists():
        sys.exit(f'{XACRO} not found')
    with socketserver.TCPServer(('', PORT), Handler) as httpd:
        print(f'mod101 configurator: http://localhost:{PORT}/')
        print(f'editing  {XACRO}')
        print(f'tools    {list_tools()}')
        try:    httpd.serve_forever()
        except KeyboardInterrupt: print()


if __name__ == '__main__':
    main()
