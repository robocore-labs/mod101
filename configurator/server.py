#!/usr/bin/env python3
"""mod101 configurator backend.

Serves configurator/ as static files. Endpoints:
  GET  /load              - read shoulder_ext_length and elbow_ext_length
                            from src/mod101_description/urdf/mod101.xacro
  POST /save              - write those two values back in-place
  GET  /urdf              - run `xacro` and return the expanded URDF, with
                            mesh URIs rewritten to /meshes/<file>
  GET  /meshes/<file>     - serve binary mesh files from the description pkg

Stdlib only. Run from project root:
    python3 configurator/server.py
"""

import http.server
import json
import re
import shutil
import socketserver
import subprocess
import sys
from pathlib import Path

PORT = 8000
HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
SRC  = ROOT / 'src'
DESC = SRC / 'mod101_description'
XACRO = DESC / 'urdf' / 'mod101.xacro'

PROP_RE = re.compile(
    r'(<xacro:property\s+name="(?P<name>shoulder_ext_length|elbow_ext_length)"\s+value=")'
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


def read_props() -> dict[str, float]:
    out: dict[str, float] = {}
    for m in PROP_RE.finditer(XACRO.read_text()):
        out[m['name']] = float(m['val'])
    if {'shoulder_ext_length', 'elbow_ext_length'} - out.keys():
        raise RuntimeError(
            f'Both extrusion length properties not found in {XACRO}.'
        )
    return {'shoulder': out['shoulder_ext_length'], 'elbow': out['elbow_ext_length']}


def write_props(shoulder_m: float, elbow_m: float) -> None:
    for label, v in (('shoulder', shoulder_m), ('elbow', elbow_m)):
        if not (0.05 <= v <= 0.40):
            raise ValueError(f'{label} length {v} m outside [0.05, 0.40]')
    targets = {'shoulder_ext_length': shoulder_m, 'elbow_ext_length': elbow_m}

    def sub(m: re.Match) -> str:
        return f'{m.group(1)}{targets[m["name"]]:.4f}{m.group(4)}'

    new_text, n = PROP_RE.subn(sub, XACRO.read_text())
    if n != 2:
        raise RuntimeError(f'Expected 2 replacements, did {n}')
    XACRO.write_text(new_text)


def expand_urdf() -> str:
    """Run `xacro` and rewrite mesh URIs to web-accessible paths."""
    xacro_bin = shutil.which('xacro')
    if not xacro_bin:
        raise RuntimeError(
            "xacro CLI not on PATH. Source ROS first:\n"
            "    source /opt/ros/jazzy/setup.bash && source install/setup.bash"
        )
    proc = subprocess.run(
        [xacro_bin, str(XACRO)],
        capture_output=True, text=True
    )
    if proc.returncode != 0:
        raise RuntimeError(f'xacro failed: {proc.stderr.strip()}')
    # Two mesh URI styles to rewrite to web paths:
    #   file:///abs/.../<pkg>/share/<pkg>/meshes/foo.stl → /pkg/<pkg>/meshes/foo.stl
    #   package://<pkg>/meshes/foo.stl                   → /pkg/<pkg>/meshes/foo.stl
    urdf = proc.stdout
    urdf = re.sub(
        r'file://[^"]*/([^/]+)/share/\1/meshes/([^"]+)',
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
                write_props(float(data['shoulder']), float(data['elbow']))
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
