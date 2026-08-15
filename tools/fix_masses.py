#!/usr/bin/env python3
"""Replace mod101's steel-density CAD-export link masses with real ones.

WHY
---
The Fusion export assigned **steel (7.85 g/cm^3) to every part**. Check it:
base_link 0.8668 kg / 110.5 cm^3 = 7.85; jaws_body 0.4470 / 57.0 = 7.85.
Printed PLA-CF parts come out 6-11x too heavy, which makes every dynamics
result and every MoveIt effort/acceleration limit wrong.

mod101 already ships `tools/estimate_masses.py` and a generated
`src/mod101_description/link_masses.json` — but that output was **never applied
to the URDF**, so the wrong masses have been live the whole time. This script
estimates AND applies in one pass, specifically so that cannot happen again.

WHAT IT TOUCHES
---------------
Only links whose *implied* density (mass / mesh volume) is steel, ~7.85. Parts
already corrected by hand are left alone:

    servo_* .................. 1.87 g/cm^3  ->  66 g   STS3215, correct
    ST3120 servos ............ 0.210 kg             correct (configurator MOTORS)
    mod101_tool_parallel ..... already corrected
    mod101_tool_pincopen ..... already corrected
    arm/forearm extrusion .... no mesh, analytic; 2020 alu at 0.45 kg/m,
                               82 mm -> 37 g and 98 mm -> 44 g, both match

Printed parts use the shell + sparse-infill model (same as tools/estimate_masses.py).
`BOUGHT_G` overrides parts that are bought rather than printed but were still
exported at steel density — currently just the jaws gripper servo.

Mass and inertia scale together by k = m_new / m_old: geometry and mass
*distribution* are unchanged, only density is wrong, so the whole inertia
tensor scales and the COM origin is untouched.

    python3 tools/fix_masses.py --dry-run
    python3 tools/fix_masses.py
"""
from __future__ import annotations

import argparse
import json
import re
import struct
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

RHO_PLACF = 1.24
INFILL = 0.15
T_WALL = 0.16

STEEL_LO, STEEL_HI = 7.4, 8.3     # implied-density window that marks a bad export

# Bought parts exported at steel density: force the datasheet value instead of
# the printed-shell estimate.
BOUGHT_G = {
    'jaws_servo': 66.0,           # STS3215 on the jaws gripper, same as the arm servos
    # Camera module (electronics), not a print. Its implied density lands in the
    # steel window only by coincidence — 2.1 cm^3 x 7.85 = 16.5 g — and 16.6 g
    # is the value the configurator already assumes (M_CAMERA). Listed here so
    # the shell model does not "correct" a correct number down to 2.6 g.
    'wrist_camera_v1_1': 16.6,
}

MESH_DIRS = [REPO / 'src/mod101_description/meshes'] + [
    REPO / f'src/mod101_tool_{t}/meshes' for t in ('jaws', 'parallel', 'pincopen', 'none')
]


def find_mesh(name: str):
    for d in MESH_DIRS:
        p = d / name
        if p.exists():
            return p
    return None


def read_stl(path: Path):
    data = path.read_bytes()
    if len(data) >= 84:
        n = struct.unpack('<I', data[80:84])[0]
        if 84 + n * 50 == len(data):
            tris, off = [], 84
            for _ in range(n):
                v = struct.unpack('<12f', data[off:off + 48])
                tris.append(((v[3], v[4], v[5]), (v[6], v[7], v[8]),
                             (v[9], v[10], v[11])))
                off += 50
            return tris
    tris, cur = [], []
    for line in data.decode('utf-8', 'replace').splitlines():
        s = line.strip().split()
        if len(s) == 4 and s[0] == 'vertex':
            cur.append((float(s[1]), float(s[2]), float(s[3])))
            if len(cur) == 3:
                tris.append(tuple(cur)); cur = []
    return tris


def volume_area(tris):
    vol = area = 0.0
    for a, b, c in tris:
        cx = b[1]*c[2] - b[2]*c[1]
        cy = b[2]*c[0] - b[0]*c[2]
        cz = b[0]*c[1] - b[1]*c[0]
        vol += a[0]*cx + a[1]*cy + a[2]*cz
        ux, uy, uz = b[0]-a[0], b[1]-a[1], b[2]-a[2]
        vx, vy, vz = c[0]-a[0], c[1]-a[1], c[2]-a[2]
        wx, wy, wz = uy*vz - uz*vy, uz*vx - ux*vz, ux*vy - uy*vx
        area += 0.5*(wx*wx + wy*wy + wz*wz) ** 0.5
    return abs(vol)/6.0, area


LINK_RE = re.compile(r'(<link name="([^"]+)">.*?</link>)', re.S)
MASS_RE = re.compile(r'<mass value="([0-9.eE+-]+)"/>')
INERTIA_RE = re.compile(r'<inertia ([^/]*)/>')


def fmt(x):
    return f'{x:.9g}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    files = sorted(REPO / p for p in subprocess.run(
        ['grep', '-rl', '<mass value', 'src/'],
        cwd=REPO, capture_output=True, text=True).stdout.split())

    rows, results = [], {}

    for f in files:
        text = f.read_text()

        def fix(m):
            block, raw = m.group(1), m.group(2)
            name = raw.replace('${prefix}', '')
            mm = MASS_RE.search(block)
            if not mm:
                return block
            old = float(mm.group(1))
            mesh_m = re.search(r'meshes/([^"]+\.stl)', block)
            mesh = find_mesh(mesh_m.group(1)) if mesh_m else None
            if mesh is None:
                rows.append((f, name, old, old, 'no mesh', None))
                return block
            v_mm3, a_mm2 = volume_area(read_stl(mesh))
            v, a = v_mm3/1000.0, a_mm2/100.0
            dens = old*1000/v if v else 0.0
            if not (STEEL_LO < dens < STEEL_HI):
                rows.append((f, name, old, old, f'ok ({dens:.2f})', v))
                return block
            if name in BOUGHT_G:
                new = BOUGHT_G[name]/1000.0
                src = 'bought'
            else:
                v_sh = min(v, a*T_WALL)
                new = RHO_PLACF*(v_sh + INFILL*max(0.0, v - v_sh))/1000.0
                src = 'printed'
            k = new/old
            rows.append((f, name, old, new, src, v))
            results[name] = {'mass_g': round(new*1000, 2), 'V_cm3': round(v, 2),
                             'source': src}
            block = MASS_RE.sub(f'<mass value="{fmt(new)}"/>', block, count=1)

            def scale(im):
                attrs = im.group(1)
                for key in ('ixx', 'iyy', 'izz', 'ixy', 'iyz', 'ixz'):
                    attrs = re.sub(
                        rf'{key}="([0-9.eE+-]+)"',
                        lambda mo, kk=key: f'{kk}="{fmt(float(mo.group(1))*k)}"',
                        attrs)
                return f'<inertia {attrs}/>'

            return INERTIA_RE.sub(scale, block, count=1)

        out = LINK_RE.sub(fix, text)
        if not args.dry_run and out != text:
            f.write_text(out)

    w = max(len(r[1]) for r in rows)
    print(f"{'file':46s} {'link'.ljust(w)} {'old':>8} {'new':>8} {'x':>6}  class")
    print('-' * (46 + w + 34))
    changed = 0
    for f, name, old, new, src, v in rows:
        rel = str(f.relative_to(REPO))
        ratio = f'{old/new:6.1f}' if new > 0 and abs(old-new) > 1e-9 else '     -'
        if abs(old-new) > 1e-9:
            changed += 1
        print(f'{rel:46s} {name.ljust(w)} {old:8.4f} {new:8.4f} {ratio}  {src}')
    print('-' * (46 + w + 34))
    print(f'{changed} links rewritten, {len(rows)-changed} left alone')

    if not args.dry_run:
        out_json = REPO / 'src/mod101_description/link_masses.json'
        prev = json.loads(out_json.read_text()) if out_json.exists() else {}
        prev.update(results)
        out_json.write_text(json.dumps(prev, indent=2, sort_keys=True) + '\n')
        print(f'updated {out_json.relative_to(REPO)}')
    else:
        print('(dry run - nothing written)')


if __name__ == '__main__':
    main()
