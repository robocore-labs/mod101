#!/usr/bin/env python3
"""Estimate 3D-printed link masses from STL geometry (PLA-CF, sparse infill).

Why this exists
---------------
The configurator and URDF need *real* masses for the printed brackets/bodies,
not guesses. A sliced print is a dense shell (perimeters + solid top/bottom
skins) wrapped around a sparse infill lattice, so its mass is well below the
"solid volume x filament density" a naive estimate gives. This script models
that:

    V_model  = watertight mesh volume                 # cm^3
    A_model  = mesh surface area                       # cm^2
    V_shell  = min(V_model, A_model * T_WALL)          # cm^3  (perimeters+skins)
    V_infill = max(0, V_model - V_shell)
    mass_g   = RHO_PLACF * (V_shell + INFILL * V_infill)

`V_shell = A_model * T_WALL` treats the shell as a uniform skin of thickness
T_WALL over the whole surface; clamping with `min(V_model, ...)` keeps thin
parts (where the shell would "overflow" the part) from going over solid mass.

Calibration (do not ship uncalibrated)
--------------------------------------
The shell model is first-order. Two ways to ground it:

  1. Weigh ONE real print of a known part, then solve for RHO_PLACF (or apply
     an overall fudge factor) so this script matches the scale; reuse for all.
  2. Better: read the slicer's predicted grams for each part sliced at the real
     profile (PLA-CF, 15%, actual walls) -- that's ground truth. Pass them via
     `--from-slicer slicer_masses.csv` (columns: name,grams) and this script
     uses those directly, falling back to the geometric estimate only for parts
     the CSV doesn't list.

Sanity check: a calibration cube printed SOLID (`--infill 1.0`) returns
mass == RHO * V exactly, since the infill term then accounts for the full
interior. The same holds at default infill for any part thin enough that the
shell saturates (A*T_WALL >= V, i.e. wall-to-wall solid), e.g. a <~10 mm cube
at T_WALL=0.16 cm.

Usage
-----
    python3 tools/estimate_masses.py src/mod101_description/meshes \\
        -o src/mod101_description/link_masses.json

    # tune the print profile
    python3 tools/estimate_masses.py MESHES --rho 1.24 --infill 0.15 --wall 0.16

    # ingest slicer ground-truth where available
    python3 tools/estimate_masses.py MESHES --from-slicer slicer_masses.csv
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from pathlib import Path

try:
    import trimesh
except ImportError:
    sys.exit(
        "trimesh is required: pip install trimesh\n"
        "(it pulls in numpy; no other heavy deps needed for STL volume/area)."
    )

# --- defaults (CLI-overridable) ---------------------------------------------
RHO_PLACF = 1.24   # g/cm^3  printed-effective PLA-CF density (calibrate!)
INFILL = 0.15      # 15% infill, per spec
T_WALL = 0.16      # cm -> 1.6 mm shell (4 perimeters @0.4 mm, folds in skins)

# Non-printed parts: bought servos, cut aluminium extrusion, and electronics.
# Everything else under meshes/ is treated as a printed part. Matched
# case-insensitively against the stem (filename without extension).
NON_PRINTED_PATTERNS = (
    r"^servo_",        # servo_* -> bought hobby servo BODIES (anchored: keeps
                       #   printed *_servo_adapter_* parts, which DO print)
    r"extrusion",      # *extrusion* -> cut 2020 aluminium, handled analytically
    r"wrist_camera",   # wrist_camera_v1 -> camera module (electronics)
    r"_pcb",           # *_pcb -> bare PCB / electronics
)


def is_printed(stem: str) -> bool:
    s = stem.lower()
    return not any(re.search(p, s) for p in NON_PRINTED_PATTERNS)


def estimate_one(mesh_path: Path, rho: float, infill: float, t_wall: float):
    """Return (mass_g, V_cm3, shell_frac, watertight) for one STL.

    Units: STL assumed in mm; volume/area are converted to cm^3 / cm^2.
    """
    mesh = trimesh.load(mesh_path, force="mesh")

    watertight = bool(mesh.is_watertight)
    if not watertight:
        # Attempt a repair; warn loudly either way -- an open mesh gives a
        # meaningless (often negative) volume.
        mesh.fill_holes()
        watertight = bool(mesh.is_watertight)

    # trimesh reports volume in the mesh's own units^3 (mm^3 here), area in
    # units^2 (mm^2). Convert: 1 cm^3 = 1000 mm^3, 1 cm^2 = 100 mm^2.
    v_cm3 = abs(float(mesh.volume)) / 1000.0
    a_cm2 = float(mesh.area) / 100.0

    v_shell = min(v_cm3, a_cm2 * t_wall)
    v_infill = max(0.0, v_cm3 - v_shell)
    mass_g = rho * (v_shell + infill * v_infill)
    shell_frac = (v_shell / v_cm3) if v_cm3 > 0 else 0.0

    return mass_g, v_cm3, shell_frac, watertight


def load_slicer_csv(path: Path) -> dict[str, float]:
    """name -> grams, from a slicer export. Header optional; matches on stem."""
    out: dict[str, float] = {}
    with path.open(newline="") as f:
        for row in csv.reader(f):
            if not row or len(row) < 2:
                continue
            name, grams = row[0].strip(), row[1].strip()
            if name.lower() in ("name", "part", "file"):  # header row
                continue
            try:
                out[Path(name).stem] = float(grams)
            except ValueError:
                continue
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("meshes_dir", type=Path, help="directory of STL meshes")
    ap.add_argument("-o", "--output", type=Path, default=None,
                    help="write link_masses.json here (default: <meshes_dir>/../link_masses.json)")
    ap.add_argument("--rho", type=float, default=RHO_PLACF, help=f"filament density g/cm^3 (default {RHO_PLACF})")
    ap.add_argument("--infill", type=float, default=INFILL, help=f"infill fraction (default {INFILL})")
    ap.add_argument("--wall", type=float, default=T_WALL, help=f"shell thickness cm (default {T_WALL})")
    ap.add_argument("--units", choices=("mm", "cm"), default="mm",
                    help="STL units (default mm; cm meshes are scaled down)")
    ap.add_argument("--printed-only", action="store_true",
                    help="skip servo / extrusion / camera meshes (non-printed)")
    ap.add_argument("--from-slicer", type=Path, default=None,
                    help="CSV of name,grams from the slicer; used in place of the estimate where present")
    args = ap.parse_args(argv)

    if not args.meshes_dir.is_dir():
        sys.exit(f"not a directory: {args.meshes_dir}")

    # cm meshes: trimesh loads in file units; we assume mm, so a cm file needs
    # x10 to become mm before the mm->cm conversion in estimate_one.
    cm_to_mm = 10.0 if args.units == "cm" else 1.0

    slicer = load_slicer_csv(args.from_slicer) if args.from_slicer else {}

    stls = sorted(p for p in args.meshes_dir.iterdir()
                  if p.suffix.lower() == ".stl")
    if not stls:
        sys.exit(f"no .stl files in {args.meshes_dir}")

    results: dict[str, dict] = {}
    warnings: list[str] = []

    for stl in stls:
        stem = stl.stem
        if args.printed_only and not is_printed(stem):
            continue

        if stem in slicer:
            mass_g = slicer[stem]
            entry = {"mass_g": round(mass_g, 3), "V_cm3": None,
                     "shell_frac": None, "watertight": None, "source": "slicer"}
        else:
            mesh = trimesh.load(stl, force="mesh")
            if cm_to_mm != 1.0:
                mesh.apply_scale(cm_to_mm)
            watertight = bool(mesh.is_watertight)
            if not watertight:
                mesh.fill_holes()
                watertight = bool(mesh.is_watertight)
            v_cm3 = abs(float(mesh.volume)) / 1000.0
            a_cm2 = float(mesh.area) / 100.0
            v_shell = min(v_cm3, a_cm2 * args.wall)
            v_infill = max(0.0, v_cm3 - v_shell)
            mass_g = args.rho * (v_shell + args.infill * v_infill)
            shell_frac = (v_shell / v_cm3) if v_cm3 > 0 else 0.0
            entry = {"mass_g": round(mass_g, 3), "V_cm3": round(v_cm3, 3),
                     "shell_frac": round(shell_frac, 3), "watertight": watertight,
                     "source": "geometry"}
            if not watertight:
                warnings.append(stem)

        results[stem] = entry

    # --- report ----------------------------------------------------------
    name_w = max((len(n) for n in results), default=4)
    print(f"{'part'.ljust(name_w)}  {'mass_g':>8}  {'V_cm3':>8}  {'shell':>6}  watertight  source")
    print("-" * (name_w + 48))
    total = 0.0
    for name, e in results.items():
        total += e["mass_g"]
        v = "" if e["V_cm3"] is None else f"{e['V_cm3']:.2f}"
        sf = "" if e["shell_frac"] is None else f"{e['shell_frac']:.2f}"
        wt = "-" if e["watertight"] is None else ("yes" if e["watertight"] else "NO ")
        print(f"{name.ljust(name_w)}  {e['mass_g']:>8.2f}  {v:>8}  {sf:>6}  {wt:>10}  {e['source']}")
    print("-" * (name_w + 48))
    print(f"{'TOTAL'.ljust(name_w)}  {total:>8.2f} g   ({len(results)} parts)")

    if warnings:
        print(f"\n!! {len(warnings)} non-watertight mesh(es) (volume is unreliable -- "
              f"check/repair these): {', '.join(warnings)}", file=sys.stderr)

    out_path = args.output or (args.meshes_dir.parent / "link_masses.json")
    out_path.write_text(json.dumps(results, indent=2) + "\n")
    print(f"\nwrote {out_path}")


if __name__ == "__main__":
    main()
