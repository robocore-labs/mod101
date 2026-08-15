# Configurator

`configurator/` is a self-contained static page + stdlib Python backend for
sizing, motoring, and tooling the arm. It edits the four build args in
`src/mod101_description/urdf/mod101.xacro` in place and shows a live three.js
view (`urdf-loader`) that re-renders on every save.

## Running

```bash
cd ~/Work/mod101
source /opt/ros/jazzy/setup.bash && source install/setup.bash   # for /urdf
python3 configurator/server.py
# open http://localhost:8000/
```

Backend (`server.py`) is stdlib-only Python — no Flask, no virtualenv.

## What you can change

The page's four inputs map 1:1 to the URDF's four xacro args:

| input | xacro arg | notes |
|---|---|---|
| **Shoulder / elbow motor** | `shoulder_mount` / `elbow_mount` | STS3215 / STS3250 → `small`; ST3120 → `big` |
| **Shoulder / elbow length** | `shoulder_ext_length` / `elbow_ext_length` | 2020 rail length, 80–280 mm |
| **Tool** | `tool` arg `default` | lists every `mod101_tool_*` package under `src/` |

Picking a **big** (ST3120) motor flips that joint's mount to `big`, which the
URDF and the payload model both treat as heavier servo body **plus** a fixed
downstream lever extension Δ (`delta_shoulder` / `delta_elbow`) — see the
integration spec, Part A3.

### Live readouts

- **Payload** — first-order static balance at the binding (bottleneck) pitch
  joint, for three duty regimes (Duty 70 % rated / Hold 50 % / Peak 100 %) ×
  two reaches (full / 70 %). The headline is duty-cycle at full reach, with the
  binding joint named. Both shoulder and elbow caps are shown so you can see
  which one limits.
- **Mass & reach** — uses **real printed masses** from `link_masses.json`
  (Part C, `tools/estimate_masses.py`) when present; falls back to lumped
  estimates with an "estimated masses" badge otherwise.
- **Estimated cost** — servo-only BOM (base 3215 · shoulder sel · elbow sel ·
  3× wrist/gripper 3215). Placeholder prices until the real BOM lands; **not** a
  purchase price.
- **Reference builds** — t-shirt presets **S** (dual-3250) / **M** (3120
  shoulder + 3250 elbow) / **L** (dual-3120) that set both motor dropdowns.

After changing anything, **Save** writes the four args. **Reload preview**
re-fetches `/urdf` into the 3D view (also auto-fires after Save and tool
changes), so edits to the xacro on disk show up without restarting. The preview
renders directly from `src/` via a temp ament overlay, so it reflects live edits
even if the colcon workspace isn't built or sourced. You can also open
`configurator/index.html` via `file://` for a read-only preview; Save and the
live `/urdf` view need the server.

## Endpoints

| Method + path | Purpose |
|---|---|
| `GET /load` / `POST /save` | the four build args (two lengths + two mounts) |
| `GET /masses` | `link_masses.json` (Part C output) or `{"masses": null}` |
| `GET /tool` / `POST /tool` | active tool + discovered tools |
| `GET /nudge` / `POST /nudge` | big-module mesh alignment offsets — see below |
| `GET /urdf` | runs `xacro` and rewrites mesh URIs to `/pkg/<pkgname>/meshes/<file>` |
| `GET /pkg/<pkgname>/meshes/<file>` | serves binary meshes from any package's `meshes/` dir |

`GET /tool` reads the `tool` xacro arg out of `mod101.xacro` and lists every
`mod101_tool_*` package under `src/`; `POST /tool` rewrites that arg's
`default`. If the arg is missing from `mod101.xacro`, both return HTTP 500 and
the page's tool dropdown comes up empty — that's the symptom to look for.

## The nudge endpoints

`/nudge` reads and writes the `snx/sny/snz` (shoulder) and `enx/eny/enz`
(elbow) properties in `urdf/modules/*_big.xacro`. They shift **all** of a big
module's meshes together — cosmetic only, kinematics unchanged — to absorb a
frame mismatch when the big parts come from a different CAD canvas than the
small ones.

**They should be zero whenever the big parts are exported in the same frame as
the small ones**, which is the case for the current shoulder export. A stale
non-zero nudge left over from an older export is indistinguishable, at a
glance, from a genuinely broken URDF: the whole assembly floats off the arm.
Before touching these, check the module's datums instead — the adapter's AABB
centre should sit on the extrusion axis, and the rotation link should be
centred on its joint axis. `shoulder_big.xacro` lists the exact numbers.

There is no UI for these endpoints; they're a backend affordance, edited by
hand or over HTTP.
