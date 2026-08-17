# Configurator

`configurator/` is a self-contained static page + stdlib Python backend for
sizing, motoring, and tooling the arm. It edits the four build args in
`src/mod101_description/urdf/mod101_config.xacro` in place and shows a live three.js
view (`urdf-loader`) that re-renders on every save.

## Running

```bash
cd ~/robots/mod101
source /opt/ros/jazzy/setup.bash && source install/setup.bash   # for /urdf
python3 configurator/server.py
# open http://localhost:8001/
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
| `GET /collisions` | status of the background self-collision regen |
| `GET /masses` | `link_masses.json` (Part C output) or `{"masses": null}` |
| `GET /tool` / `POST /tool` | active tool + discovered tools |
| `GET /urdf` | runs `xacro` and rewrites mesh URIs to `/pkg/<pkgname>/meshes/<file>` |
| `GET /pkg/<pkgname>/meshes/<file>` | serves binary meshes from any package's `meshes/` dir |

`GET /tool` reads the `tool` xacro arg out of `mod101_config.xacro` and lists every
`mod101_tool_*` package under `src/`; `POST /tool` rewrites that arg's
`default`. If the arg is missing from `mod101_config.xacro`, both return HTTP 500 and
the page's tool dropdown comes up empty — that's the symptom to look for.

## Big-module alignment offsets

`urdf/modules/shoulder_big.xacro` and `elbow_big.xacro` each carry three
properties — `snx/sny/snz` and `enx/eny/enz` — that shift **all** of that
module's meshes together. Cosmetic only; kinematics are unchanged. They exist to
absorb a frame mismatch when the big parts come from a different CAD canvas than
the small ones.

These used to be editable over HTTP (`GET`/`POST /nudge`). That endpoint is
**removed** — it had no UI, and hand-editing the xacro is clearer than a
write-only API nobody could see the effect of. Edit the properties directly.

**They should be zero whenever the big parts are exported in the same frame as
the small ones.** Current state:

| Module | Offsets | |
|---|---|---|
| `shoulder_big.xacro` | `snx=0  sny=0  snz=0` | clean — same-frame export |
| `elbow_big.xacro` | `enx=0  eny=-0.002  enz=-0.006` | **non-zero** — 2 mm and 6 mm of shim |

A stale non-zero offset left over from an older export is indistinguishable, at a
glance, from a genuinely broken URDF: the whole assembly floats off the arm.
Before touching these, check the module's datums instead — the adapter's AABB
centre should sit on the extrusion axis, and the rotation link should be centred
on its joint axis. `shoulder_big.xacro` lists the exact numbers.

## Two buttons, because there are two costs

**Save to xacro** writes the five build values into `mod101_config.xacro`. It is
instant (~4 ms) and it does **not** rebuild anything.

**Rebuild MoveIt config** regenerates the self-collision matrices, at
`REGEN_TRIALS` (1,000,000) samples — about 20 s for all four tools. `POST
/collisions/regen`.

They were one button. That was wrong: it made a Save look cheap while quietly
running a *fast, under-sampled* regeneration (the generator's own 10,000 default),
which disables pairs that really can collide. Splitting them lets the rebuild be
slow and correct, and makes the cost something you choose rather than something
that happens to you.

`GET /collisions` reports `stale` — whether the generated matrices still carry
the same build stamp as `mod101_config.xacro`. That is read from the files
themselves, not remembered in the server, so it survives a restart and is still
right if you edit the xacro by hand. The UI uses it to mark the rebuild button as
owed.

**Switching tool does not make anything stale.** The generator writes one matrix
per tool on every run, all four stamped with the current rails and mounts, so
picking a different tool just selects an already-current file.

Neither button touches a robot that embeds the arm.

That is deliberate. An earlier version reached into a base101 workspace through a
`BASE101_WS` setting, which inverted the dependency: the arm had to know where
its consumers lived. That breaks as soon as there are two of them, one is on
another machine, or a consumer isn't base101 at all.

So each consumer owns its own regeneration, and the configurator only tells you
to run it — the `/save`, `/tool` and `/collisions` responses all carry a
`downstream` field with the instruction. For base101:

```bash
cd ~/robots/base101
./src/base101_arm/base101_arm_moveit_config/scripts/sync_arm_change.sh
```

That script sources the mod101 underlay (`MOD101_WS`, default `~/robots/mod101`)
so it reads the build args you just saved, then regenerates base101's chassis
matrices. Arguments pass through to its `gen_collision_matrix.py`.
