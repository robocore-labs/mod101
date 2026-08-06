# mod101 — Integration & Configurator Spec (for Claude Code)

Three independent tasks against `github.com/cristidragomir97/mod101`:
- **A.** Consolidate the two CAD exports (`mod101-large`, `mod101-small`) into **one parametric URDF** with parameterised extrusion lengths **and** a per-joint big/small mount selector for shoulder and elbow, with canonical joint names and a corrected axis.
- **B.** Rewrite the configurator per the agreed model, plus a live total-cost readout. No order/checkout page yet.
- **C.** New script: estimate 3D-printed link masses from STL volume (PLA-CF, 15% infill) and emit a masses file the configurator/URDF consumes.

Do A, B, C as separate PRs. A is a prerequisite for the real masses in B.

---

## PART A — Consolidated parametric URDF

**Goal:** one `mod101.xacro` in `src/mod101_description/` that takes **four xacro args** and resolves to any build in the family. The two CAD exports become *source geometry* feeding this one file — not two maintained packages.

```
<xacro:arg name="shoulder_ext_length" default="0.130"/>   <!-- arm rail, m -->
<xacro:arg name="elbow_ext_length"    default="0.219"/>   <!-- forearm rail, m -->
<xacro:arg name="shoulder_mount"      default="small"/>   <!-- small | big -->
<xacro:arg name="elbow_mount"         default="small"/>   <!-- small | big -->
```
`small` = compact STS3215/3250 body (25T). `big` = ST3120 large body (different spline). These args are exactly the configurator's two length inputs + two motor dropdowns, so the configurator drives the URDF directly.

### Inputs (the exports)
- `mod101-large_description` — **double-nested** with a **second, differing** stray `urdf/mod101-large.xacro` at the outer level. Use the fully-nested copy; discard the stray. This is the **big** geometry (includes `big_elbow_servo_1.stl`).
- `mod101-small_description` — clean, single-level. This is the **small** geometry.
- Both share the **same 18 link names** (verified, 0 mismatches) — that parity is what makes consolidation possible. Mine each export for its mount meshes, masses, and the mount-induced offsets (A3).

### A1. Canonical joints (one vocabulary)
Both exports use CAD auto-names (`Revolute 24`, `Rigid 12`, bare `4`) and disagree. Define these once in the parametric file:

| canonical name | parent → child | type | canonical axis | notes |
|---|---|---|---|---|
| `joint_base` | servo_base_1 → shoulder_rotation_link_1 | continuous | `0 0 1` | base yaw |
| `joint_shoulder` | servo_shoulder_1 → shoulder_adapter_1 | continuous | `0 1 0` | pitch |
| `joint_elbow` | servo_elbow_1 → elbow_adapter_1 | continuous | **`0 1 0`** | **see A4** |
| `joint_wrist_tilt` | servo_wrist_tilt_1 → wrist_roll_adapter_1 | continuous | `0 -1 0` | pitch |

Fixed joints → `fixed_<childlink>`, `type="fixed"`. Add `<limit>` to continuous joints per the ros2control file if controllers need it.

### A2. Parametric extrusion length
The rail-dependent joint origins advance along the local extrusion axis with `*_ext_length`. From the exports, the pure rail-length terms are visible as the origins that move only along local X:
- arm: `elbow_link_1` origin X = −82 mm (small) ↔ −97 mm (big-export, which also bundles the mount Δ — separate them in A3).
- forearm: `wrist_servo_adapter_1` origin X = 98 mm (small) ↔ 128 mm (big-export).

Author the elbow-side and wrist-side mount origins as `f(ext_length)` so cutting a rail to length L places the downstream joint correctly. Anchor the function at the **small** export (known length → known origin); the configurator already assumes 1 mm of rail = 1 mm of link advance.

### A3. Per-joint mount selector (mesh + mass + downstream offset) — THE CORE
Verified from the exports: swapping a joint small→big is **axis-shifting**, not axis-preserving. The big servo body pushes the *next* joint outward by a fixed amount, independent of rail length. So each `*_mount` macro must switch **three** things:

1. **Mesh** — small: `servo_shoulder_1.stl` / `servo_elbow_1.stl`; big: the `big_*_servo` meshes + matching bracket meshes.
2. **Mass + inertia** — small: STS3250 (or 3215) body; big: ST3120 body (210 g, larger inertia). Pull from the motor table / `link_masses.json`.
3. **Downstream origin offset `Δ`** — add a fixed translation to the *next* joint's origin when the mount is `big`. This lengthens the effective link, so it MUST be reflected here and in the configurator's payload math (picking `big` lengthens the lever — physically correct).

```
shoulder_mount=big → big shoulder servo mesh+mass, + push elbow axis out by Δ_shoulder
elbow_mount=big    → big elbow servo mesh+mass,    + push wrist axis out by Δ_elbow
```

**Offsets to fill in (MEASURE IN CAD — axis-to-axis, big-mount minus small-mount, SAME rail length):**
- `Δ_shoulder` = (elbow axis position with big shoulder mount) − (with small mount). **TODO: Cristi to provide.**
- `Δ_elbow` = (wrist axis position with big elbow mount) − (with small mount). **TODO: Cristi to provide.**

Provisional estimate from the exports: `Δ_elbow ≈ 30 mm` (forearm/wrist side), arm-side contribution `≈ 15 mm` — but these are **entangled with the rail-length difference between the two export sizes**, so do NOT hardcode them. Wait for Cristi's clean axis-to-axis measurements; until then, parameterise `Δ_shoulder`/`Δ_elbow` as xacro properties at the top of the file with the provisional values and a `FIXME`.

Implement each selector as a xacro macro: `<xacro:joint_mount name="shoulder" variant="$(arg shoulder_mount)" .../>` that conditionally (`<xacro:if value="${variant == 'big'}">`) instantiates the big mesh/mass and applies `Δ`.

### A4. The axis fix (Cristi flagged this — confirm, don't guess)
Cristi reported "Revolute 24 has the wrong axis." Investigation:
- `Revolute 24` = **base yaw**, axis `0 0 1` in **both** exports — they agree; no inter-package inconsistency visible there.
- The axis actually inconsistent is the **elbow**: big-export `0 +1 0` vs small-export `0 -1 0` (opposite sign).

Action: in the consolidated file the elbow axis is defined **once** as canonical `0 1 0`, which resolves the disagreement by construction. Then: build, open RViz, command each joint positive, and confirm a positive elbow command raises the forearm and base yaw is the right handedness against the real hardware. If Cristi confirms the **base** axis is genuinely wrong (not just the elbow), update the canonical `joint_base` axis to the verified value. Comment each: `<!-- axis canonicalized; was <old> -->`.

### A5. Verify wrist-roll DOF
Both exports model `servo_wrist_rotation_1` via a **fixed** joint, but the arm is nominally 5+1 DOF. Confirm with Cristi whether `joint_wrist_roll` should be **actuated** in the consolidated file or stays fixed. Do not change DOF silently.

### A6. Repo structure
- Single source of truth: `src/mod101_description/urdf/mod101.xacro` (parametric, four args).
- Keep the two raw exports only as `reference/` geometry + a regression check (render the parametric file at small-config and at big-config, diff link poses against the corresponding export within tolerance). Do **not** ship them as parallel ament packages.
- Move all mount/servo/bracket meshes into `mod101_description/meshes/` with both small and big variants present; fix `package://` URIs.
- Delete the broken double-nested/hyphenated export packaging.

### A7. Acceptance (Part A)
- `mod101.xacro` renders for all four combinations of `{shoulder_mount, elbow_mount} × {small,big}` and arbitrary `*_ext_length` in 0.08–0.28 m, with no unresolved meshes.
- Rendering at `shoulder_mount=small, elbow_mount=small, lengths=small-export` reproduces the **small export's** link poses within ~1 mm (excluding the known different-pose artifact); same for big.
- Setting a joint to `big` moves the downstream joint out by exactly `Δ` and swaps mesh+mass.
- RViz: positive command on each joint moves the right link the right way; elbow consistent.
- `git grep -iE 'revolute |rigid [0-9]'` returns nothing.
- Default args produce the 130/219 small-mount default build.

---

## PART B — Configurator rewrite

Implements the model in `configurator-v2-spec.md` (single-output per joint, per-joint motor selection, two-regime payload, binding-joint readout, three reference builds). That spec is the source of truth for the **physics**; this section adds what's new since: real masses (from Part C) and a **total-cost** readout. **No order/checkout page** — cost is display-only.

### B1. Carry over from configurator-v2-spec.md (unchanged) + mount coupling
- Per-joint motor dropdowns (shoulder, elbow) ∈ {STS3215, STS3250, ST3120}; defaults STS3250/STS3250. **These dropdowns map 1:1 to the URDF's `shoulder_mount`/`elbow_mount` args** (STS3215/3250 → `small`, ST3120 → `big`) and the two length inputs → `*_ext_length`. The configurator sets all four xacro args.
- **Mount Δ feeds the payload math:** when a joint is set to ST3120 (`big`), add that joint's `Δ` to the effective link length in the torque model (`Δ_shoulder` lengthens L1, `Δ_elbow` lengthens L2). Picking the big servo both raises the cap *and* lengthens the lever — the model must do both, or it will over-report payload for big-mount builds. Use the same `Δ` values as the URDF (single source).
- Two length inputs (`shoulder_ext_length`, `elbow_ext_length`), bounds 80–280 mm; write to the parametric xacro; live three.js reload.
- Payload model: both joints computed, **binding joint reported**, three regimes (duty 0.70 / hold 0.50 / peak 1.0) × two reaches (full / 70%). Lead with duty-cycle as the rated headline.
- Golden acceptance cases A–D from that spec.

### B2. Real masses (replaces the lumped estimates)
- Replace the hardcoded `M_WRIST_LUMP` / elbow-lump / `RHO·L` rail terms with values from `link_masses.json` (Part C output) wherever the printed parts are concerned. Keep `RHO` (extrusion) analytic since rails are cut-to-length, but pull **printed bracket/body masses** from the JSON.
- Map: the model's `m_elbow_lump` = (elbow servo body) + (elbow fork/adapter/body printed masses from JSON); `m_wrist_lump` = (wrist tilt + wrist roll servos) + (printed wrist/camera bracket masses from JSON). Servo body masses stay in the motor table.
- If `link_masses.json` is absent, fall back to the spec's lumped constants and show a "estimated masses" badge.

### B3. Total cost (new — display only)
- Add an editable price table (clearly marked placeholder until Cristi sets real BOM costs):
  ```
  SERVO_PRICE_USD = { 'STS3215': 18, 'STS3250': 30, 'ST3120': 60 }   # 3120≈2×3250≈4×3215, per Cristi
  ```
- BOM for cost: `joint_base` = STS3215, `joint_shoulder` = (selected), `joint_elbow` = (selected), `joint_wrist_tilt` = STS3215, `joint_wrist_roll` = STS3215, gripper = STS3215/SC09 (use STS3215 price). So:
  `total = base + shoulder(sel) + elbow(sel) + wrist_tilt + wrist_roll + gripper`.
- Optionally add a flat `FRAME_COST_USD` placeholder (rails + printed parts + fasteners) so the number reads as a rough build cost, but label the whole thing **"estimated servo cost"** if frame cost is omitted. Do not imply it's a purchase price.
- Show total next to the payload panel; update live with motor selection.

### B4. UI additions
- Cost line under the payload table: "Servos: $X (base 3215 · shoulder {sel} · elbow {sel} · 3× wrist/gripper 3215)".
- The three reference builds (DEFAULT dual-3250 / PLUS 3120-sh+3250-el / MAX dual-3120) as one-click presets that set both dropdowns.
- Keep the honesty footer (regime definitions, static-vs-duty, "verify on bench").

### B5. Acceptance (Part B)
- Golden cases A–D still pass with real masses within ±5% (masses shift the absolute numbers; binding-joint and ordering invariants must hold).
- Cost updates correctly on motor change; DEFAULT/PLUS/MAX presets produce the payload+cost rows from the lineup table.

---

## PART C — PLA-CF printed-mass estimation script

`tools/estimate_masses.py` — estimates printed-part mass from STL geometry so the configurator/URDF use real masses instead of guesses.

### Model
A printed part = dense shell (walls + solid top/bottom) + sparse infill interior. From an STL, estimate:
```
V_model = watertight mesh volume            # cm^3
A_model = mesh surface area                  # cm^2
V_shell = min(V_model, A_model * t_wall)     # cm^3, t_wall in cm  (perimeter+skin shell)
V_infill = max(0, V_model - V_shell)
mass_g  = RHO_PLACF * (V_shell + INFILL * V_infill)
```

### Parameters (CLI flags, with defaults)
```
RHO_PLACF = 1.24   g/cm^3   # PLA-CF filament density (printed effective; calibrate — see below)
INFILL    = 0.15            # 15% infill, per spec
T_WALL    = 0.16   cm       # 1.6 mm shell (4 perimeters @0.4mm ~= walls; folds in top/bottom skin)
```
Expose `--rho --infill --wall --units`. Default STL units = mm (convert to cm internally).

### Implementation notes
- Use `trimesh` (`mesh.volume`, `mesh.area`; check `mesh.is_watertight`, attempt `mesh.fill_holes()` and warn if still open — open meshes give garbage volume).
- Batch over a `meshes/` dir; skip the servo/camera/extrusion meshes (non-printed) via an exclude list or a `--printed-only` name filter (printed = brackets/bodies/adapters/links: `*_adapter_*`, `*_link_*`, `*_body_*`, `camera_wing`, etc.; **not** `servo_*`, `*extrusion*`, `wrist_camera_v1`).
- Output `link_masses.json`: `{ "<link_name>": {"mass_g": float, "V_cm3": float, "shell_frac": float, "watertight": bool} }`. Also print a table and a total.

### Calibration (important — do not ship uncalibrated)
The shell model is first-order. Two calibration paths, document both in the script header:
1. **Weigh one real print** of a known part, solve `RHO_PLACF` (or an overall fudge factor) so the script matches the scale, apply to all.
2. **Better: read the slicer's predicted gram value** for each part sliced at the real profile (PLA-CF, 15%, actual walls) — that's ground truth. Optionally add `--from-slicer slicer_masses.csv` to ingest those directly and bypass the geometric estimate. The geometric script is the fallback for parts not yet sliced.

### Acceptance (Part C)
- Runs over a `meshes/` dir, emits `link_masses.json` + a printed table with total.
- Flags any non-watertight mesh instead of silently producing a wrong volume.
- A solid calibration cube STL returns `mass ≈ RHO * V` (shell ≈ whole volume) as a sanity check.
- Output JSON drops straight into Part B (B2).

---

## Suggested order
1. **A** (consolidate into one parametric URDF: canonical joints, fixed axis, `*_ext_length` + `*_mount` args with Δ offsets) — unblocks correct geometry. Get Cristi's two Δ measurements before finalising A3.
2. **C** (mass script) — run on the consolidated meshes to produce `link_masses.json`.
3. **B** (configurator) — drives the four xacro args, consumes real masses, couples mount Δ into payload, adds cost.
