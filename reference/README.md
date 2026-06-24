# reference/ — CAD export source geometry

These two Fusion/CAD URDF exports are the **source geometry** that the single
parametric arm in `src/mod101_description/urdf/mod101_macro.xacro` was
consolidated from. They are **not built** (they live outside `src/`, so colcon
ignores them) and are **untracked** in git — kept locally as a reference for
mesh provenance, mass figures, and mount-induced offsets.

- `mod101-small_description/` — clean single-level export. The **small**
  (STS3215/3250, 25T) geometry.
- `mod101-large_description/` — the **big** (ST3120) geometry. The original
  export was double-nested with a second, *differing* stray
  `urdf/mod101-large.xacro` at the outer level; per the integration spec the
  fully-nested copy was kept and the stray discarded. The big elbow servo mesh
  (`big_elbow_servo_1.stl`) lives in the small export's `meshes/`.

Both exports share the same 18 link names (0 mismatches) — that parity is what
made consolidation possible — but use CAD auto-generated joint names
(CAD auto-names like `Revolute<N>` / `Rigid<N>` / bare integers) that disagree between the two. The
maintained file defines one canonical joint vocabulary instead
(`joint_base` / `joint_shoulder` / `joint_elbow` / `joint_wrist_tilt` /
`joint_wrist_roll`, fixed joints as `fixed_<childlink>`).

## Note on the consolidation approach

The maintained arm is a **newer physical design** than these exports (it has an
`elbow_fork`, `camera_tilter`, split L/R servo halves, dual shoulder rails, and
a `wrist_flange` tool-mount contract that the exports lack). Rather than rebuild
from the export vocabulary, the spec's parametric concepts were retrofitted onto
the maintained macro: the four build args (`shoulder_ext_length`,
`elbow_ext_length`, `shoulder_mount`, `elbow_mount`), the canonical joint names,
and the per-joint mount selector (mass + downstream Δ). These exports therefore
serve as historical/source reference, not as a pose-parity regression target.
