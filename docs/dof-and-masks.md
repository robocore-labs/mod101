# DOF and `dof_mask` — what the arm can and cannot do

Measured against the live base101 + mod101 sim on 2026-08-24, with
`robocore.kinematics` at engine@pending. Every number here came from the
robot, not from reasoning about the wrist. That distinction is the whole
point of this document: the first `dof_mask` written for this arm was
argued from its shape and was **wrong** — see
[Trap 1](#trap-1-reasoning-from-the-wrist).

---

## 1. What `dof_mask` is

A group's `dof_mask` is six flags, `[x, y, z, rx, ry, rz]`, saying which
Cartesian degrees of freedom `kin.ik` should **constrain**. A `0` means
"leave this free — the chain cannot choose it independently, so do not
pretend otherwise".

```yaml
groups:
  arm:
    dof_mask: [1, 1, 1, 1, 1, 0]     # position + roll + pitch; yaw free
```

It exists because an arm with fewer than six joints has a reachable set of
fewer than six dimensions. Ask such an arm to hold a fully specified pose
and there is generally no solution. Two bad things can happen if the mask
is absent or wrong:

- **too many constraints** — `ik` raises `Unreachable` for poses the arm
  could perfectly well reach;
- **the wrong axis freed** — `ik` silently solves a different problem than
  the mission asked, and the error shows up as a tool that is off by a few
  degrees in a way nothing reports.

Neither crashes. That is why this is worth writing down.

---

## 2. The chain

Five joints, read off `/controller_manager` and `/robot_description`:

| joint | axis | limits (rad) | contributes |
|---|---|---|---|
| `arm_joint_base` | +z | −1.571 … 1.571 | slews the whole arm |
| `arm_joint_shoulder` | +y | −0.001 … 3.142 | pitch |
| `arm_joint_elbow` | −y | −0.001 … 3.142 | pitch |
| `arm_joint_wrist_tilt` | −y | −1.230 … 1.540 | pitch |
| `arm_joint_wrist_roll` | +x | 0.000 … 3.142 | roll about the tool axis |

Tip: `arm_wrist_flange` (the tool mount — the jaws are modelled but have
no grasp-point frame, see `found_along_the_way.md` item 7).

Note the shape: **one yaw joint, three pitch joints, one roll joint.**

---

## 3. What it can and cannot do — measured

Take a pose the arm is known to hold (`q = [0.3, 1.0, 1.2, 0.2, 0.5]`),
perturb one axis at a time, and try to solve it with **all six** DOF
constrained.

| perturbation | 0.02 / 0.1 | 0.05 / 0.2 | 0.1 / 0.4 | 0.8 |
|---|---|---|---|---|
| **x** (m) | UNREACHABLE | UNREACHABLE | UNREACHABLE | — |
| **y** (m) | UNREACHABLE | UNREACHABLE | UNREACHABLE | — |
| **z** (m) | ok | ok | ok | — |
| **roll** (rad) | ok (5e-05) | ok (3e-07) | ok (2e-05) | ok (5e-06) |
| **pitch** (rad) | ok (2e-05) | ok (7e-04) | ok (2e-04) | ok (1e-04) |
| **yaw** (rad) | UNREACHABLE | UNREACHABLE | UNREACHABLE | UNREACHABLE |

With the profile's mask applied (`rz` free) the same perturbations all
succeed:

| perturbation | result |
|---|---|
| x, y, z ±0.02 … ±0.10 m | all ok |
| roll, pitch ±0.2, ±0.5 rad | all ok |

**So: the arm commands position and tool roll/pitch freely. It cannot
choose yaw.**

The physical story is one sentence: *the base joint that aims the arm at a
target is the same joint that sets the tool's yaw*, so the two are not
independent. Point at something to the left and the tool faces left; there
is no second joint to undo that.

---

## 4. The subtlety: the deficiency is not axis-aligned

`dof_mask` has one flag per axis, which quietly assumes the
uncontrollable direction *is* an axis. It is not. Taking the left singular
vector of the smallest singular value of the Jacobian — the direction in
which the tool cannot be moved at all:

| configuration | vx | vy | vz | wx | wy | wz |
|---|---|---|---|---|---|---|
| zero (stowed) | −0.000 | **+0.992** | −0.000 | +0.000 | −0.000 | −0.128 |
| folded forward | −0.000 | **+0.979** | +0.000 | +0.000 | +0.000 | −0.205 |
| mid workspace | −0.286 | **+0.926** | −0.000 | +0.092 | +0.028 | −0.227 |
| reaching out | −0.000 | **+0.973** | −0.000 | −0.165 | −0.000 | −0.161 |

It is dominated by **lateral translation (vy)** with a yaw (wz) component,
and it rotates as the arm moves.

So why mask `rz` and not `y`?

Because the mask decides *what the mission may command*, and a mission
wants to command position. Freeing `rz` gives the solver exactly one
degree of slack, which it spends absorbing the vy–wz coupling — and the
result is that x, y, z, roll and pitch are all achievable (§3, second
table). Freeing `y` instead would give up commanding lateral position,
which is the thing you actually care about, to keep a yaw you cannot use.

**`dof_mask` is therefore an approximation, chosen for what it buys the
caller — not a description of the null space.** For this arm the
approximation is exact enough that everything else becomes reachable. On a
robot where it is not, the honest answer is a tighter mask and a mission
that checks `kin.reachable()` rather than assuming.

---

## 5. Singularities

`kin.manipulability` over sample configurations, masked to the five
controllable DOF — see [Trap 3](#trap-3-manipulability-on-a-sub-6-dof-arm):

| configuration | rank | manipulability | smallest σ | singular values |
|---|---|---|---|---|
| zero (stowed) | 5 | 0.000816 | 0.0224 | 1.749, 1.008, 1.000, 0.161, 0.022 |
| folded forward | 5 | 0.006766 | 0.1271 | 1.758, 1.022, 1.000, 0.145, 0.127 |
| mid workspace | 5 | 0.008310 | 0.1251 | 1.767, 1.191, 0.798, 0.175, 0.125 |
| reaching out | 5 | 0.001601 | 0.0501 | 1.750, 1.316, 0.544, 0.159, 0.050 |
| elbow straight | 5 | 0.000769 | 0.0224 | 1.748, 1.220, 0.728, 0.161, 0.022 |

Rank never drops below 5 in normal use — this arm has no interior
singularity that costs it a whole degree of freedom. What it has is
**near-singular configurations**: stowed and elbow-straight both fall to
σ_min ≈ 0.022, an order of magnitude below mid-workspace. Those are the
configurations where a small Cartesian velocity demands a large joint
velocity.

Practical rule: **manipulability above ~0.005 is comfortable; below
~0.001 the arm is near the edge of its workspace or folded on itself.**
Stowed (all zeros) is the worst of the sampled set, which is worth knowing
because it is where the arm starts.

For a `Controller.STREAM` session this is the number to watch. It is not a
guarantee — the server-side joint velocity clamps are (spec §21, §26.5).

---

## 6. Reach envelope

`kin.reach()` reports **0.711 m**, which is the summed link extension —
an upper bound, not a promise. What is actually solvable, sweeping
`y = 0` with the profile mask:

| height | reachable x |
|---|---|
| z = 0.25 m | 0.15 … 0.50 m |
| z = 0.35 m | 0.15 … 0.45 m |
| z = 0.45 m | 0.05 … 0.40 m |
| z = 0.55 m | 0.00 … 0.35 m |

The inner limit is the arm folding back into its own base; the outer is
extension. Higher targets must be closer in — the envelope leans back as
it rises, which is what you would expect from an arm on a deck.

A slice at z = 0.35 across y (`#` reachable, from
`examples/27_kinematics.py`):

```
y=+0.30  .#######.....
y=+0.15  .#########...
y=+0.00  ...#######...
y=-0.15  .#########...
y=-0.30  .#######.....
         x=0.00 ......... 0.60
```

Symmetric, as it must be, with the hole at `y=0` where the arm cannot fold
back far enough.

**The `workspace:` box in the profile is `x [-0.15, 0.55] y [-0.45, 0.45]
z [0.20, 0.85]`, which is deliberately conservative and NOT this
envelope.** It is a safety clamp sized so nothing inside it hits the
chassis. **Enforcement is a Phase 7 item** ("Safety at rate: joint/
velocity clamps, FK workspace box") — not 6.4, which an earlier version of
this file got wrong — so the box is inert today and has not been tightened
against the numbers above.

---

## 7. Determining the mask for a new robot

Do not reason from the wrist. The recipe:

1. Get the arm to a **general** configuration — not stowed, not straight,
   nothing at a limit. A degenerate pose makes everything look coupled.
2. `target = kin.fk(group, q=q0)`. This pose is reachable by construction,
   so it is useless as a test on its own (see
   [Trap 2](#trap-2-testing-with-a-pose-that-came-from-fk)).
3. Perturb **one axis at a time** by a few centimetres or a few tenths of
   a radian and solve with `mask=[1]*6`. Record which perturbations raise
   `Unreachable`.
4. The axes that fail are the coupled ones. Free the **rotational** one
   the mission is least likely to need — freeing a translation gives up
   commanding position, which is almost never the right trade.
5. Confirm: with the candidate mask, every perturbation from step 3
   should now solve.
6. Sanity-check `np.linalg.matrix_rank(kin.jacobian(group, q=q0))`. It
   should equal the number of joints; if it is lower, the arm has a
   genuine internal singularity at that configuration and step 1 picked a
   bad pose.

`examples/27_kinematics.py` does steps 2, 3 and 5 for whatever arm it is
pointed at.

---

## 8. Traps

### Trap 1: reasoning from the wrist

The first mask written for this arm was `[1, 1, 1, 1, 0, 1]` — pitch
masked out — on the reasoning that a wrist with a single roll joint could
not independently pitch. Three of the five joints rotate about y, so pitch
is precisely what it *can* do. The mask was inverted from the truth for a
plausible-sounding reason, and nothing caught it for two tasks because
`dof_mask` was a field nobody read yet.

### Trap 2: testing with a pose that came from `fk`

A pose produced by `kin.fk(group, q=...)` is in the arm's own image, so it
solves with **all six** DOF constrained no matter how deficient the arm
is. A test built on one proves nothing about the mask. Perturb it first.

### Trap 3: manipulability on a sub-6-DOF arm

The textbook Yoshikawa measure is `sqrt(det(J · Jᵀ))`, which assumes at
least as many joints as task dimensions. With five joints and six rows,
`J · Jᵀ` is singular by construction and the measure returns **0.0 at
every configuration** — a number that reads as a permanent singularity and
means nothing. `kin.manipulability` computes over the rows `dof_mask`
selects, and raises rather than lying if more DOF are constrained than
there are joints.

### Trap 4: `reach()` is not the workspace

0.711 m is the sum of the link lengths. Nothing at that distance is
reachable in any useful orientation. Use `kin.reachable()` against real
targets.

---

## See also

- `engine/docs/api-reference.md` § Kinematics — the API surface
- `examples/27_kinematics.py` — all of the above, live
- `found_along_the_way.md` item 9 — how the wrong mask was found
- `robocore-api-v0.5-draft.md` §26 — the spec
- Decisions 28, 30, 31 in `robocore-implementation-plan.md`
