# Calibration

The configurator's **Calibrate** tab (`configurator/calibrate.html`) is the
bring-up path for a physical arm: it sweeps each joint by hand, derives its
travel limits and home position from the motion, verifies the joint is
mechanically healthy, checks that travel against the URDF the arm will actually
be driven by, and then emits the two config files that ROS and LeRobot each need.

The page has two tabs, over one shared bus connection:

Prose on the page itself is kept behind the **(i)** in each panel header — the
explanations are worth having once and in the way every load after that.

- **Hardware** — the serial chain and nothing else. Pick a port, scan the bus,
  and get one card per servo that answers. The servo is the component and the
  card is all of it: its ID, the joint it happens to be mapped to (ROS joint,
  LeRobot motor, sign, model), its live telemetry, and the buttons that move it.
  Reassigning an ID is the one bus-wide action, so it stays outside the cards.
- **Model check** — the same three.js scene as the build page, loaded from
  `configurator/viewer.js`, driven by live servo telemetry. Each joint is swept
  here, the model follows the arm as you move it, and the measured travel is put
  side by side with the joint's URDF `<limit>`.

**Every joint is one servo.** There is no mirrored-pair path.

The wizard flow is ported from the Axon calibration tool in
`link101/link-base101/web/calibrate.html`, but the transport is not: this talks
to the servos **directly**, with no intermediate firmware. The DDSM210 wheel
control that lived alongside the original in `axon-config.html` is deliberately
not carried over — that's base101 chassis hardware, not ST-series servos.

```bash
pip install st3215                 # once
python3 configurator/server.py     # http://localhost:8001/calibrate.html
```

Your user needs read/write on the serial device — on Debian/Ubuntu that means
membership in `dialout` (`sudo usermod -aG dialout $USER`, then log out and
back in).

## How it's wired

```
browser (calibrate.html)
   │  HTTP/JSON
   ▼
configurator/server.py  ──► configurator/motors.py ──► st3215 ──► USB-TTL ──► servo chain
```

The browser never touches a serial port. `motors.py` owns the port and every
servo transaction; the page drives it over plain HTTP. Two consequences worth
knowing:

- **Any browser works.** There's no Web Serial dependency, so Firefox and
  Safari are fine.
- **The bus is serialized.** All access goes through one lock, and the built-in
  HTTP server handles one request at a time. The telemetry poller deliberately
  skips whichever joint is being swept or drive-tested so it isn't competing
  with that joint's own read loop.

`motors.py` is also where `st3215`'s inconsistent return values get flattened —
`ReadSpeed` hands back a `(speed, comm, error)` tuple while its siblings return
a scalar or `None`, `StartServo` returns a tuple where `StopServo` returns a
bool, and `ChangeId` returns `None` on *success* — where "success" only means
the bytes reached the UART, because its EPROM ops are all TxOnly. `change_id()`
therefore pings the new ID to learn what actually happened, refuses an ID that
is already answering, and re-locks the EEPROM (the library's own final
`LockEprom` is addressed to the *old* ID, which has already stopped replying).

### HTTP surface

| Endpoint | Purpose |
|---|---|
| `GET /bus` | connection status + known servo IDs |
| `GET /bus/ports` | candidate serial devices (USB only, unless there are none) |
| `POST /bus/connect` | `{port}` — open and scan |
| `POST /bus/disconnect` | disarm + close |
| `POST /bus/scan` | re-ping IDs 1..99 |
| `POST /bus/disarm` | torque off everything |
| `GET /servo/<id>` | telemetry: pos, speed, current, voltage, load, temp |
| `POST /servo/<id>/torque` | `{on}` |
| `POST /servo/<id>/move` | `{pos, speed?, accel?}` |
| `POST /servo/<id>/profile` | `{speed, accel}` |
| `POST /servo/<id>/zero` | `{target}` — define the current position as `target` (writes EEPROM) |
| `POST /servo/<id>/id` | `{to}` — reassign servo ID (writes EEPROM) |
| `GET /urdf` | the expanded URDF, for the model tab's scene |

Bus scanning is bounded to IDs 1..99 on purpose (`DEFAULT_SCAN_HI`). `st3215`'s
own `ListServos()` pings 0..253, which is seconds of dead time on a real bus.
ID 0 is never scanned, which is why `change_id()` will not assign it.

The arm is left disarmed at every exit: on disconnect, on server shutdown
(`atexit`), and on tab close (the page fires `/bus/disarm` through
`navigator.sendBeacon`, which survives unload).

## The flow

Per joint, in order:

1. **Guided capture** — torque drops and the joint is moved by hand through five
   prompted steps, in this order:

   | step | what you do | what it captures |
   |---|---|---|
   | 1 | put the joint at its **URDF default** | `home` |
   | 2 | move to the **maximum right** stop | one limit |
   | 3 | move to the **maximum left** stop | the other limit |
   | 4 | **sweep twice**, stop to stop | the health check |
   | 5 | return to the **URDF default** | the slip check |

   Limits are the two captured stops with `MARGIN` = 30 ticks pulled in from
   each, so a calibrated limit never commands into the stop itself.

   Steps 1 and 5 — the two that hold the joint at its zero pose — also offer
   **Set zero here** and **Set centre here**; see
   [The encoder seam](#the-encoder-seam).

   **Home is the URDF default, not the middle of the travel**, and it is
   captured first for two reasons. Everything downstream is an angle relative to
   home — ROS radians, LeRobot's `homing_offset`, the pose the 3D model draws —
   so if home were the midpoint of the measured travel then "0 rad" would mean
   "halfway between the stops", which is the URDF's zero only on a joint that
   happens to be symmetric. On every other joint the model and the hardware
   would sit at different angles while both claimed zero. And it has to be step
   1 because the model has nothing to draw against until it exists: before home
   is known the anchor is wherever the arm was when first seen.

   Step 4 needs two full end-to-end traversals; the counter watches for arrival
   at one captured end having last been at the other, so a twitch at the top of
   the range does not score as a sweep. Step 5 is the one that catches an
   encoder that moved during those sweeps — the joint is returned to the same
   physical pose, so any residual is slip, and every limit just measured is off
   by that much. Under 8 ticks reads as servo noise; more is reported as a
   fault.
2. **Verdict** — `clean` or `fault`. A joint fails on a position discontinuity
   (`JUMP_THRESH` = 400 ticks ⇒ encoder slip).
3. **Accept** — adjust min/max/home, or capture each from the current
   hand-held position. The page shows live what those ticks mean in radians for
   the mapped ROS joint, and how that compares to the joint's URDF `<limit>`.
4. **Arm-check** — re-arms at a slow profile holding present position, then
   reads current. Must be low (< 250 mA) or it disarms immediately. This is a
   *soft* check: nothing in this path can cap current in hardware.
5. **Drive test** — jog the joint across its calibrated range under power and
   watch the tracking error, with the model tracking it pose for pose.

### What a write produces

| file | for |
|---|---|
| `src/mod101_control/config/calibration.yaml` | ROS — radians relative to each joint's home |
| `src/hardware/mod101_hw_bringup/config/servos.generated.yaml` | the hardware servo driver — motor IDs, joints, directions |
| LeRobot calibration JSON | LeRobot |

The middle one is what makes calibrate-then-launch enough to get a working arm.
It carries the **wiring** half of the servo driver's config — which motor ID is
which joint and which way it turns, straight out of the servo map on the
hardware tab. The bus port, rates, speeds and group structure stay hand-authored
in `servos.yaml`, which the configurator never overwrites, and
`bringup.launch.py` merges the two before injecting the tick calibration from
the first file. See
[mod101_hw_bringup](../src/hardware/mod101_hw_bringup/README.md).

Group membership is read out of `servos.yaml` rather than guessed from joint
names. A servo mapped to a joint that no group claims — or a group joint with no
servo behind it — is reported in the console after the write.

**Start over** (in the *Joints* panel header) discards every calibrated value
and runs the wizard again from the first joint: results, model anchors and the
working min/max/home are cleared, and anything still armed is disarmed — a
drive-test leaves its joint under power, and its Disarm button goes away with
the card that owned it.

It deliberately **keeps the servo map**. Joint names, signs, LeRobot names and
models describe how the bus is wired, not what a sweep found; re-typing them
after every restart is how a sign ends up wrong, and a wrong sign is the failure
that survives all the way to a policy driving the arm backwards. Use
Export/Import on the hardware tab to move a map between machines.

**Nothing is written to the servos.** Calibration lives in the generated repo
files, not in servo EEPROM — no wear, and nothing lost when a servo is swapped.
The only EEPROM write in the whole page is the explicit *Reassign a servo ID*
action.

### The encoder seam

The encoder is absolute over one turn and its count wraps 4095 → 0 at one
physical angle, decided by nothing more than how the horn happened to spline on.
If that seam falls inside a joint's travel, the calibrated range is an arc
across it and **`ticks_min` comes out numerically larger than `ticks_max`**.

The page itself does not care — everything here is computed as signed ticks from
home, and `wrapSigned` keeps such an arc contiguous. The *files* care, because
`ticks_min`/`ticks_max` and LeRobot's `range_min`/`range_max` are plain numbers
with no seam in them. `st3215_manager`'s `MotorGroup` rejects the group outright
(`min_ticks >= max_ticks`) and the driver refuses to start — at launch, on
another day, a long way from the arm the numbers were measured on.

**Set zero here** and **Set centre here**, on either of the guided flow's
zero-pose steps, each write a position-correction offset into the servo's EEPROM
so that the pose being held reads a chosen tick. That survives power cycles and
the ROS driver reads the same shifted frame, which is the point — a fix that
lived only in this page would be undone by the next power-up. The ticks already
captured for that joint are rebased by the same shift, so the flow carries on
rather than starting again, and other joints are untouched.

**There are two buttons because there are two shapes of joint, and it is a fact
about the mechanism rather than about the numbers.**

| | for | target | why |
|---|---|---|---|
| **Centre** | the zero pose is in the **middle** of the travel and the joint moves both ways from it | 2048 | half the encoder available in each direction |
| **Zero** | the zero pose **is a hard stop** and the joint moves one way only | 250, or 3845 if the travel runs the other way | the whole range goes to the side that can use it, and nothing can drive past the stop into the seam |

Centring a one-sided joint spends half the encoder on a direction it cannot
travel, and on a joint with 180° of range that puts the far end past 4095 — it
manufactures the seam crossing it was reached for. mod101 splits four and four:

| joint | range | zero pose is | button | travel |
|---|---|---|---|---|
| `joint_base` | −90° … 90° | mid-travel | centre → 2048 | 1024 … 3072 |
| `hn_pan_joint` | −90° … 90° | mid-travel | centre → 2048 | 1024 … 3072 |
| `hn_tilt_joint` | −80° … 80° | mid-travel | centre → 2048 | 1138 … 2958 |
| `joint_wrist_tilt` | −70.5° … 88.2° | mid-travel | centre → 2048 | 1246 … 3052 |
| `joint_shoulder` | −0.1° … 180° | a hard stop | zero → 250 | 249 … 2298 |
| `joint_elbow` | −0.1° … 180° | a hard stop | zero → 250 | 249 … 2298 |
| `joint_wrist_roll` | 0° … 180° | a hard stop | zero → 250 | 250 … 2298 |
| `6` (jaw) | 0° … 122.6° | a hard stop | zero → 250 | 250 … 1645 |

The page reads the URDF's range and marks the button it suggests, but **the
suggestion does not gate anything**. Whether the default pose is really a hard
stop is something you can feel and the URDF cannot, and a placement whose
declared travel would not fit is a warning in the log, not a refusal — an arm
whose URDF is wrong is precisely what this page exists to find, so it must stay
calibratable.

`ZERO_MARGIN` is 250 ticks (~22°) rather than 0. The stop is a stop, but the
pose is set against it **by hand**, and the URDF's idea of the default may
itself sit a degree or two off it — a joint zeroed 22° adrift still does not
wrap. On the widest joint here that still leaves ~1800 ticks unused, so the
margin costs nothing that is needed.

**When to press one.** At step 1, joint held at its URDF default, before
capturing — once per servo, the first time that arm is calibrated. The offset is
in EEPROM, so it is not a per-calibration step; a later recalibration inherits
it. Press again only when the mechanics change (horn re-splined, servo swapped,
joint reassembled), and necessarily when a verdict shows the seam banner,
because nothing else clears that. Pressing the right one on a joint that does
not need it is harmless. The one reason to hold off is an existing LeRobot
calibration worth keeping: this shifts the frame it was measured in.

The pose it is pressed at is the pose that becomes the reference, so a joint
held 20° off its default puts the whole travel 20° off. That costs margin rather
than correctness, but it is not a button to press with the joint somewhere
arbitrary.

This is also why the buttons are only on steps 1 and 5: both targets are
measured from the zero pose, so pressing either at a hard stop would place the
range from the wrong reference.

Three things guard the failure independently: the flag on the guided result, a
live banner under the editable limits that disables **Accept**, and a refusal in
`bakeJoint` — the one funnel every path takes, including manual entry.

Two caveats. The servo must be **disarmed**: re-framing one that is holding
position leaves its goal register meaning a different angle, and it slews there
the moment the offset lands, so the request is refused if torque is on. And this
is the page's only write to servo EEPROM besides ID assignment; calibration
itself deliberately stays in the repo's generated files.

## The servo card

Everything true of one servo is on one card, because on the bench it is one
question: *which motor is this, is it answering, what joint does the map think
it is, and what happens when I move it?* That used to be a mapping table, a
separate **Drive a servo** pane and a **Joint status** sidebar — three places to
watch while turning one motor, and only one of the three said which joint the
motor even was.

The cards stack one per row, and each splits the way the work does: **everything
you set on the left, everything the servo tells you on the right.** Left is the
four mapping fields, the drive row and the speed/accel fold; right is eight live
readings — position, angle, velocity, goal, current, voltage, load, temperature.
A header spanning both carries the online dot, the ID, the mapped joint name and
the torque state. Clicking a card selects it, which is also what the model tab
highlights. Below 1080px the two halves stack instead.

### Jogging

**Torque on** arms the servo at its present position, so arming never makes it
snap anywhere. Only then do **−** and **+** light up: an STS servo ignores a goal
it isn't armed for, and buttons that look live while doing nothing are worse than
buttons that are visibly off.

Each press commands exactly one step — 1, 5, 10 or 25 ticks (0.1° to 2.2°),
picked from the step selector, which is shared across every card. **Nothing
repeats while a button is held**, and the next press waits for the current one to
be acknowledged. This replaced a 0…4095 slider: the slider spanned the raw
encoder range rather than any calibrated travel, so a single drag could take a
joint into its hard stop. Steps still add up — watch the joint, and keep
**Disarm all** in reach.

Where the step counts from depends on whether the servo is holding. Armed, it is
still chasing its last goal, so the step is added to that goal — counting from
the present position would land inside the motion already in flight and every
press would ask for less than a step. Unarmed, the joint can be moved by hand,
which makes the tracked goal stale, so the step is added to a fresh reading.

Goals are **clamped** at 0 and 4095 rather than wrapped. Those are the ends of
the encoder, and a button that silently jumped a joint half a turn across the
0/4095 seam would be the most dangerous control on the page.

## Joint mapping

There's no configuration stored on the arm, so the cards' mapping fields are the
only source of truth for which servo is which joint. Defaults follow
`mod101_description/urdf/mod101_macro.xacro`:

| ID | ROS joint | LeRobot | Sign |
|---|---|---|---|
| 1 | `joint_base` | `shoulder_pan` | +1 |
| 2 | `joint_shoulder` | `shoulder_lift` | **−1** |
| 3 | `joint_elbow` | `elbow_flex` | +1 |
| 4 | `joint_wrist_tilt` | `wrist_flex` | **−1** |
| 5 | `joint_wrist_roll` | `wrist_roll` | +1 |
| 6 | `6` (tool joint) | `gripper` | +1 |
| 7 | `hn_pan_joint` | — | +1 |
| 8 | `hn_tilt_joint` | — | +1 |

**Sign** reconciles the servo's counting direction with the URDF joint axis.
`joint_wrist_tilt` has axis `0 -1 0`, hence −1 by default. `joint_shoulder`'s
axis is `0 1 0` and positive does lift the arm, but the servo on this build
counts the other way, so it is −1 too — measured, not derived. The tool joint
is literally named `6` per [tool-convention.md](tool-convention.md).

**7 and 8 are the camera tower**, not the arm, so they have no LeRobot
counterpart — the export keys them `head_pan` / `head_tilt` only to keep the
JSON unique. Their travel is **declared, not swept**: the pan/tilt was driven to
its URDF zero and given ±90° (1024 ticks) each way by decision, because the
mechanism has no hard stop to sweep into within that range. That makes
`ticks_min`/`ticks_max` a policy limit rather than a measured one — if the head
does foul something before 90°, the driver will happily drive into it instead
of clamping short. Re-sweep them here to replace the declared numbers with real
ones. Neither arc crosses the 0/4095 seam, but servo 8's max lands at 4068, 27
ticks short of it; moving the tilt zero up would push the arc over.

**ROS joint** is a picker over the joints the loaded URDF actually declares,
not free text — a typo there is a joint that silently never matches the model,
which is exactly what the model tab exists to catch. Press **Rebuild joints**
after renaming.

Edits persist in `localStorage` and travel with the exported JSON.

Getting a sign wrong is the classic failure: sim and hardware move opposite
ways on that one joint. Check each against the URDF before trusting a policy.

## Model check — does the URDF fit the arm?

The model tab answers one question: for every joint, does the URDF's declared
travel match what the hardware actually has?

The scene is the build page's, imported from `configurator/viewer.js` so there is
one renderer, one URDF fetch and one Z-up correction rather than two that drift.
On top of that the calibration page adds:

- **Live pose from the bus.** Every telemetry read is converted to an angle and
  pushed at the model, so the arm and the model move together. A servo tick only
  becomes an angle once you know which tick is the joint's zero — before a joint
  is calibrated the anchor is wherever the arm was when first seen, and from the
  moment step 1 of the guided flow captures the URDF default it is that tick,
  held for the rest of the flow. Nothing re-anchors mid-sweep, so the joint on
  screen tracks the hardware instead of drifting as the measured range grows.
- **Limits deliberately not enforced.** `setIgnoreLimits(true)` — a joint driven
  past what the URDF declares has to be visible, because that mismatch is the
  thing being looked for.
- **Highlight.** The link a joint drives directly lights up. With *follow motion*
  ticked, whichever joint is actually turning takes the highlight, so nudging a
  joint by hand tells you which one the map thinks it is.
- **Travel bars.** Two bands on one scale — what the URDF declares, and what was
  measured — with a live needle. Where the URDF reaches past the measured travel,
  the band goes red.

The **URDF ↔ hardware fit** table scores each joint, within a 2° tolerance:

| verdict | meaning |
|---|---|
| `1:1` | measured travel matches the `<limit>` both ends |
| `conservative` | the arm has more travel than the URDF allows — safe, just unused |
| `over-travel` | **the URDF allows more than the arm has** — a planner will command into a hard stop |
| `not in urdf` | a servo mapped to a joint the URDF doesn't declare |
| `no servo mapped` | a URDF joint with no servo behind it |

`over-travel` is the failure; the other direction only wastes reach. **URDF limit
block** prints the measured ranges as paste-ready `<limit>` lines. Nothing on
this page writes the URDF — the build page owns that file.

## Output

**Write to repo** posts to `POST /calibration`, which writes both files into
`src/mod101_control/config/` (already covered by the package's
`install(DIRECTORY config ...)`):

- **`calibration.yaml`** — ROS. Per joint: servo ID, model, sign, the raw tick
  min/max/home, and `limit_lower`/`limit_upper` in radians relative to home.
- **`lerobot_calibration.json`** — LeRobot. Per motor: `id`, `drive_mode`,
  `homing_offset`, `range_min`, `range_max`.

Both are **generated files**. Re-run the wizard rather than hand-editing them.

### The tick math

STS3215 encoders are 4096 counts/rev and direct-drive, so one tick is
`2π/4096` rad and the same counts serve both stacks:

```
ROS:      limit = sign · wrapSigned(ticks − home) · 2π/4096
LeRobot:  homing_offset = 2047 − home
          range_min/max = min/max + homing_offset
          drive_mode    = 1 when sign is −1
```

`wrapSigned` keeps an arc that crosses the 0/4095 seam contiguous instead of
spiking. All of this is covered by the page's **Self-test** button, which runs
29 assertions over synthetic sweeps — including the seam case, both signs, the
URDF fit verdicts, the guided flow's captures (home is the captured default
rather than mid-travel, limits pull `MARGIN` in from both stops, captures either
side of the seam, and the return-to-default residual), and both placements:
which one each shape of joint is told to use, that zero follows the travel to
whichever end of the count it runs toward, and that each placement fits its own
shape while the *other* one overruns 0..4095 — the regression the two buttons
exist for. No hardware attached.

The seam is where this stops being arithmetic: `wrapSigned` handles it here, and
the generated files cannot express it at all. See
[The encoder seam](#the-encoder-seam).

> **Verify the LeRobot schema against your installed version.** The calibration
> file's field names have moved between LeRobot releases. The shape written
> here matches the SO-101 follower layout; confirm before relying on it.

## Handing off to LeRobot

Copy the JSON to where LeRobot looks:

```bash
cp src/mod101_control/config/lerobot_calibration.json \
   ~/.cache/huggingface/lerobot/calibration/robots/<robot_type>/<id>.json
```

LeRobot drives the same chain the same way — its native `FeetechMotorsBus`
opens the USB-TTL adapter directly — so the tick values transfer as-is. Stop
the configurator first; two processes cannot hold the same serial port.

Still open, and not solved by this page:

- **No leader arm.** LeRobot's data collection assumes a backdriven leader.
  mod101 has no leader variant; the options are building one, remapping an
  SO-101 leader, or accepting worse data from keyboard/gamepad teleop.
- **Cameras.** The wrist camera in the URDF is a Gazebo sensor. LeRobot needs a
  real camera config, and which physical camera sits in `wrist_camera_v1_1`
  isn't recorded anywhere in this repo.
- **Datasets don't port across builds.** mod101 is parametric; a policy trained
  on one reach envelope won't transfer to another. Record the build config
  (rail lengths, motor variants, tool) into dataset metadata and key datasets by
  build.

## Demo mode

Ticking **Demo mode** synthesizes a six-servo arm so the whole flow — the five
guided steps, gauges, verdict, arm-check, drive test, config generation — can be
walked without hardware, and without touching a serial port. The synthetic servo
reads back wherever the current step tells you to put the joint, because
otherwise both "hard stops" capture the same tick and the flow rejects them as
too close together: correct behaviour, but it makes the demo unwalkable. The 3D model animates from
the synthetic telemetry too, so the model tab is fully explorable dry.

The synthetic joint's two stops **move with the offset**. A hard stop is a fact
about a mechanism and does not stay a fixed distance from wherever a zero was
just placed: after **Set zero here** the rest pose sits near the bottom of the
count, and a demo servo still sweeping 850 ticks either side of it would run
through 0 into 4095 — manufacturing the seam crossing the placement exists to
prevent, and making the feature look broken in the one mode you can try without
an arm. `demoTravel()` clamps both stops into the count instead, so a zeroed
joint sweeps one way from its stop, the way the real one does.

`?demo=1` and `?tab=model` do the same from the URL, which is how the page gets
driven headless for a screenshot or a check.

## Re-zeroing the model

**Re-zero model**, under the 3D view. A joint's model zero is a guess until the
joint is calibrated: `driveModel` takes the first tick it sees after the bus
opens as that joint's zero, so an arm that happened to be folded up when you
connected draws every joint offset by however far it was — silently, and for the
rest of the session.

The button re-anchors every joint at once. A **calibrated** joint does not guess:
it goes back to its measured home, which is the real thing. An **uncalibrated**
one falls back to "the arm is at its URDF default right now" — a claim about the
arm that the person pressing the button is making, so put it there first. The log
says which joints got which. It repaints from the last tick seen rather than
waiting for the next poll, so it works with the bus closed too.
