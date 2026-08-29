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
bool, and `ChangeId` returns `None` on *success*.

### HTTP surface

| Endpoint | Purpose |
|---|---|
| `GET /bus` | connection status + known servo IDs |
| `GET /bus/ports` | candidate serial devices (USB only, unless there are none) |
| `POST /bus/connect` | `{port}` — open and scan |
| `POST /bus/disconnect` | disarm + close |
| `POST /bus/scan` | re-ping IDs 1..20 |
| `POST /bus/disarm` | torque off everything |
| `GET /servo/<id>` | telemetry: pos, speed, current, voltage, load, temp |
| `POST /servo/<id>/torque` | `{on}` |
| `POST /servo/<id>/move` | `{pos, speed?, accel?}` |
| `POST /servo/<id>/profile` | `{speed, accel}` |
| `POST /servo/<id>/id` | `{to}` — reassign servo ID (writes EEPROM) |
| `GET /urdf` | the expanded URDF, for the model tab's scene |

Bus scanning is bounded to IDs 1..20 on purpose. `st3215`'s own `ListServos()`
pings 0..253, which is seconds of dead time on a real bus.

The arm is left disarmed at every exit: on disconnect, on server shutdown
(`atexit`), and on tab close (the page fires `/bus/disarm` through
`navigator.sendBeacon`, which survives unload).

## The flow

Per joint, in order:

1. **Sweep** — torque drops, you move the joint by hand stop-to-stop. Positions
   stream in unwrapped (the 0/4095 seam is stitched out) and the wizard reads
   travel limits from the extremes, pulling `MARGIN` = 30 ticks in from each
   hard stop.
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

**Nothing is written to the servos.** Calibration lives in the generated repo
files, not in servo EEPROM — no wear, and nothing lost when a servo is swapped.
The only EEPROM write in the whole page is the explicit *Reassign a servo ID*
action.

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
| 2 | `joint_shoulder` | `shoulder_lift` | +1 |
| 3 | `joint_elbow` | `elbow_flex` | +1 |
| 4 | `joint_wrist_tilt` | `wrist_flex` | **−1** |
| 5 | `joint_wrist_roll` | `wrist_roll` | +1 |
| 6 | `6` (tool joint) | `gripper` | +1 |

**Sign** reconciles the servo's counting direction with the URDF joint axis.
`joint_wrist_tilt` has axis `0 -1 0`, hence −1 by default. The tool joint is
literally named `6` per [tool-convention.md](tool-convention.md).

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
  is calibrated the anchor is wherever the arm was when first seen, during a
  sweep it tracks the midpoint of the travel seen so far (the model re-centres
  as the sweep grows), and on **Accept** it becomes the real home.
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
11 assertions over synthetic sweeps — including the seam case, both signs and
the URDF fit verdicts — with no hardware attached.

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

Ticking **Demo mode** synthesizes a six-servo arm so the whole flow — sweep,
gauges, verdict, arm-check, drive test, config generation — can be walked
without hardware, and without touching a serial port. The 3D model animates from
the synthetic telemetry too, so the model tab is fully explorable dry.

`?demo=1` and `?tab=model` do the same from the URL, which is how the page gets
driven headless for a screenshot or a check.
