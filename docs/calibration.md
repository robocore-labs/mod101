# Calibration

The configurator's **Calibrate** tab (`configurator/calibrate.html`) is the
bring-up path for a physical arm: it sweeps each joint by hand, derives its
travel limits and home position from the motion, verifies the joint is
mechanically healthy, and then emits the two config files that ROS and LeRobot
each need.

The wizard flow is ported from the Axon calibration tool in
`link101/link-base101/web/calibrate.html`, but the transport is not: this talks
to the servos **directly**, with no intermediate firmware. The DDSM210 wheel
control that lived alongside the original in `axon-config.html` is deliberately
not carried over — that's base101 chassis hardware, not ST-series servos.

```bash
pip install st3215                 # once
python3 configurator/server.py     # http://localhost:8000/calibrate.html
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
2. **Verdict** — `clean`, `binding`, or `fault`. A single servo fails only on a
   position discontinuity (`JUMP_THRESH` = 400 ticks ⇒ encoder slip). A pair
   additionally has to hold a constant mirror sum `K`; drift means the pair
   isn't coaxial, reversal means a slipped encoder.
3. **Accept** — adjust min/max/home, or capture each from the current
   hand-held position. The page shows live what those ticks mean in radians for
   the mapped ROS joint.
4. **Arm-check** — re-arms at a slow profile holding present position, then
   reads current. Must be low (< 250 mA), and matched within 100 mA for a pair,
   or it disarms immediately. This is a *soft* check: nothing in this path can
   cap current in hardware.
5. **Drive test** — jog the joint across its calibrated range under power and
   watch the tracking (or mirror-sum) error.

**Nothing is written to the servos.** Calibration lives in the generated repo
files, not in servo EEPROM — no wear, and nothing lost when a servo is swapped.
The only EEPROM write in the whole page is the explicit *Reassign a servo ID*
action.

## Joint mapping

There's no configuration stored on the arm, so the **Servo setup** table is the
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
literally named `6` per [tool-convention.md](tool-convention.md). **Mirror**
pairs a second servo onto the same joint (0 = single); mod101 is six single
servos, so the pair path stays dormant — it's kept because the same wizard
serves arms that do have coupled pitch joints. Press **Rebuild joints** after
changing a mirror.

Edits persist in `localStorage` and travel with the exported JSON.

Getting a sign wrong is the classic failure: sim and hardware move opposite
ways on that one joint. Check each against the URDF before trusting a policy.

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
10 assertions over synthetic sweeps — including the seam case and both signs —
with no hardware attached.

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
without hardware, and without touching a serial port. Useful for UI work and
for seeing what the exports look like.
