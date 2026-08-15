# Performance notes and DDS traps

Measurements from 2026-08-13, taken while bringing up MoveIt on an i9-13900K
under WSL2. Written down because most of the "this feels slow" and "this is
flaky" in this stack turned out **not** to be what it looked like.

Headline: **the hardware and WSL are not the bottleneck.** Bringup is dominated
by fixed `sleep`s in the launch files, and most intermittent failures are DDS
environment problems, not code.

---

## What was measured

| Thing | Result | Verdict |
|---|---|---|
| Gazebo real-time factor | **0.992** (7.64 s sim in 7.70 s wall, GUI running) | Real time. Physics is fine. |
| GPU | `/dev/dxg` present, no llvmpipe, OpenGL 4.5 | Hardware accelerated |
| Filesystem | `/home/cdr` = ext4 | Native. (`/mnt/c` **would** be slow) |
| move_group robot-model parse | 0.002 s | Free |
| OMPL plan (home → ready) | 0.07–0.26 s | Fast |
| IK (pick_ik, 5-DOF) | 0.07–0.21 mm error, well inside its 50 ms budget | Fast |
| CPU / RAM | 32 threads, 31 GB | Idle during startup |
| Collision geometry | 16 meshes, **111,766 triangles**, visual == collision | Coping, but see below |

Gazebo throttles to real time by design, so a 3-second trajectory takes 3
seconds. That's not slowness.

### How to re-measure RTF

```python
# rtf.py — run against a live sim
import time, rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
rclpy.init(); n = Node('rtf')
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
s = []
n.create_subscription(Clock, '/clock', lambda m: s.append(
    (time.time(), m.clock.sec + m.clock.nanosec * 1e-9)), qos)
end = time.time() + 12
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.2)
(w0, t0), (w1, t1) = s[0], s[-1]
print(f'RTF = {(t1 - t0) / (w1 - w0):.3f}')
```

## Where bringup time actually goes

`demo.launch.py` reaches "You can start planning now!" in roughly 16–20 s, and
**almost all of it is hardcoded `TimerAction` delays**, not work:

| Stage | Source | Delay |
|---|---|---|
| joint_state_broadcaster | `gazebo.launch.py` | 3 s |
| arm_controller | `gazebo.launch.py` | 5 s |
| tool gripper_controller | `mod101_tool_*/launch/tool.launch.py` | 10 s |
| arm/gripper trajectory controllers | `demo.launch.py` | 8 s, 10 s |
| move_group | `demo.launch.py` | 16 s |

Measured stage completion in Gazebo: JSB at +2.5 s, arm JTC at +5.3 s, gripper
JTC at +7.5 s, robot model loaded at +13.0 s. The CPU is idle through most of
that. A faster machine does **not** help — the timers are wall-clock.

### The event-driven fix — attempted, and it FAILED

The obvious improvement is to drop the timers: the `spawner` blocks on
`/controller_manager`'s services by itself, so it can start immediately, and
subsequent spawners can chain off `RegisterEventHandler(OnProcessExit(...))`.

**This was implemented and it broke bringup.** Do not retry it blind.

Symptom, in `mock.launch.py`: starting a spawner in the same instant as
`ros2_control_node` makes the controller manager **never receive
`/robot_description`**. It hangs forever printing:

```
[controller_manager]: Waiting for data on 'robot_description' topic to finish initialization
```

In `demo.launch.py` the same change produced a sibling failure — `ros_gz_sim
create` couldn't fetch the description either:

```
[ros_gz_sim]: Failed to get XML from topic [robot_description]
```

Reproduced 3/3 with a freshly cleaned `/dev/shm`, so it is **not** shm
contamination. Relevant facts:

- `controller_manager` 4.45 **ignores a `robot_description` parameter** — it
  only reads the `/robot_description` topic, which `robot_state_publisher`
  latches (transient_local).
- Delaying `ros2_control_node` instead of the spawner does **not** help
  (tested: still 0/3).
- Delay sensitivity on the spawner was non-monotonic — 0.5 s worked, 1.0 s and
  2.0 s failed, 3.0 s worked. That erraticism is unexplained and is the reason
  this wasn't just tuned to a smaller number.

Everything was **reverted to the original timers**, which are verified working.
The delays now carry comments pointing here.

If you pick this up again: the mechanism to understand first is why a
late-joining subscriber misses a latched `robot_description`, and why a third
participant appearing seems to trigger delivery. `OnProcessIO` watching for
`"Received robot description from topic."` before starting spawners is a
plausible event-driven approach that avoids guessing.

## DDS traps

Two separate environment problems, both of which produce failures that look
exactly like code bugs. Between them they cost hours.

### 1. Stale FastDDS shared-memory segments

Every launch killed with SIGINT/SIGKILL can leave shared-memory segments behind
in `/dev/shm`. They accumulate — I reached **174** after ~40 launches — and once
they pile up, discovery starts failing intermittently.

**Symptoms:** controllers randomly failing to load; action servers never
discovered (`Action client not connected to action server:
arm_trajectory_controller/follow_joint_trajectory`, surfacing as MoveIt
`CONTROL_FAILED` / error code `-4`); tests passing and failing with identical
code.

**Confirmed:** `moveit_smoke.py` with `tool:=jaws` failed the execute check 2/2,
cleaned `/dev/shm`, then passed 5/5 twice with no code change.

**Fix**, with no ROS processes running:

```bash
pgrep -af 'ros2|gz sim|rviz'          # confirm nothing is alive first
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
```

Worth doing routinely between heavy test cycles. **This is not WSL-specific** —
it happens on native Linux too.

### 2. Another ROS system on the same DDS domain

A second, unrelated ROS system running on the machine produces the most
misleading failure of all — spawners talk to *its* controller_manager:

```
[spawner]: Controller already loaded, skipping load_controller
[controller_manager]: Could not configure controller with name
                      'joint_state_broadcaster' because no controller with this name exists
```

Those two lines contradict each other, which is the tell. Also seen: leftover
`gz sim` processes from earlier runs still serving a controller_manager.

**Check:** `ros2 node list` — if you see nodes you didn't start, set
`ROS_DOMAIN_ID` to something unused or shut the other system down.

## Bug found and fixed: `/clock` published twice

`/clock` was being bridged **twice** — a dedicated `clock_bridge` node in
`gazebo.launch.py`, *and* a `/clock` entry in `config/gz_ros_bridge.yaml`. Two
processes sampling Gazebo's clock at different moments, publishing to one topic,
so subscribers saw sim time stepping backwards:

```
[tf2_buffer]: Detected jump back in time. Clearing TF buffer.
```

24 occurrences in a 50 s run; MoveIt's TF lookups were being broken by it. It
was latent for a long time because nothing in the plain sim consumed TF on sim
time.

Removed `/clock` from the YAML, keeping the dedicated bridge. **24 → 0.**

**Rule: `/clock` gets exactly one publisher.** Check with
`ros2 topic info /clock --verbose`.

A misdiagnosis worth recording: I first blamed
`{'use_sim_time': LaunchConfiguration(...)}` passing a *string* to a bool
parameter, citing wall-clock log timestamps as evidence. That reasoning was
wrong — **rcl log timestamps always use the wall clock regardless of
`use_sim_time`**, so they prove nothing. The parameter was being set correctly
all along. (The `ParameterValue(..., value_type=bool)` wrapper was kept anyway:
it's correct practice, just not the fix.)

## Things that are fine, despite looking suspicious

- **112k-triangle collision meshes.** Visual and collision geometry are the same
  full-resolution STLs, up to 7.3 MB (`servo_shoulder_big_1.stl`, 146,944
  triangles — only loaded on `mount:=big`). Convex hulls would still be a real
  improvement, but they are not what makes anything slow today.
- **Gazebo at RTF ~1.0.** By design. Trajectories run in real time.
- **`Could not enable FIFO RT scheduling policy`.** Expected without realtime
  privileges; harmless.
