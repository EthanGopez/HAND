# Testing the CSV hand-tracking teleop path

Four stages, cheapest first. Each one adds a layer and can only be trusted if
the one before it passed. Stage 0 needs nothing but a built ROS workspace;
stage 3 moves a physical arm.

The pipeline under test is:

```
hand_tracking_demo.py -> palm_center.csv -> palm_csv_node -> /hand/palm_center
    -> palm_teleop_node -> /servo_server/delta_twist_cmds -> Servo -> controller -> arm
```

## Stage 0 — no MoveIt, no simulator, no arm

[`bench_csv_teleop.py`](bench_csv_teleop.py) stands in for the robot: it
publishes a latched robot description and steady joint feedback at a fixed
bent pose, which is all `palm_teleop_node`'s guards need in order to stop
blocking. Everything the teleop node then publishes is what it would send to
Servo for real.

Three terminals, each with ROS and the workspace sourced:

```bash
ros2 run hand_vision palm_csv_node --ros-args \
    -p csv_path:=/workspace/palm_center.csv -p replay:=true
ros2 run hand_teleop palm_teleop_node
python3 diagnostics/csv_teleop/bench_csv_teleop.py --seconds 20 --expect-motion
```

Expected output, which is what the committed recording actually produces:

```
  /hand/palm_center              204 msgs    25.0 Hz
  /servo_server/delta_twist…     361 msgs    20.0 Hz
  non-zero velocity commands      51   (14%)
  commanded speed              median 0.024   max 0.050 m/s
  PASS
```

The palm rate is about 25 Hz live or at `replay_speed:=1.0`, and proportionally
higher if you speed the replay up. The twist rate must stay near 20 Hz
regardless, including while no hand is visible — Servo stops on a 0.2 s gap in
commands, so a stream that pauses is a failure even though the arm would be
still either way.

To test against a live camera instead of the recording, start the tracking demo
and drop `replay:=true`.

**What this proves:** the CSV is read, validated, mapped to the right axes, and
capped. **What it does not:** anything about Servo, the controller, kinematics,
or a robot.

### Reading palm_teleop_node's own log

It prints a throttled status line every second, and that is the first place to
look when nothing moves:

```
ACTIVE | Direction allowed | samples 126 rejected 0 (depth 2) | X +0.000 Y -0.003 Z +0.050 m/s
```

| Field | Meaning when it misbehaves |
| --- | --- |
| `WAITING` | No sample within `stale_after` (0.2 s). The hand left frame, or the vision node is not publishing |
| `BLOCKED` + reason | The guard refused. `Waiting for robot model` means no `/robot_description`; `Waiting for fresh joint feedback` means no `/joint_states` |
| `rejected` climbing | Tracking is glitching: jumps beyond `max_jump`, or repeated timestamps |
| `(depth N)` climbing | Depth spikes being discarded. Harmless while `scale_x` is 0 |
| `ACTIVE` but all zeros | The hand is visible and still. Position commands nothing; only movement does |

## Stage 1 — fake hardware, full Servo stack

```bash
bash scripts/launch_csv_xarm.sh                 # live camera, demo running separately
bash scripts/launch_csv_xarm.sh csv_replay:=true  # recorded CSV, no camera
```

Open <http://localhost:6080/vnc.html> for RViz. Wait for the ready-pose move
(about 12 seconds) — input does not start until it completes.

Note this stack is an **xArm 6**, while the physical arm is a **Lite6**. Stage 1
validates the pipeline, not the arm-specific kinematics.

Checks worth running in a spare terminal:

```bash
ros2 topic hz /servo_server/delta_twist_cmds   # ~20 Hz, always
ros2 topic hz /xarm6_traj_controller/joint_trajectory  # output only while commanding
ros2 topic echo /servo_server/status           # Servo's own warnings
ros2 topic echo /joint_states --once           # positions should change as you move
```

**Measure the actual tool speed here rather than assuming it.** Command a known
velocity, watch how far the tool actually travels, and compare. The Servo config
sets `scale.linear: 0.1` alongside `command_in_type: "speed_units"`, and whether
that scale applies to speed-unit commands is worth confirming empirically rather
than reading off the YAML comment. Our own cap is `max_speed` (0.05 m/s by
default) and the singularity prefilter halts at condition 25, while Servo's own
`hard_stop_singularity_threshold` is 30 — so the prefilter should always trip
first, and seeing Servo halt before our node reports `BLOCKED` would mean the
prefilter is not doing its job.

If the launch fails with `No module named 'launch_param_builder'`, MoveIt is
missing from the container; run `bash scripts/bootstrap_ros_workspace.sh`. This
affects the mouse demo identically and is not specific to the CSV path. Note
that the dev container losing its apt packages on a rebuild does not touch
`ros_ws/install/`, which is on a host bind mount, so a stale build can look
healthy while its libraries are gone.

### Measured result

Run on the fake stack with the committed recording (`csv_replay:=true`):

| Observation | Result |
| --- | --- |
| `palm_teleop_node` state | `ACTIVE \| Direction allowed`, velocities up to the 0.05 m/s cap |
| `/servo_server/status` | `0` (no warning) throughout |
| `/xarm6_traj_controller/joint_trajectory` | ~15 Hz, commanded positions changing |
| `/joint_states` during the active window | largest joint excursion 0.112 rad — **the arm moves** |

One trap worth knowing when you check this yourself: the recording contains a
23.7 s stretch with no hand. Sampling `/joint_states` across that window shows a
perfectly still arm and looks exactly like a broken pipeline. Watch
`palm_teleop_node`'s own log to confirm it is `ACTIVE` with non-zero velocity
before concluding anything from joint motion.

## Stage 2 — Gazebo physics

```bash
bash scripts/launch_mouse_gazebo.sh input_mode:=csv
bash scripts/launch_mouse_gazebo.sh input_mode:=csv csv_replay:=true  # no camera
```

Or directly: `ros2 launch hand_bringup gazebo.launch.py input_mode:=csv`.
`input_mode` defaults to `mouse`, so existing Gazebo usage is unchanged. Extra
arguments are `csv_path`, `csv_replay`, `scale_y`, `scale_z`, `scale_x`. Note
this launch's `max_speed` default is 0.25 m/s, five times the fake stack's.

This is the first stage where commanded and achieved velocity can diverge: the
arm has inertia and the controller tracks a trajectory rather than teleporting
to it. So compare what `palm_teleop_node` logs against what the joints do,
rather than assuming they match.

### Measured result

Verified headless with the committed recording:

| Observation | Result |
| --- | --- |
| `prepare_sim` | `Ready pose reached; enabling teleoperation`, exits cleanly |
| `palm_teleop_node` | `ACTIVE \| Direction allowed`, velocities to 0.087 m/s under the 0.25 cap |
| Tracking gaps | `WAITING`, zero velocity |

Gazebo start-up is slow without a GPU — `prepare_sim` has 90 s service timeouts
and took well over a minute to complete here. A few
`moveit_servo.servo_calcs: run_duration: 0.1 (0.067)` warnings appeared, meaning
Servo missed its target period; that is this container being slow under software
rendering, not a fault in the input path.

## Stage 3 — the physical Lite6

[`hardware_csv.launch.py`](../../ros_ws/src/hand_bringup/launch/hardware_csv.launch.py)
exists, **but has never been run against a robot.** It is the mouse hardware
launch with the pad replaced by the two CSV nodes, keeping every startup gate:
input starts only after `prepare_hardware` passes, and the stack shuts down if
the teleop node, the vision node, the watchdog, or Servo exits.

```bash
bash scripts/launch_csv_hardware.sh 192.168.1.166
```

What differs from the mouse launch, and why:

| Setting | Value | Reason |
| --- | --- | --- |
| `feedback_topic` | `/xarm/joint_states` | The driver's successful-read topic. `/joint_states` would let the guard trust feedback the hardware never confirmed |
| `max_speed` | 0.03 m/s (mouse: 0.05) | This input has no deadman, so start slower |
| `scale_x` | 0.0 | Depth is centimetres but too noisy to trust until measured |
| `replay` | forced `False` | Replaying a recording into a physical arm is never wanted |

What has been checked offline: the launch builds its arguments, and `robot_ip`
validation rejects `auto`, loopback, unspecified, and malformed addresses before
any action that could contact a controller. Nothing else about it is verified.

Before trusting it:

* Read the hardware section of
  [`hand_teleop/README.md`](../../ros_ws/src/hand_teleop/README.md) first. It
  documents a past incident of unexpected physical motion during startup.
* Run stages 0 and 1 against the Lite6 description, not the xArm 6 one:
  `bench_csv_teleop.py` already defaults to the Lite6 snapshot in
  `diagnostics/lite6/model_snapshot.urdf`.
* Dry-run with the arm powered but **not** enabled, or on a separate
  `ROS_DOMAIN_ID`, and confirm the twist stream looks exactly as it did in
  stage 0 before any motor is live.
* Have the physical emergency stop in hand. Servo's command timeout and our
  guards are software stops, not an e-stop.

### The failure mode specific to this input

The mouse pad is a deadman: release the button and the arm stops. The CSV path
has no such thing. Motion stops because samples stop arriving — and samples stop
because the tracking demo stopped writing. So the stop path now depends on a
separate process, a file, and a 0.2 s timeout, rather than on a finger.

Two of the three stop paths have been measured at stage 0:

| Event | Result |
| --- | --- |
| `palm_csv_node` killed while commanding 0.05 m/s | Not one non-zero command afterwards; stopped inside one 50 ms tick. The 20 Hz stream stayed alive publishing zeros, which is what Servo needs |
| Hand leaves frame (the recording's 23.7 s gaps) | `WAITING`, zero velocity, stream alive |

The third — killing the tracking demo itself — is the same thing from
`palm_csv_node`'s point of view, since both simply stop appending rows. Worth
confirming on fake hardware anyway, together with the demo's own five-second
tripwire timeout, before relying on any of it with a physical arm.

One warning from measuring this: `subprocess.terminate()` on a `ros2 run`
command kills only the wrapper and **orphans the node**, which keeps publishing.
An early version of this test appeared to show four seconds of runaway motion
that was entirely the orphan. Use `start_new_session=True` and `os.killpg`, or
check with `pgrep -af palm_csv_node` afterwards.
