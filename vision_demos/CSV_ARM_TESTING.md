# Testing the CSV nodes on a fake arm and a real arm

This is the short runbook for the two nodes that turn the hand-tracking CSV into
arm motion. The longer test plan, with measured results and failure analysis, is
[`diagnostics/csv_teleop/README.md`](../diagnostics/csv_teleop/README.md); this
page is the order to do things in and the commands to type.

```
hand_tracking_demo.py -> palm_center.csv -> palm_csv_node -> /hand/palm_center
    -> palm_teleop_node -> /servo_server/delta_twist_cmds -> MoveIt Servo -> controller -> arm
```

| Node | Package | Job |
| --- | --- | --- |
| `palm_csv_node` | `hand_vision` | Follows the CSV and publishes each new row as `PalmCenter` |
| `palm_teleop_node` | `hand_teleop` | Turns hand *movement* into a capped Cartesian tool velocity, run past the singularity and joint-feedback guards |

Neither node builds a trajectory. `palm_teleop_node` publishes a **velocity**
(`TwistStamped`, 20 Hz); MoveIt Servo converts that into the joint trajectory the
arm controller executes. So "does the arm follow the trajectory" is really "does
the velocity stream look right, and does Servo turn it into motion".

## The CSV

`hand_tracking_demo.py` writes `palm_center.csv` in the repo root:

```
timestamp_ms,center_x,center_y,center_z
```

All three values are in **centimetres**: `x`, `y` are the palm's offset from image
centre and `z` is its distance from the camera (confirm with a ruler, see below).
Depth is much noisier than `x`, `y`, so `scale_x` defaults to 0 and depth does not
move the arm. Recordings made before this change have `x`, `y` in metres and will
replay about 100x too slowly. Rows are written only
while a hand is tracked. Silence means "no hand", and the arm must stop.

Two ways to feed the nodes:

* **Live:** run `python3 vision_demos/hand_tracking_demo.py` in its own terminal.
  Nodes follow new rows only.
* **Replay (no camera):** pass `csv_replay:=true` to a launch, or `-p replay:=true`
  to the node. It replays the whole file at recorded speed (`replay_speed`
  scales it). **Fake arm only. Never replay into a real arm.** The hardware
  launch forces `replay` off.

## Axes and the `scale_*` settings

The scale names refer to the **robot** axis they drive, not the CSV column:

| Hand motion | CSV column | Tool axis | Parameter | Default | Sign |
| --- | --- | --- | --- | --- | --- |
| Toward / away from camera | `center_z` | X | `scale_x` | **0** (depth off) | away from camera -> tool -X |
| Left / right in the image | `center_x` | Y | `scale_y` | 0.01 | right -> tool -Y |
| Up / down in the image | `center_y` | Z | `scale_z` | 0.01 | up -> tool +Z |

Each scale is **metres of tool travel per centimetre of hand motion**, so `0.01`
tracks the hand one-for-one and `0.005` moves the tool half as far. The node
multiplies how far the hand moved between ticks by the scale, divides by the
elapsed time to get a velocity, then caps the total at `max_speed`. Only
movement commands anything; a still hand, however far off centre, commands zero.

`scale_x` is the one that is off by default: depth comes from the palm's pixel
width, which is much noisier than the image-plane position. Set it with
`scale_x:=0.01` on a launch (or `-p scale_x:=0.01` on the node) once the other two
axes look right. The signs come from `palm_input.py` and match `mouse_teleop`; if
the arm moves the wrong way on screen, the cause may be a mirrored camera preview
or the arm's base frame rather than the code.

## Recordings for replay

Replay needs a file in the current all-centimetre units. Starting
`hand_tracking_demo.py` **truncates `palm_center.csv` to its header**, so any
recording left there is lost the next time the demo runs. Keep recordings under
other names.

**Record a real one:**

```bash
python3 vision_demos/hand_tracking_demo.py     # move your hand for ~30 s, then quit
wc -l palm_center.csv                          # should be hundreds of rows, not 1
cp palm_center.csv vision_demos/recording_cm.csv
```

Pass it with `csv_path:=/workspace/vision_demos/recording_cm.csv` on a launch, or
`-p csv_path:=...` on the node. Recordings made before the all-centimetre change
have `x`, `y` in metres and replay about 100x too slowly.

**Or generate a synthetic one (no camera, no hand):**

```bash
python3 vision_demos/make_synthetic_palm_csv.py vision_demos/synthetic_cm.csv
```

[`make_synthetic_palm_csv.py`](make_synthetic_palm_csv.py) writes a made-up hand,
not a real one: a 6 cm circle every 10 s in the image plane while depth swings
+/-5 cm around 40 cm, at 25 Hz for 30 s. It exercises every axis. Options:
`--seconds`, `--radius`, `--period`, `--depth-swing`. Expect `Y` and `Z` commands
to alternate sign about every 5 s, and `X` only if `scale_x` is non-zero. It
can't show how the tracker behaves with real noise, lost hands or glitches, so
use a real recording for that.

Example fake-arm replay with the synthetic file:

```bash
bash scripts/launch_csv_xarm.sh csv_replay:=true \
    csv_path:=/workspace/vision_demos/synthetic_cm.csv scale_x:=0.01
```

## Order of testing

Do these in order. Don't go to the next until the previous one passed.

### 0. Nodes only, no arm and no MoveIt (about 1 minute)

[`bench_csv_teleop.py`](../diagnostics/csv_teleop/bench_csv_teleop.py) fakes the
robot description and joint feedback so the guards stop blocking. **Do not run
`hand_tracking_demo.py` for this stage:** the recording stands in for it, and the
demo would overwrite `palm_center.csv` (it opens the file in write mode).

Replay plays the file **once**, so start the vision node last. Three terminals,
each with ROS and the workspace sourced
(`source /opt/ros/humble/setup.bash && source ros_ws/install/setup.bash`):

```bash
# A: start first
ros2 run hand_teleop palm_teleop_node

# B: start second, so it is listening before any motion
python3 diagnostics/csv_teleop/bench_csv_teleop.py --seconds 10 --expect-motion

# C: start last, right after B
ros2 run hand_vision palm_csv_node --ros-args \
    -p csv_path:=/workspace/vision_demos/synthetic_cm.csv -p replay:=true -p replay_speed:=3.0
```

Replay needs a recording in the current units; see "Recordings for replay" above.
Check `wc -l` on the file first, since the demo truncates `palm_center.csv`. Pass its
path as `csv_path`. The command above uses the synthetic file, which has a hand in
frame from the first row, so motion begins at once; with your own recording, motion
starts when the hand first appears. `replay_speed:=3.0` and `--seconds 10` keep the run short. For the doc's original
numbers use `replay_speed:=1.0` and `--seconds 20`. If a short run reports no
non-zero velocity, raise `--seconds` before suspecting the nodes. To re-run,
`pkill -f palm_csv_node` first, then start again in the same order.

What you should see:

* **B:** `PASS`, with non-zero velocity commands, a median and max speed no higher
  than `max_speed`, and a twist stream near 20 Hz even with no hand visible.
  The palm rate scales with `replay_speed` (about 25 Hz at 1.0, about 75 Hz at 3.0).
* **A:** `BLOCKED: Waiting for robot model` until B starts, then `ACTIVE` while the
  replay runs, then `BLOCKED: Waiting for fresh joint feedback` once B finishes.
  That last change is correct: B stopped publishing feedback and the guard refused
  to command without it.
* **C:** prints a startup line, then goes quiet. It logs only on start and on
  file-status changes, not per row.

This proves CSV parsing, axis mapping and the speed cap. It says nothing about
Servo or an arm.

### 1. Fake arm: full Servo stack (xArm 6, fake hardware)

```bash
bash scripts/launch_csv_xarm.sh csv_replay:=true     # recorded CSV, no camera
bash scripts/launch_csv_xarm.sh                      # live; start hand_tracking_demo.py yourself
```

Open <http://localhost:6080/vnc.html> to see RViz. Input starts only after the
ready-pose move finishes (about 12 s). For a live run, start the launch first and
`hand_tracking_demo.py` after that move; old rows in the CSV are skipped when the
node starts, and the reader reopens the file if the demo restarts. Useful options: `max_speed`, `scale_y`,
`scale_z`, `scale_x`, `csv_path`. Each scale is metres of tool per cm of hand, and
`0.01` is 1:1; `scale_y` and `scale_z` already default to it.

In a second terminal:

```bash
ros2 topic hz /servo_server/delta_twist_cmds            # ~20 Hz, always
ros2 topic hz /xarm6_traj_controller/joint_trajectory   # only while commanding
ros2 topic echo /servo_server/status                    # 0 = no Servo warning
ros2 topic echo /joint_states --once                    # positions change as you move
```

What to look at: `palm_teleop_node`'s once-a-second log line
(`ACTIVE | Direction allowed | ... X Y Z m/s`). If `/joint_states` isn't moving,
check that line first. The recording has a 23.7 s stretch with no hand, so a
still arm there is correct (`WAITING`), not a broken pipeline.

#### Depth (toward and away from the camera)

Depth is off by default (`scale_x` 0), so at first the arm follows only left/right
and up/down. `center_z` is the hand's distance from the camera, most likely in
centimetres. Confirm that first: run the demo, hold your hand 30 cm from the lens
with a ruler, and the on-screen `Depth:` should read about 30. Then:

```bash
bash scripts/launch_csv_xarm.sh scale_x:=0.01      # live; 0.01 m of tool per cm of hand
bash scripts/launch_csv_xarm.sh scale_x:=0.005     # gentler, if it jitters
```

`scale_x:=0.01` makes the tool track the hand one-for-one; start lower if unsure.
(`csv_replay:=true` replays the recording, which may predate the cm change and
use other units, so judge depth with live runs.) Things to check:

* **Direction:** tool X is `-dz * scale_x`, so moving your hand away from the
  camera pushes the tool toward -X. If that is the wrong way for your setup, try a
  negative `scale_x` and see whether it is accepted.
* **Noise:** the teleop log shows `(depth N)`. A count that climbs fast means depth
  steps over `max_depth_jump` (1.0 cm per sample) are being discarded, so the
  signal is too noisy to trust.
* Keep `scale_x` at 0 on the real arm until it behaves on the fake one.

Then run the stop tests, because the CSV input has no deadman:

1. Start live, move your hand so the arm moves, then **cover the camera or drop
   your hand**. Arm must stop within a fraction of a second.
2. Move the arm, then **Ctrl+C `hand_tracking_demo.py`**. Arm must stop.
3. Move the arm, then kill `palm_csv_node` (`pkill -f palm_csv_node`). Arm must
   stop and the twist stream must keep publishing zeros at 20 Hz.

If a node stays alive after you stop a test, check `pgrep -af palm_csv_node`.
Killing a `ros2 run` wrapper can orphan the node.

Optional physics check (slow without a GPU, allow a few minutes to start):

```bash
bash scripts/launch_mouse_gazebo.sh input_mode:=csv csv_replay:=true
```

### 2. Real arm: Lite6

> **Not yet run against a robot.** The launch builds its arguments and rejects bad
> IPs; nothing else about it is verified. Read the hardware section of
> [`hand_teleop/README.md`](../ros_ws/src/hand_teleop/README.md) first. It
> documents an earlier incident of unexpected physical motion at startup.

Before connecting:

* Someone has a hand on the **physical emergency stop** for the whole session.
  Software stops (Servo timeout, our guards, Ctrl+C) are not an e-stop.
* Put the arm in a clear, bent, non-singular pose with its own controls. There
  is no automatic ready-pose move; Servo may halt from a straight pose.
* Stop every other command publisher or simulation. Use a private domain:
  `export ROS_DOMAIN_ID=42`.
* Rebuild if needed: `bash scripts/build_ros_workspace.sh`.
* You need the controller's IPv4 address. There is no discovery for Lite6.
* Repeat stage 0 and the stage 1 stop tests against the Lite6 model. The bench
  already defaults to `diagnostics/lite6/model_snapshot.urdf`.
* Dry-run with the arm powered but **not enabled**, and confirm the twist stream
  looks the same as in stage 0 before any motor is live.

Run (replace the IP with your controller's):

```bash
# terminal 1
bash scripts/launch_csv_hardware.sh 192.168.1.166
# terminal 2, only once the stack reports ready
python3 vision_demos/hand_tracking_demo.py
```

Defaults are deliberately gentler than the fake arm: `max_speed` 0.03 m/s,
`scale_x` 0, feedback from the driver's `/xarm/joint_states`. Start at those.
Launching lets the vendor driver clear faults, enable motors and select Servo mode.

First live attempt. Start the demo only after the launch is up and the arm is
still; do not start it first, because a hand already in frame would begin
commanding motion the moment the nodes come up.

1. Launch without the tracking demo running. Confirm `prepare_hardware` passes,
   the arm does not move, and the teleop log shows `WAITING`.
2. Start the demo with your hand out of frame for a moment, then bring it in and
   move it **a small amount**. Expect a slow tool move
   in the matching direction. Hand right/left maps to tool Y, up/down to tool Z.
3. Repeat the three stop tests from stage 1, now slowly, with the e-stop in reach.
4. Only then raise `max_speed` in small steps (for example `max_speed:=0.05`).

To stop: end the demo (or drop your hand) first, then Ctrl+C the launch. The
e-stop is still the real stop.

The stack shuts down on its own if joint feedback is lost for 0.5 s, if either
CSV node, the watchdog, or Servo exits, or if the readiness check fails. It does
not restart itself; relaunch deliberately.

## Quick reference: when nothing moves

| Log line / symptom | Meaning |
| --- | --- |
| `WAITING` | No sample in the last 0.2 s: no hand, demo not running, or wrong `csv_path` |
| `BLOCKED: Waiting for robot model` | No `/robot_description` received |
| `BLOCKED: Waiting for fresh joint feedback` | No fresh `/joint_states` (fake) or `/xarm/joint_states` (real) |
| `rejected` count climbing | Tracking jumps beyond `max_jump` or repeated timestamps |
| `ACTIVE` but zeros | Hand visible and still; only movement commands velocity |
| `No module named 'launch_param_builder'` | MoveIt missing; run `bash scripts/bootstrap_ros_workspace.sh` |
