# Hand and mouse teleoperation

For node responsibilities, data flow, startup, timing, and implementation details,
see [ARCHITECTURE.md](ARCHITECTURE.md).

Both input nodes publish `geometry_msgs/TwistStamped` velocities to
`/servo_server/delta_twist_cmds`. MoveIt Servo converts Cartesian tool motion
into joint trajectories; `xarm6_traj_controller` executes those trajectories.
The input nodes do not calculate inverse kinematics themselves.

## Start the Gazebo mouse simulation

From the repository root, after building the ROS workspace:

```bash
bash scripts/launch_mouse_gazebo.sh
```

This starts **Gazebo Classic physics** with the xArm 6 on a table, a live RViz
view of Gazebo's joint feedback, and a mouse control window.
Open <http://localhost:6080/vnc.html> and connect. In the VS Code dev
container, forward port 6080 if necessary. Drag inside the mouse pad to control
the Gazebo-simulated arm visible in RViz. Wait for the ready move and mouse window.
Run only one arm demo at a time; stop the previous launcher with Ctrl+C first.

For an existing desktop, source ROS and the workspace and run:

```bash
ros2 launch hand_bringup gazebo.launch.py
```

The launch uses `gazebo_ros2_control/GazeboSystem`; Gazebo owns the controller
manager and publishes joint states. Servo and mouse messages use Gazebo's
`/clock` (`use_sim_time=true`). No fake controller manager or physical driver
is started. The bundled table world's physics settings are retained.

The native Gazebo client is optional because it hangs before creating a window
in the current container, while the physics server and RViz work. On a desktop
where the native client works, enable it with:

```bash
bash scripts/launch_mouse_gazebo.sh gui:=true
```

Use `show_rviz:=false` to omit RViz. Headless Gazebo still runs the physics and
controller stack; RViz is only a viewer, not the simulation backend.

## Start the lightweight RViz simulation

From the repository root, after building the ROS workspace:

```bash
bash scripts/launch_mouse_xarm.sh
```

Open <http://localhost:6080/vnc.html>, connect, and use the mouse control window
beside RViz. In VS Code's dev container, forward port 6080 if necessary.
The launcher uses fake hardware and initializes the six joints to a bent pose
before opening mouse control, normally about 12 seconds after launch. The
initializer pauses Servo, waits for the ready trajectory to complete through
the controller's action interface, and restarts Servo. If initialization fails,
the launch shuts down instead of enabling mouse input at the singular pose.

To launch directly on an existing desktop:

```bash
source /opt/ros/humble/setup.bash
source ros_ws/install/setup.bash
ros2 launch hand_bringup sim.launch.py input_mode:=mouse max_speed:=0.25
```

## Mouse control of the physical Lite6

A separate launcher connects to the real controller. Stop all simulation and
other command publishers first. Use a dedicated ROS domain for this stack if
other robots or demos share your network (`export ROS_DOMAIN_ID=42`, for example).

Build the corrected vendor driver, controller plugin, and HAND packages:

```bash
cd /workspace
bash scripts/build_ros_workspace.sh
```

The build applies `patches/xarm_communication_faults.patch` to the pinned vendor
submodule idempotently. A changed vendor revision or conflicting source changes
stop the build for review rather than silently omitting the corrections.

Place the arm in a clear, non-singular bent pose using its existing controls
before connecting. Check that the controller is reachable from the container
and the physical emergency stop is accessible. **Starting this launcher lets
the vendor driver clear faults, enable motors, and select Servo mode.**
There is no automatic ready-pose trajectory; the arm starts from its measured
current joint positions. A straight/singular pose may cause Servo to halt.

With Ethernet configured on the arm subnet, supply the Lite6 controller IP:

```bash
cd /workspace
bash scripts/launch_mouse_hardware.sh 192.168.1.166
```

Open <http://localhost:6080/vnc.html>. The mouse pad opens only after both
controllers are active, both the broadcaster and successful driver reads have
fresh six-joint feedback, and Servo's start service succeeds. Startup failure shuts down the launch.
A continuous watchdog then checks the successful driver-read topic
`/xarm/joint_states`. Invalid, replayed, or missing feedback for more than
0.5 seconds latches a failure and shuts down the launch. Restored data does not
resume input automatically.

The patched hardware plugin reports failed reads/writes to ros2_control and
blocks later SDK commands until the hardware is reloaded. It makes no blocking
controller-switch service calls from the update loop. Activation requires a
successful measured joint sample and seeds hold commands from those positions.
It does not clear controller faults as part of activation. SDK read latency is
still bounded by the SDK's existing timeout; this patch does not establish the
cause of the original network/read stall. These are software controls, not a
physical emergency stop or proof that the arm stops within a specified distance.

Defaults are **0.05 m/s** total tool speed and **0.0001 m/pixel** gain.
The same hold-and-drag controls apply; closing the mouse window shuts down the
stack. Ctrl+C stops the launch. GUI stopping and Servo's command timeout are
software stops, not a physical emergency stop.

On an existing ROS desktop, source the workspace and use:

```bash
ros2 launch hand_bringup hardware_mouse.launch.py robot_ip:=192.168.1.166
```

Both entry points require an explicit controller IPv4 address. The standalone
`discover_xarm.py` helper identifies xArm 6, not Lite6, and must not select a
controller for this Lite6 model. Replace `192.168.1.166` with your controller's
address if different.

The automatic move to `[0, 9.93, 31.8, 0, 21.87, 0]` degrees was removed after
unexpected physical motion was reported. The log showed one accepted startup
action, no completed result, and no mouse node before shutdown. It did not
record joint samples or controller tracking feedback sufficient to establish
the circular motion's cause. Do not restart physical operation merely to
reproduce the incident. The saved angles are a reference, not an automatic goal;
this change does not establish that the physical stack is safe or validated.

Optional arguments are `max_speed`,
`mouse_gain`, and `show_rviz`. This launch is for an unprefixed, bare Lite6;
it does not configure an attached gripper, tool geometry, payload, or real-world
obstacles. Configure those for your installation before operation. Servo checks
only the robot and objects present in its planning scene.

The hardware configuration and startup checks are tested offline. Physical-arm
motion, stopping distance, and communication-loss behavior have not been
validated on a connected robot.

## CSV hand control

`palm_teleop_node` drives the arm from the hand positions that
[`vision_demos/hand_tracking_demo.py`](../../../vision_demos/hand_tracking_demo.py)
records, by way of `hand_vision`'s `palm_csv_node`. The two nodes split the work
the same way the live hand mode does: one reads positions, one turns them into
velocities. No camera is opened by either of them.

```bash
# terminal 1 — the tracking demo, writing palm_center.csv
cd /workspace && python3 vision_demos/hand_tracking_demo.py

# terminal 2 — the simulated arm plus both CSV nodes
bash scripts/launch_csv_xarm.sh
```

Open <http://localhost:6080/vnc.html> to watch the arm in RViz. There is no
control window: the hand in front of the camera is the input.

To check the pipeline with no camera at all, replay the recorded CSV instead:

```bash
bash scripts/launch_csv_xarm.sh csv_replay:=true
```

Replay honours the recorded frame timing, including the long gaps where the
hand left the frame, so the arm correctly does nothing during them.

For Gazebo physics instead of fake hardware:

```bash
bash scripts/launch_mouse_gazebo.sh input_mode:=csv
```

`input_mode` defaults to `mouse`, so existing Gazebo usage is unchanged. That
launch's `max_speed` default is 0.25 m/s, five times the fake stack's. Physics
is the first place commanded and achieved tool velocity can differ, because the
arm has inertia and the controller tracks a trajectory rather than teleporting
to it.

### Controls

| Hand motion | Motion relative to `link_base` |
| --- | --- |
| Move right / left | Negative / positive Y velocity |
| Move up / down | Positive / negative Z velocity |
| Move toward / away from the camera | Positive / negative X velocity, **disabled by default** |
| Hold the hand still | Zero velocity on the next update |
| Leave the frame, or stop the demo | Zero velocity after `stale_after` (0.2 s) |

**Hand speed sets arm speed, and hand position commands nothing.** This is the
same relative mapping the mouse pad uses, for the same reason: an absolute
mapping would move the arm the instant tracking started, by however far the
hand happened to be from the centre of the frame. Here, displacement between
samples is divided by the time between them, so a hand that stops moving stops
the arm even while it is still plainly visible.

Every CSV coordinate is in centimetres, and each scale is metres of tool
travel per centimetre of hand motion. Y and Z default to **0.01**, meaning the
tool is asked to travel as far as the hand did. X is scaled from the demo's depth estimate, the hand's distance
from the camera in centimetres, which is far noisier than the image-plane
position. So `scale_x` defaults to **0.0** and the arm does not move in X.
`0.01` tracks the hand one-for-one; start lower and raise it deliberately.

### Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `max_speed` | `0.05` | Hard cap on total tool speed in m/s, diagonals included |
| `scale_y` | `0.01` | Metres of tool travel per cm of hand motion across the frame; 0.01 is 1:1 |
| `scale_z` | `0.01` | Metres of tool travel per cm of hand motion up the frame; 0.01 is 1:1 |
| `scale_x` | `0.0` | Metres of tool travel per cm of hand distance from the camera; 0 disables depth, 0.01 is 1:1 |
| `max_jump` | `6.0` | A larger single-sample image-plane step, in cm, is treated as a glitch |
| `max_depth_jump` | `1.0` | A larger single-sample depth step, in cm, is discarded on its own |
| `max_gap` | `0.5` | A longer gap between samples re-establishes the reference instead of commanding the difference |
| `deadband` | `0.0` | Displacement below this is treated as jitter |
| `stale_after` | `0.2` | Seconds without a sample before the arm is commanded to stop |

### Inputs this node refuses

`PalmInput` applies the mouse pad's rules, and two more that the recorded data
made necessary. In the sample CSV in this repository, about 3% of consecutive
sample pairs differ by more than 6 cm in a single frame, with the largest
single-frame step exceeding 1.5 m. These are not hand movements: the demo
derives metres from the measured palm width, so one badly fitted frame rescales
the whole position. Such a step is rejected and its position becomes the new
reference, rather than becoming a lunge.

Depth is checked separately and more loosely, because it is noisy enough to
spike by tens of units between frames. A depth spike discards only the depth
component of that sample; it does not cancel sideways motion the camera saw
perfectly well.

A gap longer than `max_gap` is not a drag. The recorded CSV contains gaps of
over twenty seconds, after which the hand reappears elsewhere in the frame;
the difference across such a gap is discarded.

Every velocity then passes through the same pose-aware singularity filter the
mouse pad uses, described under *How the implementation works* below, and the
same joint-feedback freshness check. Without a robot model or with stale joint
feedback, this node publishes zero and logs the reason, exactly as the mouse
node does.

### CSV control of the physical Lite6

[`hardware_csv.launch.py`](../hand_bringup/launch/hardware_csv.launch.py) is the
hardware equivalent, and carries every warning in the mouse hardware section
above, plus one of its own: **this input has no deadman.** The mouse pad stops
the arm when the button is released, the pointer leaves the pad, or the window
closes. Here the arm stops only because samples stop arriving, which depends on
a separate tracking process continuing to write a file.

```bash
bash scripts/launch_csv_hardware.sh 192.168.1.166
```

Defaults are **0.03 m/s** total tool speed (lower than the mouse launch's 0.05
for the reason above), `scale_x` 0.0 so depth does not move the arm, and
`feedback_topic` `/xarm/joint_states` so the guard uses the driver's
successful-read topic. Optional arguments are `max_speed`, `csv_path`,
`scale_y`, `scale_z`, `scale_x`, and `show_rviz`.

Start the tracking demo yourself, in its own terminal. With no hand in frame the
demo writes no rows and the arm stands still.

**This launch has never been run against a robot.** What is checked offline: it
builds its arguments, and `robot_ip` validation rejects `auto`, loopback,
unspecified, and malformed addresses before any action that could contact a
controller. Physical motion, stopping distance, and communication-loss behaviour
are unvalidated for this input, exactly as they are for the mouse input.
See [diagnostics/csv_teleop/README.md](../../../diagnostics/csv_teleop/README.md)
for the staged test plan to work through first.

### What this pipeline has and has not been shown to do

The two nodes, the message, the CSV following, and the velocity mapping are
tested offline. The full chain has been run on the fake-hardware stack with the
recorded CSV: Servo reported status `0` throughout, the trajectory controller
received commands at about 15 Hz, and the arm's largest joint excursion was
0.112 rad, so simulated motion works end to end. Physical arm motion, and the
whole path from a live camera, remain unvalidated.

## Mouse controls

| Input | Motion relative to `link_base` |
| --- | --- |
| Hold left button and drag right / left | Negative / positive Y velocity |
| Hold left button and drag up / down | Positive / negative Z velocity |
| Hold left button + Shift and drag up / down | Positive / negative X velocity; Z is zero |
| Stop moving the mouse while holding the button | Zero velocity on the next update |
| Release button, leave pad, or switch focus | Zero velocity; click again to resume |
| Escape or close window | Send zero velocity and exit mouse control |

For a trackpad, hover inside the pad and **hold Space while moving the pointer**
instead of holding a mouse click. Shift still selects X. Releasing Space, leaving
the pad or switching focus stops motion. After a stop, release Space and press
it again inside the pad. Two-finger scrolling is not a movement input.

The sensitivity slider multiplies the launch-time `mouse_gain` by 1–5 while
retaining the launch-time total speed cap. Changing it stops the current stroke.
If short trackpad strokes request very low speeds, increase sensitivity first.
For example, with `mouse_gain:=0.0002`, 100 pixels/second requests 0.02 m/s at
1x and 0.10 m/s at 5x, before filtering. Watch the displayed X/Y/Z command speeds
and any `BLOCKED` reason to distinguish small inputs from rejected directions.
The last rejection reason stays visible for two seconds after the input stops;
`Last blocked input` describes the earlier input, not a new rejection.

**Drag speed sets arm speed.** The mapping uses pixels traveled divided by
elapsed wall time, multiplied by `mouse_gain` (default: 0.001 meters/pixel).
For example, 100 pixels/second requests 0.1 m/s; 200 pixels/second requests
0.2 m/s. A 100-pixel stroke requests about 10 cm of travel before speed limits
and Servo filtering. Pointer position on the pad has no effect. Clicking alone
causes no motion. The total translation speed is capped at `max_speed`, including
diagonal motion. Both mouse launchers default to 0.25 m/s (25 cm/s).
Angular velocities remain zero, so this input requests no tool rotation.
Shift changes the requested Cartesian axis; X's on-screen appearance depends
on the Gazebo or RViz camera angle.

To adjust sensitivity or the speed cap:

```bash
bash scripts/launch_mouse_gazebo.sh mouse_gain:=0.0015 max_speed:=0.3
```

Doubling `mouse_gain` doubles requested speed for the same drag speed, until
the cap is reached. Release and reposition the pointer to make another stroke.

## How the implementation works

`MouseInput` contains the input mapping and speed limit independently of ROS.
`MouseTeleop` owns the Tkinter window and publishes freshly stamped commands
every 50 ms (20 Hz). Each message uses the robot-base frame `link_base`.
Mouse events accumulate displacement between ticks. Each tick consumes that
displacement once, so holding still immediately removes the requested motion.
Shift changes discard pending displacement to prevent replaying Z input as X.
Mouse measurement uses a monotonic wall clock independently of ROS time.

Each mouse command passes through a pose-aware singularity filter before it is
published. There is no fixed blacklist of mouse directions: the filter uses the
configured URDF and fresh joint feedback to predict 0.3 seconds of the requested
translation. Near a singularity (Jacobian condition at least 17), drags predicted
to worsen it are ignored. Predictions reaching condition 25 or approaching a
configured joint position/speed limit are also blocked. The window displays
`BLOCKED` and the reason. A blocked diagonal is discarded completely, and
discarded pixels are never replayed. Reversing the drag is evaluated anew.

For the last reported pose, a simulation predicts that left (+Y), down (-Z),
and Shift-up (+X) approach a singularity; these directions can change at another
pose. Separately, the reported J1=204.43 degrees exceeds the physical launch's
existing +/-178.2-degree model range. That pose blocks every direction with a
joint-limit message. This filter does not widen those limits.

Missing/invalid model data or joint feedback stops the mouse output. Feedback
older than 0.2 seconds disarms the mouse and requires another click. Hardware
uses the driver's successful-read `/xarm/joint_states`; simulation uses
`/joint_states`. This predictor supplements Servo and the hardware fault guards;
it cannot guarantee avoidance of every physical singularity or stopping distance.

Button release, pointer exit, and focus loss disarm the input and immediately
publish zero velocity. The GUI and publisher share one event loop; if that
loop stalls, it cannot continue publishing a held motion command. Servo's
configured 0.2-second incoming-command timeout stops a lost command stream.
When the GUI resumes after a stall longer than 0.2 seconds, it disarms and
requires a new click. Servo separately handles collision proximity,
singularities, and joint limits, so the executed motion may be slowed or halted.

`sim.launch.py` selects one input with `input_mode:=hand` (the default) or
`input_mode:=mouse`. Hand vision and hand teleoperation are omitted in mouse
mode. Do not run an additional command publisher alongside the chosen input.

Tkinter is supplied by the `python3-tk` runtime dependency. Mapping and stop
behavior are checked in `test/test_mouse_input.py`.
