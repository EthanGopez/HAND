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

## Mouse control of the physical xArm 6

A separate launcher connects to the real controller. Stop all simulation and
other command publishers first. Use a dedicated ROS domain for this stack if
other robots or demos share your network (`export ROS_DOMAIN_ID=42`, for example).

Build the new entry point once:

```bash
cd /workspace/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select hand_bringup hand_teleop
```

Place the arm in a clear, non-singular bent pose using its existing controls
before connecting. Check that the controller is reachable from the container
and the physical emergency stop is accessible. **Starting this launcher lets
the vendor driver clear faults, enable motors, and select Servo mode.**
There is no automatic ready-pose trajectory; the arm starts from its measured
current joint positions. A straight/singular pose may cause Servo to halt.

With Ethernet configured on the arm subnet, start without an IP:

```bash
cd /workspace
bash scripts/launch_mouse_hardware.sh
```

Open <http://localhost:6080/vnc.html>. The mouse pad opens only after both
controllers are active, all six joint positions have fresh feedback, and
Servo's start service succeeds. Startup failure shuts down the launch.
Defaults are **0.05 m/s** total tool speed and **0.0001 m/pixel** gain.
The same hold-and-drag controls apply; closing the mouse window shuts down the
stack. Ctrl+C stops the launch. GUI stopping and Servo's command timeout are
software stops, not a physical emergency stop.

On an existing ROS desktop, source the workspace and use:

```bash
ros2 launch hand_bringup hardware_mouse.launch.py
```

Both entry points default to `robot_ip:=auto`. Discovery sends only the SDK's
read-only version request on port 502 and requires exactly one identified xArm 6.
It does not enable motors; the hardware driver launched afterward does. No match
or multiple matches abort startup before constructing the hardware stack.

Discovery searches private, wired IPv4 subnets with a /24 or smaller address
range, up to 1024 addresses total. It excludes this computer's own addresses.
The computer still needs an Ethernet address on the arm's subnet. Docker bridge
networking may hide that subnet; configure container LAN access (for example,
Linux host networking) or pass the known controller IP explicitly. Older firmware
without model information in its version reply also needs an explicit IP:

```bash
bash scripts/launch_mouse_hardware.sh 192.168.1.XXX
# Or:
ros2 launch hand_bringup hardware_mouse.launch.py robot_ip:=192.168.1.XXX
```

To test discovery alone without enabling motors:

```bash
python3 ros_ws/src/hand_bringup/hand_bringup/discover_xarm.py
```

Optional arguments are `max_speed`,
`mouse_gain`, and `show_rviz`. This launch is for an unprefixed, bare xArm 6;
it does not configure an attached gripper, tool geometry, payload, or real-world
obstacles. Configure those for your installation before operation. Servo checks
only the robot and objects present in its planning scene.

The hardware configuration and startup checks are tested offline. Physical-arm
motion, stopping distance, and communication-loss behavior have not been
validated on a connected robot.

## Controls

| Input | Motion relative to `link_base` |
| --- | --- |
| Hold left button and drag right / left | Negative / positive Y velocity |
| Hold left button and drag up / down | Positive / negative Z velocity |
| Hold left button + Shift and drag up / down | Positive / negative X velocity; Z is zero |
| Stop moving the mouse while holding the button | Zero velocity on the next update |
| Release button, leave pad, or switch focus | Zero velocity; click again to resume |
| Escape or close window | Send zero velocity and exit mouse control |

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
