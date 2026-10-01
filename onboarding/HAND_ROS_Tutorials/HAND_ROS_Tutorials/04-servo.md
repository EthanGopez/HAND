# Tutorial 4: `xarm_moveit_servo` — Steering the Arm Live

**You'll learn:** the difference between planning and servoing, what runs when you start servo, how a joystick push becomes joint motion, how to drive it from the keyboard or your own code, and what the servo settings mean.

**Prerequisites:** [Tutorial 1](01-moveit-config.md). Tutorial 3 is useful for contrast.

---

> **HAND team notes (read first)**
> - Everything here runs inside the **HAND dev container**. View RViz and Gazebo at `http://localhost:6080/vnc.html?autoconnect=1&resize=scale`.
> - Every new terminal: `source /workspace/ros_ws/install/setup.bash`. If you run `ros2 launch` yourself (instead of a `scripts/launch_*.sh` script), also run `export DISPLAY=:1` first or windows won't appear in VNC.
> - **Never pass a `robot_ip` or run a `*_realmove*` launch file.** Real-arm sections are for understanding only; physical-arm operation needs the team's separate reviewed procedure.

> - **Gamepads don't work inside the HAND container** (USB devices aren't passed through). Use the **keyboard** controls in section 2, or the Python script in section 5.

## 1. Planning vs servoing

Everything so far has been **plan, then execute**: decide the whole motion up front, check it, then run it. That's right when you know where you're going.

Sometimes you don't. A person is steering with a joystick, or a camera is tracking a moving object and nudging the tool toward it many times a second. For that you want **servoing**: a continuous stream of small "move this way, this fast, right now" commands, each turned into joint motion immediately.

| | Planning (Tutorials 1 and 3) | Servoing (this tutorial) |
|---|---|---|
| Input | A goal: "go *there*" | A velocity: "move *this way*, this fast" |
| When it moves | After the whole path is computed | Immediately, continuously |
| Obstacle handling | Plans a path around them | Slows and stops as it gets close; doesn't route around |
| Typical use | Pick-and-place, known positions | Teleoperation, visual servoing, fine alignment |

`xarm_moveit_servo` sets up **MoveIt Servo** (part of MoveIt) for the xArm, and adds nodes that turn a joystick or keyboard into servo commands.

---

## 2. Try it

In the HAND container, start servo like this (the joystick setting doesn't matter, since there's no gamepad):

```bash
export DISPLAY=:1
# fake hardware, xArm 6
ros2 launch xarm_moveit_servo xarm_moveit_servo_fake.launch.py dof:=6 joystick_type:=1
```

`joystick_type`: `1` = wired Xbox 360, `2` = wireless Xbox 360, `3` = SpaceMouse Wireless.

RViz opens. Jump to **No joystick? Use the keyboard** below to drive it. (With a gamepad on a native Ubuntu install, pushing the left stick slides the arm's tip forward, back, left and right.)

### Controls (Xbox 360, native installs only)

| Input | Moves |
|---|---|
| Left stick | Tool along X and Y |
| LT / RT triggers | Tool along Z (down / up) |
| Right stick | Tool roll and pitch |
| LB / RB bumpers | Tool yaw |
| D-pad | Joint 1 and joint 2 directly |
| X / B | Last joint (e.g. joint 6) |
| Y / A | Second-to-last joint |
| Back / Start | Switch between moving relative to the **base** or the **tool** |

With a SpaceMouse, the six axes of the puck map to X/Y/Z and roll/pitch/yaw. Hold the left button to allow only X/Y/Z, the right button to allow only rotation.

### No joystick? Use the keyboard

Start the launch as above, then in a **second terminal**:

```bash
source /workspace/ros_ws/install/setup.bash
ros2 run xarm_moveit_servo xarm_keyboard_input
```

Keep that terminal focused and press keys:

| Key | Action |
|---|---|
| Arrow keys | Tool along X / Y |
| `;` / `.` | Tool up / down |
| `1` – `7` | Jog that joint |
| `R` | Reverse the jog direction for number keys |
| `W` / `E` | Commands relative to the base (**W**orld) / the **E**nd effector |
| `Q` | Quit |

Each keypress sends one command. Holding a key down (key repeat) keeps the arm moving; releasing it stops the arm within a fraction of a second.

---

## 3. What's running

```bash
ros2 node list
```

| Node | Job |
|---|---|
| `/servo_server` | MoveIt Servo: turns velocity commands into joint positions, checks collisions and singularities |
| `/joy_node` | Reads the gamepad and publishes `/joy` |
| `/joy_to_twist_publisher` | Converts `/joy` into servo commands (this package's code) |
| `/robot_state_publisher` | TF from `/joint_states` |
| `/static_tf2_broadcaster` | Fixed `world` → `link_base` frame |
| `/controller_manager` | ros2_control with fake or real hardware |
| `/rviz2` | Visualisation |

Most of these run inside one process, a **component container** called `xarm_moveit_servo_container`. Components are nodes loaded as plugins into a shared process so they can pass messages without copying them, which lowers latency.

**Notice what's missing: there's no `move_group`.** Servo doesn't use the planner. It loads the robot model and planning scene itself, and it talks to the controller directly.

---

## 4. From stick push to joint motion

```
 gamepad
    │
 joy_node ──/joy (sensor_msgs/Joy)──▶ joy_to_twist_publisher
                                          │  maps sticks/buttons to either:
                                          ├─ TwistStamped ─▶ /servo_server/delta_twist_cmds
                                          └─ JointJog ─────▶ /servo_server/delta_joint_cmds
                                                                  │
                                                          servo_server
                                          (reads /joint_states and the planning scene)
                                                                  │
                                    JointTrajectory, about 15 times a second
                                                                  ▼
                                          /xarm6_traj_controller/joint_trajectory
                                                                  │
                                                     xarm6_traj_controller
                                                                  │
                                                      hardware (fake or real)
```

Step by step:

1. **`joy_node`** reads the gamepad and publishes its raw axes and buttons on `/joy`.
2. **`joy_to_twist_publisher`** decides what the input means:
   - Sticks, triggers and bumpers become a **twist**: a linear velocity (x, y, z) and angular velocity (roll, pitch, yaw) for the tool. It's stamped with a frame, `link_base` or `link_eef`, which tells servo whether "forward" means the base's forward or the tool's forward.
   - D-pad and face buttons become a **joint jog**: "move joint N at this speed."
   Small stick movements are ignored (a dead zone), so a resting stick doesn't creep.
3. **`servo_server`**, every 0.067 s (about 15 Hz):
   - Takes the latest command.
   - For a twist, uses the arm's **Jacobian** (the maths linking joint speeds to tool speed) to work out the joint velocities that produce that tool motion.
   - Scales down near **singularities** (poses where the arm loses the ability to move in some direction, like when fully stretched out).
   - Scales down or stops near **collisions** with itself or objects in the planning scene.
   - Stops joints before they hit their **limits**.
   - Turns the resulting velocities into the next joint positions and publishes them as a short `JointTrajectory`.
4. **The controller** receives that trajectory on its **topic** (`/xarm6_traj_controller/joint_trajectory`), not its action, and moves the joints.

Note the difference from planning: servo uses the controller's plain topic interface, sending a fresh short trajectory every cycle. Each new message replaces the previous one.

### The dead-man behaviour

Servo only keeps moving while commands keep arriving. If no command arrives for **0.2 seconds**, it stops and sends a few "hold still" messages. That's why releasing the stick or key stops the arm. Anything that drives servo must **stream** commands continuously, not send one and wait.

---

## 5. Driving servo from your own code

Because servo just listens on a topic, anything that publishes `TwistStamped` can drive it: a vision tracker, a force-feedback loop, a script. This Python node moves the tool slowly upward for as long as it runs:

```python
#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import TwistStamped


def main():
    rclpy.init()
    node = rclpy.create_node('twist_streamer')
    pub = node.create_publisher(TwistStamped, '/servo_server/delta_twist_cmds', 10)

    def tick():
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()   # servo ignores stale commands
        msg.header.frame_id = 'link_base'                     # directions relative to the base
        msg.twist.linear.z = 0.02                             # 2 cm/s upward
        pub.publish(msg)

    node.create_timer(0.05, tick)   # 20 Hz: faster than servo's 15 Hz and well inside the 0.2 s timeout
    rclpy.spin(node)


if __name__ == '__main__':
    main()
```

Save it as `twist_streamer.py`, then run `python3 twist_streamer.py` in a sourced terminal with the servo launch up. Ctrl-C stops the script, and servo stops the arm 0.2 s later.

Things to notice:

- **Units:** this setup uses `command_in_type: speed_units`, so numbers are **metres per second** and **radians per second**. `0.02` is 2 cm/s.
- **The timestamp matters.** Servo uses it to decide whether a command is fresh. Always stamp with the current time.
- **The frame decides the directions.** `link_base`: z is up. `link_eef`: z is along the tool axis.

To jog joints instead, publish `control_msgs/msg/JointJog` to `/servo_server/delta_joint_cmds` with `joint_names: ['joint1']` and `velocities: [0.1]`.

### Starting and stopping servo

Servo has service switches:

```bash
ros2 service call /servo_server/start_servo std_srvs/srv/Trigger
ros2 service call /servo_server/stop_servo std_srvs/srv/Trigger
ros2 service call /servo_server/pause_servo std_srvs/srv/Trigger
ros2 service call /servo_server/unpause_servo std_srvs/srv/Trigger
```

The joystick and keyboard nodes call `start_servo` automatically when they start.

Watch servo's status (normal, near singularity, near collision, joint bound, and so on) on:

```bash
ros2 topic echo /servo_server/status
```

---

## 6. The settings file

Servo is configured by `config/xarm_moveit_servo_config.yaml`. The launch file overwrites two values to match the arm you launched: the planning group (`move_group_name`, e.g. `xarm6`) and the output topic (`command_out_topic`, e.g. `/xarm6_traj_controller/joint_trajectory`).

| Setting | Value | Meaning |
|---|---|---|
| `command_in_type` | `speed_units` | Inputs are m/s and rad/s. (The alternative, `unitless`, treats inputs as −1…1 and multiplies by the `scale` values.) |
| `scale.linear` / `rotational` / `joint` | 0.1 / 0.1 / 0.01 | Only used with `unitless` input |
| `publish_period` | 0.067 s | How often servo sends a new command (~15 Hz) |
| `command_out_type` | `trajectory_msgs/JointTrajectory` | What servo publishes to the controller |
| `publish_joint_positions` | true | Sends positions (not velocities) to the controller |
| `planning_frame` | `link_base` | The frame servo does its maths in |
| `ee_frame_name` | `link_eef` | The link treated as the tool |
| `robot_link_command_frame` | `link_base` | Default frame for incoming commands |
| `incoming_command_timeout` | 0.2 s | Stop if no command for this long |
| `num_outgoing_halt_msgs_to_publish` | 4 | "Hold still" messages sent when stopping |
| `lower_singularity_threshold` | 17 | Start slowing near a singularity |
| `hard_stop_singularity_threshold` | 30 | Stop at this point |
| `joint_limit_margin` | 0.1 rad | Stop this far before a joint limit |
| `check_collisions` | true | Check for collisions while moving |
| `collision_check_rate` | 10 Hz | How often |
| `self_collision_proximity_threshold` | 0.01 m | Slow down this close to hitting itself |
| `scene_collision_proximity_threshold` | 0.02 m | Slow down this close to scene objects |

(The singularity thresholds are condition numbers of the Jacobian, a measure of how badly the arm's motion is distorted. Higher means closer to a singularity.)

---

## 7. Real arm (reference only — HAND does not run this without a reviewed procedure)

```bash
ros2 launch xarm_moveit_servo xarm_moveit_servo_realmove.launch.py robot_ip:=192.168.1.xxx dof:=6 joystick_type:=1
```

The whole chain is the same; the hardware plugin talks to the arm instead of faking it (Tutorial 1, section 4).

**Before you jog a real arm:**

- Know the speeds. With `speed_units` input, a full stick push asks for **1 m/s** of tool speed (or 1 rad/s rotation), and each keyboard arrow press asks for **0.5 m/s**. Servo's safety features slow things near limits and singularities, not in open space.
- Try everything in fake mode first, and keep the emergency stop in your hand.
- Start with small stick movements.

The 7-axis joystick setup also jogs joint 1 very slightly for a moment at start-up, to wake servo. Expect that small motion.

Other arms: `lite6_moveit_servo_fake.launch.py` / `…_realmove.launch.py`, `uf850_moveit_servo_fake.launch.py` / `…_realmove.launch.py`. For xArm 5 and 7 use the `xarm_…` files with `dof:=5` or `dof:=7`.

---

## 8. What's in the package

```
xarm_moveit_servo/
├── config/xarm_moveit_servo_config.yaml     servo settings (section 6)
├── launch/
│   ├── xarm_moveit_servo_{fake,realmove}.launch.py    what you run (+ lite6_, uf850_)
│   └── _robot_moveit_servo.launch.py                  does the work
├── src/
│   ├── xarm_joystick_input.cpp    the /joy → servo command component
│   └── xarm_keyboard_input.cpp    the keyboard executable
└── rviz/servo.rviz
```

The launch file builds the same robot configuration as Tutorial 1, starts ros2_control, spawns the controllers, and once the arm controller is up, starts the container with servo and the input nodes.

---

## 9. Common questions

**Nothing moves when I push the stick.**
In the HAND container this is expected: there's no gamepad passthrough. Use the keyboard. On a native install, check `ros2 topic echo /joy`: if nothing appears, the gamepad isn't detected. If `/joy` works, check that `joystick_type` matches your controller: the node ignores messages with the wrong number of axes and buttons.

**It moves, then stops near a certain pose.**
Look at `/servo_server/status`. You're probably near a singularity, a joint limit or a collision.

**Can I use servo and the planner at the same time?**
They'd both be commanding the same controller. Pick one at a time: servo for live control, the planner for goal-to-goal moves.

**Why does the arm stop when my script pauses?**
The 0.2 s command timeout. Keep publishing, even zeros, while you want servo to stay "live."

---

## Recap

- Servoing = continuous velocity commands, executed immediately; planning = compute a whole path first.
- Input nodes publish `TwistStamped` (tool velocity) or `JointJog` (joint velocity) to `servo_server`.
- `servo_server` converts them to joint positions ~15 times a second, with singularity, collision and joint-limit checks, and publishes to the controller's **topic**.
- No `move_group` is involved.
- Commands must stream; 0.2 s of silence stops the arm.
- Inputs are in m/s and rad/s here, so be deliberate on the real arm.

**Back to: [Tutorial 0 — Start Here](00-start-here.md)**
