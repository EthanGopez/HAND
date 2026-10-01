# Tutorial 2: `xarm_gazebo` — Putting the Arm in a Simulator

**You'll learn:** what Gazebo adds over fake mode, what starts when you launch it, how ros2_control runs *inside* the simulator, how simulation time works, how MoveIt drives the simulated arm, and the settings you'll use most.

**Prerequisites:** [Tutorial 1](01-moveit-config.md). You should be comfortable with controllers, `/joint_states` and the `follow_joint_trajectory` action.

---

> **HAND team notes (read first)**
> - Everything here runs inside the **HAND dev container**. View RViz and Gazebo at `http://localhost:6080/vnc.html?autoconnect=1&resize=scale`.
> - Every new terminal: `source /workspace/ros_ws/install/setup.bash`. If you run `ros2 launch` yourself (instead of a `scripts/launch_*.sh` script), also run `export DISPLAY=:1` first or windows won't appear in VNC.
> - **Never pass a `robot_ip` or run a `*_realmove*` launch file.** Real-arm sections are for understanding only; physical-arm operation needs the team's separate reviewed procedure.

## 1. What Gazebo gives you that fake mode doesn't

In fake mode the arm is a ghost: commands become positions instantly, nothing has mass, and there's no world around it.

**Gazebo** is a physics simulator. The arm has mass and inertia, its motors apply effort to reach commanded positions, things can collide, and you can add simulated sensors like a depth camera. It's the closest you get to the real arm without the real arm.

What stays the same: MoveIt, the controllers, the topic and action names. The only thing that changes is the ros2_control hardware plugin at the bottom of the stack (see Tutorial 0, section 2).

### Which Gazebo?

This package supports three simulators, picked with `gz_type`:

| `gz_type` | Simulator | Notes |
|---|---|---|
| `gazebo` (default) | Gazebo Classic (gazebo11) | The best-supported option; use this unless you have a reason not to |
| `gz` | Modern Gazebo (Fortress / Harmonic, "gz sim") | Needs `ros_gz_sim`, `ros_gz_bridge` and `gz_ros2_control` installed; downloads its world models from the internet |
| `ign` | Ignition (older name for modern Gazebo) | Needs the `ros_ign_*` packages |

Everything below assumes Gazebo Classic unless it says otherwise.

---

## 2. Try it: the arm alone in Gazebo

```bash
export DISPLAY=:1
ros2 launch xarm_gazebo xarm6_beside_table_gazebo.launch.py load_controller:=true
```

Gazebo opens with a table and an xArm 6 mounted on its edge.

`load_controller:=true` tells it to start the controllers too. Without it you get the arm model but nothing listening for commands, which is fine if you only want to look at it.

Check the controllers:

```bash
ros2 control list_controllers
```

```
joint_state_broadcaster  …  active
xarm6_traj_controller    …  active
```

Same names as in fake mode. Now move the arm with the same action you used in Tutorial 1:

```bash
ros2 action send_goal /xarm6_traj_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [{positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}}"
```

The simulated arm turns. Nothing about the command changed. Only the hardware underneath did.

### What's running

```bash
ros2 node list
```

You'll see roughly:

| Node | Job |
|---|---|
| `/gazebo` | The simulator server; the arm's physics runs here |
| `/controller_manager` | ros2_control, running *inside* the Gazebo process |
| `/robot_state_publisher` | TF for every link |

Plus short-lived `spawn_entity` and `spawner` nodes that exit once their job is done.

---

## 3. What happens during launch, in order

```
1. robot_state_publisher starts
      publishes the robot description (URDF) on the /robot_description topic
                    │
2. Gazebo starts with worlds/table.world  (ground, sun, a table)
                    │
3. spawn_entity.py reads /robot_description and drops the arm into the world
      as a model named UF_ROBOT, on the table edge (x=-0.2, y=-0.5, z=1.021, rotated 90°)
                    │
4. As the arm model loads, its URDF's <gazebo> plugin starts ros2_control inside Gazebo:
      controller_manager + the Gazebo hardware plugin, running at 1000 Hz
                    │
5. After spawning finishes: spawners load joint_state_broadcaster, xarm6_traj_controller
      (and the gripper controller if add_gripper:=true)
                    │
6. With the MoveIt launch (section 5), RViz opens here
```

Two points worth understanding:

**The URDF tells Gazebo to start ros2_control.** The robot description contains a `<gazebo>` block that loads the `gazebo_ros2_control` plugin, plus a `<ros2_control>` block that says which hardware plugin to use (`gazebo_ros2_control/GazeboSystem`) and which joints it controls. That's why there's no separate `ros2_control_node` in Gazebo mode: the controller manager lives inside the simulator.

**Controllers are loaded only after the arm exists.** The launch file waits for the spawn step to finish before it loads controllers, because there's nothing to control until the model is in the world.

---

## 4. Simulation time

In Gazebo, time doesn't have to match the clock on the wall. The simulator can run slower or faster than real time, and it pauses when you hit pause.

So Gazebo publishes its own clock on **`/clock`**, and every node in a Gazebo launch runs with the parameter **`use_sim_time: true`**. That makes each node read time from `/clock` instead of the computer's clock.

```bash
ros2 topic echo /clock --once
ros2 param get /robot_state_publisher use_sim_time
```

**Rule:** any node of your own that works with the simulated arm must also have `use_sim_time: true`. If it doesn't, its timestamps won't line up with `/joint_states`, and things like MoveIt's "is my robot state recent?" checks will fail.

With `gz_type:=gz`, the clock comes through a **`ros_gz_bridge`** node, which translates `/clock` (and camera topics) from Gazebo's own message system into ROS messages.

---

## 5. Gazebo + MoveIt together

This is the usual way to use the simulator:

```bash
bash /workspace/scripts/launch_gazebo_xarm.sh
# or, to pass options yourself:
export DISPLAY=:1
ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py [add_gripper:=true]
```

This single command:

1. Builds the full MoveIt configuration (same as fake mode, but pointed at the Gazebo hardware plugin and using `fake_controllers.yaml` so MoveIt talks to ros2_control controllers for both arm and gripper).
2. Starts everything in section 3 above, with `load_controller:=true`.
3. Starts `move_group` with `use_sim_time: true`.
4. Opens RViz with the MotionPlanning panel once the arm has spawned.

Now plan and execute in RViz exactly as in Tutorial 1. You'll see the arm move in both RViz *and* Gazebo, because RViz is showing the state that comes back from the simulator.

```
RViz ──/move_action──▶ move_group ──follow_joint_trajectory──▶ xarm6_traj_controller
                                                                  (inside Gazebo)
                                                                        │
Gazebo physics moves the joints ◀────── Gazebo hardware plugin ◀───────┘
        │
        └─▶ joint_state_broadcaster ─▶ /joint_states ─▶ robot_state_publisher ─▶ /tf ─▶ RViz
```

---

## 6. The gripper in Gazebo

The xArm gripper has one motor but six moving parts: one driven joint, `drive_joint`, and five finger/knuckle joints that follow it.

- ros2_control only knows about `drive_joint`. The gripper controller, `xarm_gripper_traj_controller`, moves only that joint.
- The five followers are handled by a small Gazebo plugin built by this package: **`libgazebo_mimic_joint_plugin.so`** (source in `src/mimic_joint_plugin.cpp`). Every simulation step, each instance reads `drive_joint`'s angle and pushes its own finger joint to match.

That's the only compiled code in `xarm_gazebo`. The mimic plugin is a Gazebo Classic plugin.

Open and close the gripper by hand:

```bash
# close (0.85 rad is fully closed on the xArm gripper; 0 is open)
ros2 action send_goal /xarm_gripper_traj_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [drive_joint],
    points: [{positions: [0.85], time_from_start: {sec: 1}}]}}"
```

---

## 7. The world

The world files are in `xarm_gazebo/worlds/`:

- `table.world`: Gazebo Classic. Ground plane, sun, and a table from Gazebo's built-in model library.
- `table_gz.world`: the same scene for modern Gazebo, loading its models from Gazebo Fuel (online).

Both run physics at 1 ms steps (1000 per simulated second) and **set gravity to zero** (`<gravity>0 0 0</gravity>`). With zero gravity the arm never sags and anything you drop in stays where you put it. If you want objects that fall, or you're studying how the arm copes with its own weight, copy the world file into your own package and change gravity back to `0 0 -9.81`.

To add objects, use Gazebo's GUI (the Insert tab, or the shapes on the toolbar), or write your own world file.

---

## 8. Useful launch arguments

These work on both `xarm6_beside_table_gazebo.launch.py` and `xarm6_moveit_gazebo.launch.py`:

| Argument | Effect |
|---|---|
| `add_gripper:=true` | Adds the gripper and its controller |
| `add_vacuum_gripper:=true` / `add_bio_gripper:=true` | Other end effectors |
| `add_realsense_d435i:=true` | Mounts a simulated RealSense D435i on the wrist. Its images, depth and point cloud appear as topics under `/camera/…` (`ros2 topic list \| grep camera`) |
| `add_other_geometry:=true geometry_type:=cylinder geometry_height:=0.075 geometry_radius:=0.045` | Attaches a simple custom tool shape to the end of the arm |
| `gz_type:=gz` | Use modern Gazebo instead of Classic (not installed in the HAND container) |
| `load_controller:=true` | Start the controllers (standalone launches only; the MoveIt launch already does this) |

Other arms: `lite6_beside_table_gazebo.launch.py`, `uf850_beside_table_gazebo.launch.py`, `xarm5_…`, `xarm7_…`, `xarm7_mirror_…`.

---

## 9. What's in the package

```
xarm_gazebo/
├── launch/
│   ├── xarm6_beside_table_gazebo.launch.py     ← what you run (one per arm)
│   ├── _robot_beside_table_gazebo.launch.py    ← does the work (section 3)
│   └── _dual_robot_beside_table_gazebo.launch.py   two arms (spawned as DUAL_UF_ROBOT)
├── worlds/
│   ├── table.world        Gazebo Classic
│   └── table_gz.world     modern Gazebo
└── src/mimic_joint_plugin.cpp   gripper finger-following plugin
```

The robot model itself isn't here. It comes from `xarm_description`, the same URDF that fake and real modes use, with the Gazebo plugin switched on.

---

## 10. Common questions

**The arm spawned but won't move.**
Check `ros2 control list_controllers`. In the standalone launch, did you pass `load_controller:=true`?

**My own node sees weird timestamps / MoveIt says the robot state is too old.**
Your node needs `use_sim_time: true`.

**Gazebo is slow.**
In the HAND container it renders in software (`LIBGL_ALWAYS_SOFTWARE=1`), so expect it to be slower than on a native install. Check the "real time factor" at the bottom of the Gazebo window. Below 1.0 means your machine can't keep up with 1 ms physics steps. Everything still works; it just runs in slow motion, and sim time keeps everything consistent.

**Can I grasp objects?**
You can close the gripper on an object, but grasping in simulation depends entirely on friction and contact settings, and it's often unreliable. For pick-and-place logic, many teams simulate the grasp by attaching the object in MoveIt's planning scene instead.

---

## Recap

- Gazebo swaps the fake hardware for a physics model; MoveIt, the controllers and the names stay the same.
- The URDF's `<gazebo>` block starts ros2_control **inside** the simulator.
- Controllers are loaded after the arm is spawned.
- Everything uses **sim time** from `/clock`, so your own nodes must too.
- The gripper's followers are moved by the mimic-joint plugin, not by ros2_control.
- The table worlds have gravity switched off.

**Next: [Tutorial 3 — xarm_planner](03-planner.md)**
