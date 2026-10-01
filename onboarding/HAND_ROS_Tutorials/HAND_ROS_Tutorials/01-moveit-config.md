# Tutorial 1: `xarm_moveit_config` — MoveIt, the Arm's Planning Brain

**You'll learn:** what MoveIt does for the xArm, what starts when you launch it, how a click in RViz becomes motor motion, which config files control what, and how the same setup runs on fake hardware, in Gazebo and on the real arm.

**Prerequisites:** [Tutorial 0](00-start-here.md), a built and sourced workspace.

> **HAND team notes (read first)**
> - Everything here runs inside the **HAND dev container**. View RViz and Gazebo at `http://localhost:6080/vnc.html?autoconnect=1&resize=scale`.
> - Every new terminal: `source /workspace/ros_ws/install/setup.bash`. If you run `ros2 launch` yourself (instead of a `scripts/launch_*.sh` script), also run `export DISPLAY=:1` first or windows won't appear in VNC.
> - **Never pass a `robot_ip` or run a `*_realmove*` launch file.** Real-arm sections are for understanding only; physical-arm operation needs the team's separate reviewed procedure.

---

## 1. What this package is

`xarm_moveit_config` is a **MoveIt configuration package**. It has almost no code of its own. It's a bundle of settings and launch files that tells MoveIt everything it needs about the UFACTORY arms:

- which joints make up the arm and the gripper (planning groups)
- named poses you can ask for by name
- joint speed and acceleration limits
- which inverse-kinematics solver to use
- which motion-planning algorithms are available
- which controllers to send finished trajectories to

Every other package in this series depends on it. `xarm_planner` and `xarm_moveit_servo` both reuse its settings, and its Gazebo launch files are how you drive the simulator with MoveIt.

---

## 2. Try it first: fake mode

Fake mode needs no robot and no simulator. The "hardware" instantly reaches every commanded position, so you can focus on how the pieces connect.

```bash
export DISPLAY=:1
ros2 launch xarm_moveit_config xarm6_moveit_fake.launch.py
```

**HAND note:** use this command, not `scripts/launch_fake_xarm.sh`. That script launches the planner version (Tutorial 3), whose RViz is view-only and has no planning panel.

RViz opens with an xArm 6 standing at the origin. Let's move it.

### Plan and execute in RViz

1. On the left, find the **MotionPlanning** panel and open the **Planning** tab.
2. Check that **Planning Group** says `xarm6`.
3. Under **Goal State**, choose `hold-up` from the dropdown. A ghost (orange) arm appears in that pose.
4. Click **Plan**. An animated preview of the path plays.
5. Click **Execute**. The arm follows the path.
6. Now drag the arm's tip by its **interactive marker** (the coloured arrows and rings on the end of the arm) to a new spot, then click **Plan & Execute**.
7. Set the Goal State back to `home` and **Plan & Execute** again.

You just used the full stack. The rest of this tutorial explains what happened underneath.

### Look at what's running

Open a second terminal (remember `source /workspace/ros_ws/install/setup.bash`):

```bash
ros2 node list
```

You'll see roughly:

| Node | Job |
|---|---|
| `/move_group` | MoveIt: plans paths and oversees their execution |
| `/rviz2` | The 3D window and planning panel |
| `/robot_state_publisher` | Publishes TF for every link, from `/joint_states` + URDF |
| `/static_transform_publisher` | Publishes the fixed `world` → `link_base` frame |
| `/controller_manager` | ros2_control: runs controllers and the fake hardware |

The `spawner` nodes you may see briefly load controllers and then exit.

Now look at the controllers:

```bash
ros2 control list_controllers
```

```
joint_state_broadcaster  joint_state_broadcaster/JointStateBroadcaster  active
xarm6_traj_controller    joint_trajectory_controller/JointTrajectoryController  active
```

And watch the joint angles change as you execute moves in RViz:

```bash
ros2 topic echo /joint_states
```

---

## 3. What happened when you clicked "Plan & Execute"

```
 RViz MotionPlanning panel
     │  goal: "xarm6 group to this pose"          (action /move_action)
     ▼
 move_group
     │  1. IK: pose → joint angles                (KDL solver, kinematics.yaml)
     │  2. Plan a collision-free path             (OMPL, RRTConnect by default)
     │  3. Add timing within speed limits         (joint_limits.yaml)
     │  4. Hand trajectory to the controller      (action /xarm6_traj_controller/follow_joint_trajectory)
     ▼
 xarm6_traj_controller  (inside controller_manager, 150 Hz loop)
     │  interpolates the trajectory, sends a position for every joint every cycle
     ▼
 hardware interface  (fake: just copies command → state)
     │
     ▼
 joint_state_broadcaster → /joint_states → robot_state_publisher → /tf → RViz redraws
                                         └──────────────────────→ move_group (knows where the arm is)
```

Step by step:

1. **RViz sends a goal.** The MotionPlanning panel is a client of `move_group`'s `/move_action` action. The goal says which group (`xarm6`), where the start is (the current state), and where the goal is.
2. **move_group does inverse kinematics.** If you dragged the marker, you gave a *pose* (position + orientation of the tip). The KDL solver works out joint angles that put the tip there.
3. **move_group plans.** OMPL, a library of sampling-based planners, searches for a path from the current joint angles to the goal that doesn't hit the arm itself or anything in the planning scene. Default algorithm: RRTConnect.
4. **move_group times the path.** A raw path is just a list of poses. MoveIt adds timestamps so no joint exceeds its velocity/acceleration limits. This package's RViz layout starts with velocity and acceleration scaling at 10% of those limits; the scaling sliders in the Planning tab change that.
5. **move_group executes.** It sends the timed trajectory to the controller named in its controller config, using the `FollowJointTrajectory` action, and monitors it until the controller reports success.
6. **The controller drives the joints.** `xarm6_traj_controller` runs every cycle (150 times a second here), figures out where each joint should be *right now* along the trajectory, and writes that to the hardware.
7. **The state flows back.** `joint_state_broadcaster` publishes `/joint_states`, `robot_state_publisher` turns it into TF, RViz redraws, and `move_group` updates its idea of where the arm is.

### Do it without RViz

To prove the controller is independent of MoveIt, send it a trajectory yourself. This moves joint 1 to 0.5 rad over 3 seconds:

```bash
ros2 action send_goal /xarm6_traj_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [{positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}}"
```

The arm in RViz turns. This is exactly the message `move_group` sends, just with one point instead of hundreds. **Only do this in fake mode.** There's no collision checking at this level.

---

## 4. The three launch modes

The same package launches three ways. Only the bottom layer changes.

### Fake

```bash
ros2 launch xarm_moveit_config xarm6_moveit_fake.launch.py [add_gripper:=true]
```

Hardware is `UFRobotFakeSystemHardware`. Everything else is as described above.

### Real arm (reference only — HAND does not run this without a reviewed procedure)

```bash
ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py robot_ip:=192.168.1.xxx [add_gripper:=true]
```

What's different:

- The hardware plugin is `UFRobotSystemHardware`. When it starts, it connects to the arm over the network, clears errors, enables the motors, and switches the arm into **servo mode** (UFACTORY's mode 1, where the controller accepts a continuous stream of joint targets). It switches back to mode 0 when ROS shuts down. While ROS is running, the teach pendant and UFACTORY Studio can't move the arm.
- The UFACTORY driver (the `xarm_api` code) runs **inside** that hardware plugin. You don't start a separate driver.
- The driver publishes the arm's state on **`/xarm/joint_states`**, and a `joint_state_publisher` node relays it to `/joint_states`. There's no `joint_state_broadcaster` in this mode.
- The gripper is **not** a ros2_control controller on the real arm. MoveIt talks to it through a **`/xarm_gripper/gripper_action`** action (type `GripperCommand`), which the driver serves directly.
- You also get the driver's extras under `/xarm/`, such as `/xarm/robot_states` (arm state, mode, error codes, TCP pose) and a set of services like `/xarm/clean_error`. Most driver services are switched off by default; they're enabled in `xarm_api/config/xarm_user_params.yaml`.

**Safety:** keep the emergency stop in reach. Plan in RViz, watch the preview, and only then execute. Start with the velocity scaling low.

### Gazebo

```bash
bash /workspace/scripts/launch_gazebo_xarm.sh
# which runs: ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py
```

This launches Gazebo (Tutorial 2) *and* MoveIt together. The controllers run inside the simulator and every node uses the simulator's clock.

### Side-by-side summary

| | Fake | Real | Gazebo |
|---|---|---|---|
| Hardware plugin | Fake | xArm over network | Gazebo |
| Where `controller_manager` runs | `ros2_control_node` | `ros2_control_node` | inside Gazebo |
| `/joint_states` comes from | `joint_state_broadcaster` | driver → `joint_state_publisher` | `joint_state_broadcaster` |
| Arm controlled by | `xarm6_traj_controller` | `xarm6_traj_controller` | `xarm6_traj_controller` |
| Gripper controlled by | `xarm_gripper_traj_controller` | `/xarm_gripper/gripper_action` (driver) | `xarm_gripper_traj_controller` |
| Clock | wall clock | wall clock | `/clock` from Gazebo |

---

## 5. What's in the package

```
xarm_moveit_config/
├── config/
│   ├── xarm6/                  one folder per arm model
│   │   ├── kinematics.yaml       IK solver settings
│   │   ├── joint_limits.yaml     max velocity / acceleration per joint
│   │   ├── ompl_planning.yaml    planners available for this group
│   │   ├── controllers.yaml      where MoveIt sends trajectories — real arm
│   │   └── fake_controllers.yaml where MoveIt sends trajectories — fake & Gazebo
│   ├── xarm_gripper/           one folder per gripper, same file types
│   ├── bio_gripper/ …
│   └── moveit_configs/         defaults shared by every arm (OMPL plugin, planner list)
├── srdf/                       planning groups, named poses, collision rules (xacro)
├── launch/                     *_fake / *_realmove / *_gazebo + dual_* versions
└── rviz/                       moveit.rviz (planning panel), planner.rviz (view only)
```

When you add a gripper, its folder is **merged on top of** the arm's folder. That's how `add_gripper:=true` adds a gripper group, its limits and its controller without anyone writing a new config.

### The files one at a time

**`kinematics.yaml`**: how MoveIt solves "what joint angles put the tip here?"

```yaml
xarm6:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_timeout: 0.005   # seconds per attempt
  kinematics_solver_attempts: 3
```

**`joint_limits.yaml`**: MoveIt respects these when it times a trajectory. Lower `max_velocity` to make every planned move slower.

```yaml
joint_limits:
  joint1:
    has_velocity_limits: true
    max_velocity: 2.14        # rad/s
    has_acceleration_limits: true
    max_acceleration: 10.0    # rad/s²
```

**`ompl_planning.yaml`**: which OMPL algorithms the group can use. `RRTConnect` is the default; you can pick another in RViz's "Context" tab.

**`controllers.yaml` / `fake_controllers.yaml`**: tells MoveIt which action to send trajectories to for each group:

```yaml
controller_names:
  - xarm6_traj_controller
xarm6_traj_controller:
  action_ns: follow_joint_trajectory   # → /xarm6_traj_controller/follow_joint_trajectory
  type: FollowJointTrajectory
  default: true
  joints: [joint1, joint2, joint3, joint4, joint5, joint6]
```

The gripper's real-arm version uses `type: GripperCommand` and `action_ns: gripper_action` instead. That one difference is why the gripper uses a different action on real hardware.

**The SRDF** (`srdf/_xarm6_macro.srdf.xacro`) defines:

- **Planning groups:** `xarm6` (joints 1–6, ending at `link_eef`, or at `link_tcp` when a tool is attached) and, with a gripper, `xarm_gripper` (driven by `drive_joint`).
- **Named states:** `home` (all zeros) and `hold-up` for the arm; `open` (0) and `close` (0.85) for the gripper. These are what show up in RViz's Goal State dropdown, and your code can ask for them by name.
- **Disabled collisions:** link pairs that can never touch (neighbours, for example), so MoveIt skips checking them.

### Where the controller side is configured

MoveIt's `controllers.yaml` only says *where to send* trajectories. The controllers themselves are defined in the neighbouring package **`xarm_controller/config/xarm6_controllers.yaml`**: the update rate (150 Hz), joint names, and how closely the controller must follow (the `constraints` block).

---

## 6. How the launch files are organised

You'll never need to edit these to use the arm, but it helps to know the shape:

```
xarm6_moveit_fake.launch.py            ← what you run; just sets dof=6, robot_type=xarm
  └─ _robot_moveit_fake.launch.py      ← does the real work (files starting with _ are internal)
       ├─ builds URDF, SRDF and all the config above into one set of parameters
       ├─ starts robot_state_publisher and ros2_control_node (fake hardware)
       ├─ spawns joint_state_broadcaster, xarm6_traj_controller (+ gripper controller)
       └─ _robot_moveit_common2.launch.py   ← shared by fake, real and Gazebo
            ├─ move_group
            ├─ rviz2
            └─ static_transform_publisher (world → link_base)
```

The config-building step is done by a helper class in `uf_ros_lib` called `MoveItConfigsBuilder`. It gathers the URDF, SRDF and YAML files into one dictionary of parameters, which is then handed to `move_group`, RViz and anything else that needs it.

---

## 7. Useful launch arguments

| Argument | Example | Effect |
|---|---|---|
| `add_gripper` | `add_gripper:=true` | Adds the gripper model, group, limits and controller |
| `add_vacuum_gripper` | `add_vacuum_gripper:=true` | Adds the vacuum gripper model (no extra group) |
| `add_bio_gripper` | `add_bio_gripper:=true` | Adds the BIO gripper (not on Lite 6) |
| `add_other_geometry` | `add_other_geometry:=true geometry_type:=box geometry_length:=0.1 …` | Adds a custom tool shape so MoveIt accounts for it |
| `attach_to`, `attach_xyz`, `attach_rpy` | `attach_xyz:='"0 0 0.5"'` | Mounts the arm somewhere other than the world origin |
| `no_gui_ctrl` | `no_gui_ctrl:=true` | Read-only RViz and starts the planner node (Tutorial 3) |
| `kinematics_suffix` | `kinematics_suffix:=AAA` | Uses per-robot calibrated kinematics (real arms built after Aug 2023) |

Note the double quotes inside single quotes for `attach_xyz`: the value needs to keep its quote marks.

---

## 8. Two arms

The `dual_*` launch files run **one** `move_group` and **one** controller manager for two arms. Every name gets a prefix, `L_` or `R_` by default: groups `L_xarm6` and `R_xarm6`, controllers `L_xarm6_traj_controller` and `R_xarm6_traj_controller`, frames `L_link_base` and `R_link_base`.

```bash
ros2 launch xarm_moveit_config dual_xarm6_moveit_fake.launch.py
```

In RViz, pick `L_xarm6` or `R_xarm6` as the planning group to move each arm.

---

## 9. Common questions

**"Plan" works but "Execute" fails.**
The controller rejected the trajectory or isn't running. Check `ros2 control list_controllers`: the arm controller must be `active`. On the real arm, check `/xarm/robot_states` for an error code.

**Why doesn't RViz show my gripper group?**
You need `add_gripper:=true` on the launch command. Without it the gripper doesn't exist in the URDF or SRDF.

**Why do two nodes publish the `world` → `link_base` frame?**
The URDF already contains a fixed `world` joint (so `robot_state_publisher` publishes it) and the launch also runs a static publisher. They agree, so it's harmless.

**Where do I change how fast moves are?**
For everything: `config/xarm6/joint_limits.yaml`. For one move: the velocity scaling slider in RViz, or the scaling factor in your code.

---

## Recap

- `move_group` plans; the controller executes; the hardware plugin decides whether the motion is fake, simulated or real.
- MoveIt hands trajectories to `/xarm6_traj_controller/follow_joint_trajectory`.
- `/joint_states` → `robot_state_publisher` → TF closes the loop.
- The config folders are small YAML files; the gripper's folder is merged on top of the arm's.
- On the real arm, the gripper goes through the driver's `GripperCommand` action instead of a controller.

**Next: [Tutorial 2 — xarm_gazebo](02-gazebo.md)**
