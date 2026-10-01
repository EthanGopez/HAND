# Tutorial 0: Start Here — How the xArm ROS 2 Stack Fits Together

This series explains the four packages from `xarm_ros2` (humble branch) that our work is built on:

| Tutorial | Package | One-line summary |
|---|---|---|
| **1** | `xarm_moveit_config` | The "brain": MoveIt setup that plans motions and sends them to the arm |
| **2** | `xarm_gazebo` | A physics simulator the arm can live in instead of the real world |
| **3** | `xarm_planner` | A simple service/API layer so your code can say "go here" without writing MoveIt code |
| **4** | `xarm_moveit_servo` | Real-time jogging: steer the arm live with a joystick, keyboard or your own code |

> **HAND team notes (read first)**
> - Everything here runs inside the **HAND dev container**. View RViz and Gazebo at `http://localhost:6080/vnc.html?autoconnect=1&resize=scale`.
> - Every new terminal: `source /workspace/ros_ws/install/setup.bash`. If you run `ros2 launch` yourself (instead of a `scripts/launch_*.sh` script), also run `export DISPLAY=:1` first or windows won't appear in VNC.
> - **Never pass a `robot_ip` or run a `*_realmove*` launch file.** Real-arm sections are for understanding only; physical-arm operation needs the team's separate reviewed procedure.

Read this page first. It explains the ROS vocabulary you need, in plain English, using the xArm as the example every time. Then do the tutorials in order: each one builds on the one before.

---

## 1. ROS 2 in plain English

ROS 2 is not an operating system. It's a set of conventions and libraries that let many small programs work together on one robot. Everything below is one of a handful of building blocks.

### Nodes: the programs

A **node** is one running program with one job. On our arm there's a node that plans motions (`move_group`), a node that draws the 3D view (`rviz2`), a node that calculates where every part of the arm is in space (`robot_state_publisher`), and so on. A normal session has 5 to 15 of them running at once.

You rarely start nodes one by one. A **launch file** starts a whole group of them with the right settings. When you type `ros2 launch xarm_moveit_config xarm6_moveit_fake.launch.py`, that one file starts about eight nodes.

### Topics: broadcasts

A **topic** is a named channel where one node shouts messages and any number of nodes listen. Nobody replies. It's for continuous streams of data.

The most important topic on our arm is **`/joint_states`**. Many times a second, something publishes the current angle of every joint. Anything that needs to know where the arm is listens to it.

### Services: questions with one answer

A **service** is a request/response call, like a function call between programs. You send a request, you wait, you get one reply.

Example from Tutorial 3: you call `/xarm_joint_plan` with six joint angles, and it replies `success: true` or `false`.

### Actions: long jobs with progress

An **action** is for jobs that take a while and might need to be cancelled, like "move the arm along this path." You send a **goal**, you get **feedback** while it runs, and you get a **result** at the end.

The arm's motion is almost always driven through the action **`/xarm6_traj_controller/follow_joint_trajectory`**: "here is a list of joint positions with timestamps, please follow it."

### Parameters: settings

**Parameters** are named settings attached to a node, like `use_sim_time: true` or a robot's joint speed limits. Launch files set them when they start the node.

### TF: where everything is

**TF** ("transforms") is ROS's system for tracking coordinate frames. Each part of the arm has a frame: `world`, `link_base`, `link1` … `link6`, `link_eef` (end of the arm), `link_tcp` (tool center point, the tip of the gripper). TF answers questions like "where is the gripper tip relative to the base right now?"

`robot_state_publisher` produces TF by combining the arm's shape (the URDF, below) with the live `/joint_states`.

### URDF and SRDF: the robot's description

- **URDF** (Unified Robot Description Format) is an XML file describing the arm physically: every link, every joint, joint limits, 3D meshes, masses. It lives in `xarm_description`. Every package in this series reads it.
- **SRDF** (Semantic Robot Description Format) is MoveIt's extra layer on top: which joints form the "arm" group, which form the "gripper" group, named poses like `home`, and which link pairs can never collide so MoveIt doesn't waste time checking them. It lives in `xarm_moveit_config/srdf/`.

Both are written as **xacro** files: XML with variables and macros, so one file can describe an xArm5, 6 or 7, with or without a gripper.

---

## 2. ros2_control: how commands actually reach the motors

This is the piece that confuses people most, and it's what makes simulation and real hardware interchangeable.

```
   "Follow this trajectory"
            │
            ▼
 ┌───────────────────────┐
 │ Controller            │   e.g. xarm6_traj_controller
 │ (JointTrajectory      │   Works out where each joint should be
 │  Controller)          │   at each instant, 150–1000 times a second
 └──────────┬────────────┘
            │ position commands
            ▼
 ┌───────────────────────┐
 │ Hardware interface    │   A plugin. Swappable!
 │ (plugin)              │   • Fake: pretends, instantly "reaches" every command
 └──────────┬────────────┘   • Real: talks to the xArm over the network
            │                • Gazebo: moves the simulated arm
            ▼
     motors (or pretend motors)
```

- The **controller manager** (`/controller_manager`) is a node that runs a fast loop: read joint states from the hardware, let each controller compute, write commands to the hardware.
- **Controllers** are plugins loaded into it. We use two kinds:
  - **`joint_state_broadcaster`** reads joint positions and publishes them on `/joint_states`.
  - **`JointTrajectoryController`** (JTC) accepts trajectories and follows them. Ours are named after the arm: `xarm6_traj_controller`, `xarm_gripper_traj_controller`, etc.
- The **hardware interface** is the bottom layer, chosen by a single setting:

| Mode | Hardware plugin | What happens |
|---|---|---|
| **fake** | `uf_robot_hardware/UFRobotFakeSystemHardware` | No robot. Commands are echoed back as the new position. Perfect for learning. |
| **real** ("realmove") | `uf_robot_hardware/UFRobotSystemHardware` | Connects to the arm at `robot_ip` and streams joint commands to it. |
| **Gazebo** | `gazebo_ros2_control/GazeboSystem` (and gz variants) | The simulator moves a physics model of the arm. |

**Key idea:** the controllers, MoveIt, the planner and servo don't know or care which one is underneath. That's why you can develop against fake or Gazebo and switch to the real arm by changing one launch file.

---

## 3. MoveIt in one paragraph

**MoveIt** is the motion-planning framework. Its main node, **`move_group`**, knows the robot's shape (URDF), its groups and collision rules (SRDF), how to do inverse kinematics (turn "put the tip here" into joint angles), and how to search for a collision-free path. When it has a plan, it hands the trajectory to the controller through the `follow_joint_trajectory` action. Tutorial 1 covers it in depth.

---

## 4. The three modes, side by side

Every package in this series can run in these modes. The launch file name tells you which:

| File name contains | Mode | Needs a real arm? | Physics? |
|---|---|---|---|
| `_fake` | Fake hardware | No | No, joints just jump to commanded positions |
| `_gazebo` | Gazebo simulation | No | Yes |
| `_realmove` | Real arm | Yes, pass `robot_ip:=...` | Real world (**not used by HAND without a reviewed procedure**) |

**Always try new code in fake first, then Gazebo, then the real arm.**

---

## 5. Naming: one pattern explains most names

Almost every name is built from the arm model:

| Arm | Model name | Arm controller | Planning group |
|---|---|---|---|
| xArm 5 / 6 / 7 | `xarm5` / `xarm6` / `xarm7` | `xarm6_traj_controller` | `xarm6` |
| Lite 6 | `lite6` | `lite6_traj_controller` | `lite6` |
| UFACTORY 850 | `uf850` | `uf850_traj_controller` | `uf850` |
| xArm 7 mirror | `xarm7_mirror` | `xarm7_mirror_traj_controller` | `xarm7_mirror` |

Grippers: `xarm_gripper`, `uf850_gripper`, `bio_gripper`, `lite_gripper`.

Launch files use the same vocabulary for their arguments:

| Argument | Meaning |
|---|---|
| `dof` | Number of joints (5, 6, 7) |
| `robot_type` | `xarm`, `lite`, `uf850`, `xarm7_mirror` |
| `add_gripper:=true` | Attach the xArm/Lite/850 gripper |
| `add_bio_gripper:=true`, `add_vacuum_gripper:=true` | Other end effectors |
| `prefix:=L_` | Put `L_` in front of every name, used for two-arm setups |
| `robot_ip:=192.168.1.xxx` | Real arm's address (realmove only) |

**The rest of this series uses the xArm 6** (`xarm6`). For another arm, swap the name in the launch file (e.g. `lite6_moveit_fake.launch.py`) and in topic/controller names.

---

## 6. Your inspection toolkit

Open a second container terminal (VS Code: **Terminal → New Terminal**) and `source /workspace/ros_ws/install/setup.bash`. These commands work with anything running. Keep them handy; each tutorial asks you to use them so you can *see* the stack instead of trusting a diagram.

```bash
ros2 node list                         # every running program
ros2 topic list                        # every broadcast channel
ros2 topic echo /joint_states --once   # print one message
ros2 topic hz /joint_states            # how often it's published
ros2 service list                      # every callable service
ros2 action list                       # every action server
ros2 control list_controllers          # controllers and whether they're active
ros2 run tf2_ros tf2_echo world link_eef   # live position of the arm's tip
ros2 run tf2_tools view_frames         # writes frames.pdf, a picture of the TF tree
rqt_graph                              # picture of nodes and topics
```

`ros2 <thing> info <name>` works on nodes, topics, services and actions, and tells you who's talking to whom.

---

## 7. Building the workspace (HAND dev container)

The HAND repo already includes `xarm_ros2` as a submodule pinned to the tested commit, and scripts that do the setup for you.

1. Install Docker and VS Code with the Dev Containers extension.
2. Clone with submodules: `git clone --recurse-submodules <HAND repository URL>`
3. Open the repo in VS Code and choose **Dev Containers: Rebuild and Reopen in Container**.
4. In a container terminal:

   ```bash
   bash /workspace/scripts/bootstrap_ros_workspace.sh   # fetches pinned xarm_ros2 + its SDK, installs dependencies
   bash /workspace/scripts/build_ros_workspace.sh       # colcon build
   source /workspace/ros_ws/install/setup.bash          # do this in EVERY new terminal
   ```

5. Launch something:

   ```bash
   bash /workspace/scripts/launch_fake_xarm.sh     # light: fake hardware + planner services (Tutorial 3)
   bash /workspace/scripts/launch_gazebo_xarm.sh   # heavy: Gazebo + MoveIt (Tutorials 1 and 2)
   ```

6. Open the desktop in a browser: `http://localhost:6080/vnc.html?autoconnect=1&resize=scale` (or TigerVNC Viewer at `localhost:5901`).

Gazebo uses software rendering in this container, so it's slow on weaker laptops. Use the fake launch if Gazebo struggles.

If a command says "package not found," you forgot `source /workspace/ros_ws/install/setup.bash` in that terminal. If a window doesn't appear, run `export DISPLAY=:1` in that terminal.
---

## 8. Glossary

| Term | Plain English |
|---|---|
| **Node** | One running program |
| **Launch file** | A script that starts many nodes with the right settings |
| **Topic** | A broadcast channel; one-way, continuous |
| **Service** | A request with one reply |
| **Action** | A long-running job with feedback and a final result; cancellable |
| **Parameter** | A setting on a node |
| **TF / frame** | A coordinate system attached to a part of the robot |
| **URDF** | The robot's physical description |
| **SRDF** | MoveIt's semantic layer: groups, named poses, collision shortcuts |
| **xacro** | XML with variables, used to generate URDF/SRDF |
| **ros2_control** | The framework that runs controllers and talks to hardware |
| **Controller** | A plugin that turns goals into motor commands every cycle |
| **Hardware interface** | The swappable bottom layer: fake, real or simulated |
| **JTC** | Joint Trajectory Controller; follows a timed list of joint positions |
| **MoveIt / move_group** | The motion planner |
| **IK** | Inverse kinematics; "what joint angles put the tip here?" |
| **Planning group** | A named set of joints MoveIt plans for together (`xarm6`, `xarm_gripper`) |
| **TCP** | Tool Center Point; the working tip of the tool (`link_tcp`) |
| **Servo** | Continuous small moves in real time, instead of plan-then-execute |
| **Sim time** | In Gazebo, ROS nodes use the simulator's clock (`/clock`) instead of the wall clock |

**Next: [Tutorial 1 — xarm_moveit_config](01-moveit-config.md)**
