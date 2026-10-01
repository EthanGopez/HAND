# Tutorial 3: `xarm_planner` — Moving the Arm from Your Own Code

**You'll learn:** what the planner node is for, the plan-then-execute pattern, how to move the arm with plain service calls from the terminal, Python and C++, how pose targets and straight-line moves work, and how the gripper fits in.

**Prerequisites:** [Tutorial 1](01-moveit-config.md). Tutorial 2 is useful if you want to run this in Gazebo.

---

> **HAND team notes (read first)**
> - Everything here runs inside the **HAND dev container**. View RViz and Gazebo at `http://localhost:6080/vnc.html?autoconnect=1&resize=scale`.
> - Every new terminal: `source /workspace/ros_ws/install/setup.bash`. If you run `ros2 launch` yourself (instead of a `scripts/launch_*.sh` script), also run `export DISPLAY=:1` first or windows won't appear in VNC.
> - **Never pass a `robot_ip` or run a `*_realmove*` launch file.** Real-arm sections are for understanding only; physical-arm operation needs the team's separate reviewed procedure.

## 1. What this package is for

In Tutorial 1 you moved the arm by clicking in RViz. Real applications need code to do that: "go to this joint configuration," "put the gripper above the box," "move straight down 10 cm."

MoveIt has a full C++ API for this (`MoveGroupInterface`), but it's big. `xarm_planner` wraps the handful of things most people need into:

- **a node, `xarm_planner_node`, that offers simple ROS services.** Any program in any language can move the arm by calling them.
- **a small C++ class, `XArmPlanner`.** That node is built on it, and so are the example programs.

```
 your code ──service call──▶ xarm_planner_node ──MoveGroupInterface──▶ move_group ──▶ controller ──▶ arm
```

The planner node **doesn't plan by itself**. It's a client of `move_group`, which must already be running. It just turns "simple request" into "full MoveIt request."

---

## 2. The core idea: plan, then execute

Every motion is two separate steps:

1. **Plan.** Ask for a motion to a target. MoveIt computes a collision-free, timed trajectory and the planner node **stores it**. Nothing moves yet. You get back `success: true` or `false`.
2. **Execute.** Tell the node to run the stored plan. The arm moves.

Separating them lets you check that a plan exists before committing, show it to a user, or decide not to move at all.

---

## 3. Start it

```bash
# fake hardware — this is exactly what scripts/launch_fake_xarm.sh runs
bash /workspace/scripts/launch_fake_xarm.sh

# or with options (e.g. a gripper):
export DISPLAY=:1
ros2 launch xarm_planner xarm6_planner_fake.launch.py [add_gripper:=true]

# Gazebo
export DISPLAY=:1
ros2 launch xarm_planner xarm6_planner_gazebo.launch.py [add_gripper:=true]

# real arm — reference only, NOT for HAND without a reviewed procedure
# ros2 launch xarm_planner xarm6_planner_realmove.launch.py robot_ip:=...
```

These are the Tutorial 1 MoveIt launches with one setting changed, `no_gui_ctrl:=true`, which does two things:

- RViz opens in **view-only** mode (no MotionPlanning panel) so it doesn't fight your code for control.
- The planner node(s) start.

Check what appeared:

```bash
ros2 node list | grep planner
ros2 service list | grep xarm_
```

```
/xarm_planner_node
/xarm_joint_plan
/xarm_pose_plan
/xarm_straight_plan
/xarm_exec_plan
```

With `add_gripper:=true` you also get `/xarm_gripper_planner_node`, `/xarm_gripper_joint_plan` and `/xarm_gripper_exec_plan`.

In the launch terminal, the planner node prints its planning frame and end-effector link. Note them. Pose targets (section 5) are expressed in that frame, for that link.

---

## 4. Move by joint angles

The simplest request: "put the joints at these angles." Values are in **radians**, one per joint, in order `joint1` … `joint6`.

```bash
# Plan: rotate the base 45° (0.785 rad), everything else at zero
ros2 service call /xarm_joint_plan xarm_msgs/srv/PlanJoint "{target: [0.785, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

```
response: xarm_msgs.srv.PlanJoint_Response(success=True)
```

Nothing has moved. In RViz you may see the planned path shown as a preview. Now:

```bash
# Execute: wait=true means "don't reply until the motion is finished"
ros2 service call /xarm_exec_plan xarm_msgs/srv/PlanExec "{wait: true}"
```

The arm moves. Send it back:

```bash
ros2 service call /xarm_joint_plan xarm_msgs/srv/PlanJoint "{target: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
ros2 service call /xarm_exec_plan xarm_msgs/srv/PlanExec "{wait: true}"
```

**What happened underneath:**

1. The node calls MoveIt's `setJointValueTarget(...)` then `plan(...)`. That becomes a request to `move_group`'s `/move_action` with "plan only."
2. `move_group` plans with OMPL and times the path using the joint limits, scaled down to **30% of max velocity and 10% of max acceleration** (the planner node's fixed defaults).
3. The node keeps the result in memory.
4. `xarm_exec_plan` sends that stored trajectory to `move_group`'s `/execute_trajectory` action, which passes it to `/xarm6_traj_controller/follow_joint_trajectory`.

If a joint value is outside its limits, or no collision-free path exists, planning returns `success: false` and there's nothing to execute.

---

## 5. Move to a pose

Usually you care where the **tool** is, not what the joints are. A **pose** is a position (x, y, z in metres) plus an orientation (a quaternion).

```bash
ros2 service call /xarm_pose_plan xarm_msgs/srv/PlanPose \
  "{target: {position: {x: 0.3, y: 0.0, z: 0.2},
             orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}"
ros2 service call /xarm_exec_plan xarm_msgs/srv/PlanExec "{wait: true}"
```

This puts the tool 30 cm in front of the base and 20 cm up, pointing straight down.

### Understanding the pose

- **Which frame?** The planning frame printed at start-up, the root of the robot model. By default that's the frame the arm's base sits on, so x is forward, y is left, z is up, measured from the centre of the base.
- **Which point on the robot?** The end-effector link of the group: `link_eef` (the flange at the end of joint 6) with no tool, or `link_tcp` (the tool tip) with a gripper attached. You can check it with `ros2 run tf2_ros tf2_echo world link_tcp`.
- **The orientation `{x: 1, y: 0, z: 0, w: 0}`** is a 180° rotation about the x axis. The "no rotation" quaternion (`w: 1`) would point the tool's z axis straight up; this one flips it to point straight down, which is also how the tool faces when every joint is at zero. "Pointing down" is what you want for most pick-and-place.

**Tip:** to find a good pose, move the arm to it in RViz (Tutorial 1), then run `ros2 run tf2_ros tf2_echo world link_eef` (or `link_tcp`) and copy the translation and rotation it prints.

**What happened underneath:** the node called `setPoseTarget(...)`. `move_group` first uses **inverse kinematics** to find joint angles that reach the pose, then plans to those joint angles exactly as in section 4. A pose target doesn't say anything about the *path*: the tool may swing through a curve on the way.

---

## 6. Move in a straight line

When the path matters (lowering onto an object, pushing a button), use a Cartesian move. The tool travels in a straight line from where it is now to the target:

```bash
# from the pose in section 5, go straight up 10 cm
ros2 service call /xarm_straight_plan xarm_msgs/srv/PlanSingleStraight \
  "{target: {position: {x: 0.3, y: 0.0, z: 0.3},
             orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}"
ros2 service call /xarm_exec_plan xarm_msgs/srv/PlanExec "{wait: true}"
```

**What happened underneath:** the node called MoveIt's `computeCartesianPath(...)`. MoveIt steps along the straight line in **5 mm increments**, solving IK at each step. It reports what **fraction** of the line it could follow (a joint limit or collision can cut it short). The planner node treats the plan as a success if it reached at least **90%** of the line.

Straight-line moves are best kept short. Long ones through awkward regions often can't be completed.

---

## 7. The gripper

With `add_gripper:=true`, a second node, `xarm_gripper_planner_node`, controls the gripper group the same way. The gripper has one joint, `drive_joint`, from **0.0 (open)** to **0.85 (closed)**:

```bash
# close
ros2 service call /xarm_gripper_joint_plan xarm_msgs/srv/PlanJoint "{target: [0.85]}"
ros2 service call /xarm_gripper_exec_plan xarm_msgs/srv/PlanExec "{wait: true}"

# open
ros2 service call /xarm_gripper_joint_plan xarm_msgs/srv/PlanJoint "{target: [0.0]}"
ros2 service call /xarm_gripper_exec_plan xarm_msgs/srv/PlanExec "{wait: true}"
```

In fake and Gazebo mode, this ends up at `xarm_gripper_traj_controller`. On the real arm it ends up at the driver's `/xarm_gripper/gripper_action` (Tutorial 1, section 4). Your code doesn't change either way.

---

## 8. Calling the services from Python

Anything you can do from the terminal, your code can do. Here's a complete Python node that moves the arm through three joint configurations:

```python
#!/usr/bin/env python3
import rclpy
from xarm_msgs.srv import PlanJoint, PlanExec


def main():
    rclpy.init()
    node = rclpy.create_node('my_xarm_mover')

    plan_client = node.create_client(PlanJoint, 'xarm_joint_plan')
    exec_client = node.create_client(PlanExec, 'xarm_exec_plan')
    plan_client.wait_for_service()
    exec_client.wait_for_service()

    def call(client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        return future.result()

    targets = [
        [0.785, 0.0, 0.0, 0.0, 0.0, 0.0],      # base rotated 45°
        [0.0, 0.0, 0.0, 0.0, -1.5708, 0.0],    # the "hold-up" pose
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],        # home
    ]

    for target in targets:
        planned = call(plan_client, PlanJoint.Request(target=target))
        if not planned.success:
            node.get_logger().error(f'No plan for {target}, stopping')
            break
        executed = call(exec_client, PlanExec.Request(wait=True))
        node.get_logger().info(f'Moved to {target}: {executed.success}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Save it as `my_xarm_mover.py` anywhere under `/workspace` (outside `ros_ws/src` is fine for now) and run `python3 my_xarm_mover.py` in a sourced terminal while `launch_fake_xarm.sh` is up. Things to notice:

- The service names are relative (`xarm_joint_plan`), so they resolve to `/xarm_joint_plan` unless you run in a namespace.
- **Always check `success` after planning** before you execute.
- In Gazebo, run your node with `--ros-args -p use_sim_time:=true` (Tutorial 2, section 4).

---

## 9. The C++ class and the examples

The planner node is a thin shell around `XArmPlanner` (`include/xarm_planner/xarm_planner.h`):

```cpp
xarm_planner::XArmPlanner planner(node, "xarm6");   // group name

planner.planJointTarget({0.785, 0, 0, 0, 0, 0});     // returns bool
planner.planPoseTarget(pose);                        // geometry_msgs::msg::Pose
planner.planPoseTargets(poses);                      // any of several poses
planner.planCartesianPath(waypoints);                // straight lines through waypoints
planner.executePath();                               // wait=true by default
```

Each `plan…` call stores its result; `executePath()` runs whichever was planned last. Inside, each method is a few lines of `MoveGroupInterface`, so `src/xarm_planner.cpp` doubles as a short, readable introduction to MoveIt's C++ API.

The package ships example programs in `test/` in both styles. Each has its own launch file; start the matching `…_planner_fake.launch.py` first, then:

| Example | Style | What it does |
|---|---|---|
| `test_xarm_planner_client_joint` | Service calls | Cycles through joint targets |
| `test_xarm_planner_client_pose` | Service calls | Cycles through four poses |
| `test_xarm_planner_api_joint` | C++ class | Same as the client version, using `XArmPlanner` directly |
| `test_xarm_planner_api_pose` | C++ class | Four poses using `XArmPlanner` |
| `test_xarm_gripper_planner_client_joint` / `…_api_joint` | Both | Opens and closes the gripper |

```bash
ros2 launch xarm_planner test_xarm_planner_client_joint.launch.py dof:=6
ros2 launch xarm_planner test_xarm_planner_api_pose.launch.py dof:=6 robot_type:=xarm
```

They loop forever; stop them with Ctrl-C.

---

## 10. Service reference

| Service | Request | What it does |
|---|---|---|
| `/xarm_joint_plan` | `float64[] target` (radians) | Plan to joint angles |
| `/xarm_pose_plan` | `geometry_msgs/Pose target` | Plan to a tool pose (any path) |
| `/xarm_straight_plan` | `geometry_msgs/Pose target` | Plan a straight line to a pose |
| `/xarm_exec_plan` | `bool wait` | Run the last plan (`wait=false` returns immediately) |
| `/xarm_gripper_joint_plan` | `float64[] target` (one value, 0–0.85) | Plan the gripper |
| `/xarm_gripper_exec_plan` | `bool wait` | Run the gripper plan |

Every response is `bool success`.

---

## 11. Common questions

**The plan succeeds but execute fails.**
Something moved the arm between planning and executing (MoveIt checks that the arm is still where the plan starts), or the controller isn't active.

**Pose planning fails for a pose that looks reachable.**
Either the orientation isn't achievable at that spot, or IK timed out. Try a slightly different position, or check by dragging the marker in RViz (with the normal MoveIt launch) to the same place.

**Can I plan for the arm and gripper at once?**
Not with these services: they're separate nodes for separate groups. Plan and execute them one after the other.

**Why are moves slow?**
The planner uses 30% velocity and 10% acceleration scaling. That's deliberate for safety.

---

## Recap

- `xarm_planner_node` is a simple front door to MoveIt, not a planner itself; `move_group` must be running.
- Every motion is **plan** (`*_plan`, stores the result) then **execute** (`*_exec_plan`).
- Joint targets are radians; pose targets are metres + quaternion in the planning frame, for the end-effector link.
- Straight-line moves step in 5 mm increments and succeed at ≥90% coverage.
- The gripper has its own node and services; the same calls work in fake, Gazebo and real mode.

**Next: [Tutorial 4 — xarm_moveit_servo](04-servo.md)**
