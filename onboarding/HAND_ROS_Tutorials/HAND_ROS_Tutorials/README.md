# HAND ROS Tutorials: How the xArm Stack Works

**Two ways to read these:** open the `.md` files on GitHub or in VS Code (right-click → Open Preview), or double-click the matching file in the `html/` folder to read it in any browser.

Plain-English tutorials for the UFACTORY `xarm_ros2` packages that HAND builds on
(pinned at `ros_ws/src/xarm_ros2`, commit `62936f7`). They explain what runs,
how the pieces talk to each other, and how a command becomes arm motion, with
commands you can run in the HAND dev container to see it happen.

Everything is simulation-only. Never pass a `robot_ip`.

## Reading order

| # | Tutorial | What you'll understand |
|---|---|---|
| 0 | [Start Here](00-start-here.md) | ROS 2 basics (nodes, topics, services, actions, TF, URDF), ros2_control, fake vs Gazebo vs real, building the workspace |
| 1 | [`xarm_moveit_config`](01-moveit-config.md) | MoveIt: how a click in RViz becomes a planned, executed motion |
| 2 | [`xarm_gazebo`](02-gazebo.md) | The physics simulator, controllers inside Gazebo, sim time |
| 3 | [`xarm_planner`](03-planner.md) | Moving the arm from your own code with plan/execute services |
| 4 | [`xarm_moveit_servo`](04-servo.md) | Live, continuous steering (what hand teleop will use) |
| 5 | [Hands-on check](05-hands-on-check.md) | Launch the fake arm, read a topic and a transform, move it, post a screenshot |

Do Tutorial 5 (the hands-on check) right after Tutorial 0.

## Which tutorials matter for which HAND package

Everyone reads 0 and 1. Then:

| HAND package | Also read |
|---|---|
| `hand_teleop` | 4 (Servo), then 3 |
| `hand_bringup` | 2, 4 |
| `hand_vision` | 4, section 5 (how commands are streamed) |
| `hand_interfaces` | 3 and 4 (look at the message types each one uses) |
| `hand_gripper_bridge` | 3, section 7 (how the xArm's own gripper is commanded, for contrast) |
| `hand_feedback` | 0 is enough to start |

## Container quick reference

```bash
bash /workspace/scripts/bootstrap_ros_workspace.sh
bash /workspace/scripts/build_ros_workspace.sh
source /workspace/ros_ws/install/setup.bash     # every new terminal
export DISPLAY=:1                               # before any ros2 launch you run yourself

bash /workspace/scripts/launch_fake_xarm.sh     # fake arm + planner services (view-only RViz)
bash /workspace/scripts/launch_gazebo_xarm.sh   # Gazebo + MoveIt
```

Desktop: `http://localhost:6080/vnc.html?autoconnect=1&resize=scale`

The commands and names in these tutorials were checked against the pinned
`xarm_ros2` source. If something doesn't match what you see, open an issue or
tell the VHA lead.
