# Tutorial 5: Hands-On Check

Launch the fake xArm-6, inspect its topics, transforms and services, move it
from the command line, and post proof. This confirms your dev container works
and that you can see the ROS concepts from
[Tutorial 0](00-start-here.md) in a running system. No
hardware, no robot IP.

## Setup

In the HAND dev container (see the top-level README of the HAND repo):

```bash
bash /workspace/scripts/bootstrap_ros_workspace.sh
bash /workspace/scripts/build_ros_workspace.sh
bash /workspace/scripts/launch_fake_xarm.sh
```

Open `http://localhost:6080/vnc.html?autoconnect=1&resize=scale`. You should
see RViz with an xArm-6. Leave the launch running.

Open a **second** container terminal (VS Code: Terminal → New Terminal) and run:

```bash
source /workspace/ros_ws/install/setup.bash
```

## The hunt

1. **Nodes.** Run `ros2 node list`. Pick three nodes and write one sentence
   each on what they do. (Tutorial 1, section 2 has a table.)
2. **A topic.** Run `ros2 topic echo /joint_states --once`. What is `joint1`'s
   angle? What units?
3. **A transform.** Run `ros2 run tf2_ros tf2_echo world link_eef` for a few
   seconds (Ctrl-C to stop). Write down the translation of the arm's tip.
4. **Move the arm with services.**

   ```bash
   ros2 service call /xarm_joint_plan xarm_msgs/srv/PlanJoint "{target: [0.785, 0.0, 0.0, 0.0, 0.0, 0.0]}"
   ros2 service call /xarm_exec_plan xarm_msgs/srv/PlanExec "{wait: true}"
   ```

   Watch the arm turn in RViz.
5. **Check it moved.** Repeat step 2 and step 3. Which numbers changed, and by
   how much?
6. **Send it home.**

   ```bash
   ros2 service call /xarm_joint_plan xarm_msgs/srv/PlanJoint "{target: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
   ros2 service call /xarm_exec_plan xarm_msgs/srv/PlanExec "{wait: true}"
   ```

## Done when

Post in the team channel:

- one RViz screenshot with the arm rotated;
- your answers to steps 1, 2, 3 and 5;
- anything that went wrong during setup (so it can be fixed for everyone).

## Troubleshooting

| Problem | Fix |
|---|---|
| `Package 'xarm_msgs' not found` / command not found | `source /workspace/ros_ws/install/setup.bash` in that terminal |
| Service call hangs with "waiting for service" | The launch isn't running or hasn't finished starting. Check the first terminal for errors |
| Nothing in the browser | Check that VS Code's **Ports** panel forwards 6080, or try TigerVNC Viewer on `localhost:5901` |
| RViz has no planning panel | Expected: `launch_fake_xarm.sh` opens a view-only RViz. You move the arm with the service calls above |
