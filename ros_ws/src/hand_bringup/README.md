# `hand_bringup`

Launch files that connect HAND nodes with the xArm simulation. Simulation only: nothing
here takes a robot IP, and physical-arm launch must stay explicit and separate.

- `launch/gazebo_tracker.launch.py`: the stock xArm6 Gazebo/MoveIt launch, plus MoveIt
  Servo (which that launch lacks), plus exactly one node driving Servo.
- `prepare_arm`: moves the arm off its singular start pose, then starts Servo.
- `unstick_gazebo_gui`: wakes `gzclient` if it deadlocks on startup (see below), then
  maximizes and raises the Gazebo window.

```bash
bash /workspace/scripts/launch_gazebo_tracker.sh                       # ee_target_tracker
bash /workspace/scripts/launch_gazebo_tracker.sh max_speed:=0.1        # faster
bash /workspace/scripts/launch_gazebo_tracker.sh driver:=teleop        # hand_teleop instead
bash /workspace/scripts/launch_gazebo_tracker.sh driver:=none          # Servo only
```

`driver` is how the "only one node may publish `/servo_server/delta_twist_cmds`" rule is
enforced. Other arguments: `max_speed`, `max_angular_speed`, `stale_after`,
`servo_filter_coeff`.

Why Servo needs the extra steps is explained in
[`docs/interfaces/ee_target_tracker.md`](../../../docs/interfaces/ee_target_tracker.md).

## Why `unstick_gazebo_gui` exists

In the dev container on Apple Silicon (an amd64 image under emulation), `gzclient` stalls
early in startup on a lost wakeup. It sits at about 0% CPU and never opens its window, even
for plain Gazebo with an empty world and no HAND code. Stopping and continuing the process
(`SIGSTOP`, `SIGCONT`, what attaching a debugger does) frees it within seconds. The helper
does that until the window exists. It uses `xwininfo` (`x11-utils`) to find the window and
`wmctrl` to raise it; both are installed by `.devcontainer/post-create.sh`. Anything that
starts `gzclient` another way, such as `scripts/launch_gazebo_xarm.sh`, can hit the same
stall: run `ros2 run hand_bringup unstick_gazebo_gui` next to it.
