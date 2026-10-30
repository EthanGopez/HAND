# `hand_ee_target_tracker`

Turns a slow end-effector goal into a fast, bounded twist stream for MoveIt Servo. The
behavior, topics and parameters are defined in
[`docs/interfaces/ee_target_tracker.md`](../../../docs/interfaces/ee_target_tracker.md).

- `ee_target_tracker`: the node. Goal in on `ee_target/raw`, setpoint on
  `ee_target/setpoint`, health on `ee_target/status`, twist on
  `/servo_server/delta_twist_cmds` only when `publish_twist` is `true`.
- `send_goal`: gives the node a point and keeps republishing it until the arm gets there.
- `hand_ee_target_tracker/control.py`: the control law, with no ROS imports.

```bash
ros2 run hand_ee_target_tracker send_goal 0.30 0.10 0.25                 # keep the current orientation
ros2 run hand_ee_target_tracker send_goal 0.30 0.10 0.25 --rpy 3.14159 0 0.6
ros2 topic echo /ee_target/status
```

The whole simulation is started by `hand_bringup`; see `ros_ws/README.md`.

Unit tests for the control law need no ROS:
`python3 -m pytest ros_ws/src/hand_ee_target_tracker/test/test_control.py`.
