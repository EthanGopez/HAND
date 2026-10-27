# ee_target_tracker contract

Turns a slow end-effector goal into a real-time stream the arm can follow.

Nodes that decide *where* the end effector should go (MoveIt planning through
`xarm_planner`, the vision grasp demos in `xarm_vision`, future EMG intent) produce a
good target only a few times a second, and sometimes irregularly. MoveIt Servo needs a
fresh command at least every 0.2 s or it stops the arm. This node sits between them: it
holds the latest slow goal, and every cycle it emits an interpolated setpoint and a
bounded velocity toward it.

This node does not solve inverse kinematics. Servo does that, using the same
`kinematics.yaml` as `move_group`.

## Inputs

### Goal

- Topic: `ee_target/raw` (resolves to `/ee_target/raw` with no namespace)
- Type: `geometry_msgs/msg/PoseStamped`
- Frame: `link_base`. A pose in any other frame is transformed through TF; if the
  transform is unavailable the pose is rejected.
- Rate: whatever the producer manages, including irregular and slower than 1 Hz.
- Used fields: `header.frame_id`, `pose.position`, `pose.orientation`.

### Current end-effector pose

- Source: TF2, `link_base` -> `link_eef`
- Used every cycle as the point the setpoint starts from. The node never assumes the arm
  reached the last setpoint it published.

## Outputs

### Setpoint

- Topic: `ee_target/setpoint`
- Type: `geometry_msgs/msg/PoseStamped`
- Frame: `link_base`
- Rate: `publish_rate` (default 50 Hz), published continuously.
- Meaning: where the end effector should be at the next step. When there is no valid
  goal, this is the current end-effector pose, so a consumer that follows it holds still.

### Velocity command

- Topic: `/servo_server/delta_twist_cmds`
- Type: `geometry_msgs/msg/TwistStamped`
- Frame: `link_base` (matches Servo's `robot_link_command_frame`)
- Rate: `publish_rate`, published continuously, zeros included.
- Units: m/s and rad/s. Servo runs with `command_in_type: speed_units`, so these are real
  units, not normalized.
- **Off by default.** Set `publish_twist` to `true` to enable. Only one node may drive
  `delta_twist_cmds` at a time, and `hand_teleop` already publishes to it. Running both is
  a configuration error; `hand_bringup` is responsible for launching at most one.

### Status

- Topic: `ee_target/status`
- Type: `hand_interfaces/msg/EeTargetStatus`
- Rate: 10 Hz, published continuously.

| State       | Value | Meaning                                                        |
|-------------|-------|----------------------------------------------------------------|
| `NO_TARGET` | 0     | No goal received yet, or the goal was cleared                   |
| `TRACKING`  | 1     | Moving toward a valid goal                                      |
| `REACHED`   | 2     | Within `position_tolerance` and `orientation_tolerance`         |
| `STALE`     | 3     | Last goal is older than `stale_after`                           |
| `REJECTED`  | 4     | Last goal was outside the workspace box or had no TF            |

## Mapping

Each cycle, with `current` from TF and `goal` the latest accepted goal:

```
error      = goal.position - current.position
speed      = clamp(gain * |error|, 0, max_speed)
linear     = speed * normalize(error)                      # zero if |error| < deadband
angular    = clamp(angular_gain * axis_angle(current, goal), max_angular_speed)
setpoint   = current.pose advanced by (linear, angular) * (1 / publish_rate)
```

- Position and orientation are capped independently, so a large rotation never inflates
  translation speed.
- `deadband` is applied to the total position error, not per axis, so the arm does not
  creep along one axis near the goal.
- A new goal replaces the old one immediately. There is no queue and no blending between
  goals; the motion stays continuous because it always starts from the measured pose.

## Limits (ROS parameters)

| Parameter                | Default | Meaning                                                      |
|--------------------------|---------|---------------------------------------------------------------|
| `publish_rate`           | 50.0    | Setpoint and twist rate, Hz                                   |
| `max_speed`              | 0.05    | Hard cap on linear speed, m/s                                 |
| `max_angular_speed`      | 0.5     | Hard cap on angular speed, rad/s                              |
| `gain`                   | 1.0     | Linear speed per metre of position error, 1/s                 |
| `angular_gain`           | 1.0     | Angular speed per radian of orientation error, 1/s            |
| `deadband`               | 0.002   | Position errors smaller than this give zero linear speed, m   |
| `stale_after`            | 1.0     | Seconds without a new goal before the node holds position     |
| `position_tolerance`     | 0.005   | Distance at or under which the state is `REACHED`, m          |
| `orientation_tolerance`  | 0.02    | Angle at or under which the state is `REACHED`, rad           |
| `publish_twist`          | false   | Publish to `/servo_server/delta_twist_cmds`                   |
| `workspace_min`          | `[-0.7, -0.7, 0.0]`  | Lower corner of the accepted goal box, m, `link_base` |
| `workspace_max`          | `[0.7, 0.7, 0.8]`    | Upper corner of the accepted goal box, m, `link_base` |

The workspace box is a crude guard against a bad goal from vision, not a reachability
check. Servo and the controller still enforce joint limits and singularity protection.

## Safety behavior

- No goal yet, goal older than `stale_after`, goal rejected, or the `link_base` ->
  `link_eef` transform unavailable: publish a zero twist and a setpoint equal to the
  current pose. The arm holds still.
- A rejected goal is not remembered. The node reports `REJECTED` and keeps holding until
  an acceptable goal arrives; it never falls back to an older goal.
- Staleness is measured from when the node received the goal, not from the header stamp,
  matching `hand_teleop`.
- `stale_after` is 1.0 s here rather than 0.2 s because the producers are expected to be
  slow. The node itself keeps Servo fed inside its 0.2 s timeout regardless.
- Speed is clamped every cycle, after the gain is applied.
- On shutdown the node publishes one zero twist before exiting.

## Relationship to other nodes

| Node               | Role                                                                |
|--------------------|----------------------------------------------------------------------|
| `xarm_planner`     | Produces goals (slow). Publishes to `ee_target/raw`.                 |
| `xarm_vision`      | Produces goals from detected objects (slow).                         |
| `ee_target_tracker`| This node. Goal in, continuous setpoint and velocity out.            |
| `servo_server`     | Consumes the twist, solves IK, drives the joints.                    |
| `hand_teleop`      | Alternative source of twists. Mutually exclusive with `publish_twist`.|

## Open questions

- Whether `ee_target/raw` should also accept a `PoseArray` waypoint path. Deferred until a
  producer needs it; the single-pose form covers the vision and planner cases today.
- Whether orientation tracking should be optional, for producers that only know a point in
  space. A goal's orientation can be set to the current end-effector orientation by the
  producer, so no parameter is proposed yet.
