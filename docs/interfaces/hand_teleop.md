# hand_teleop contract

Translates hand position into velocity commands for MoveIt Servo, with safety limits.

## Input

- Topic: `hand/landmarks` (resolves to `/hand/landmarks` with no namespace)
- Type: `hand_interfaces/msg/HandLandmarks`
- Used fields: `tracked`, `landmarks[0]` (wrist; `x` and `y`, normalized 0-1)

## Output

- Topic: `/servo_server/delta_twist_cmds`
- Type: `geometry_msgs/msg/TwistStamped`
- Frame: `link_base` (directions are relative to the arm's base)
- Rate: 20 Hz, published continuously (zeros included)
- Axes: `linear.y` and `linear.z` only. All other fields are always 0.

## Mapping

- `linear.y = limit(-(wrist.x - 0.5))`
- `linear.z = limit(-(wrist.y - 0.5))`
- Hand up in the image means arm up. Flip the sign of the y term if motion comes out mirrored.

## Limits (ROS parameters)

| Parameter     | Default | Meaning                                              |
|---------------|---------|------------------------------------------------------|
| `max_speed`   | 0.05    | Hard cap on each axis, m/s                           |
| `gain`        | 0.25    | m/s per unit of image offset                         |
| `deadband`    | 0.05    | Offsets smaller than this (image fraction) give zero |
| `stale_after` | 0.2     | Seconds without landmarks before the command is zero |

## Safety behavior

- No message received yet, `tracked == false`, or last message older than `stale_after`: publish a zero twist.
- Speed is clamped to `max_speed` on every axis.
- Staleness is measured from when the node received the message, not from the header stamp.
- 20 Hz output keeps Servo supplied inside its 0.2 s command timeout.
