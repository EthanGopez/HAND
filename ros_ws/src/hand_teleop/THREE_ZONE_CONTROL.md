# Three-zone control

The shared guard publishes JSON on `/palm_teleop/guard_status` or
`/mouse_teleop/guard_status` with the current
condition number, maximum predicted condition, zone, applied speed scale, output
velocity, and rejection reason. Non-finite conditions are represented as null.
The preview uses the active URDF and the complete Cartesian velocity vector.
It reports predicted joint-limit margin and joint-speed usage internally.

These calculations are a local preview, not a collision checker or a guarantee
that MoveIt Servo uses the same Jacobian metric. Servo and hardware fault
protections remain authoritative.

Normal commands pass unchanged. Approaching commands near the warning boundary
are multiplied by a smoothstep speed factor, preserving direction. The complete
scaled command is predicted again; a bounded backtracking search rejects any
remaining prediction reaching the halt boundary. Retreat remains available below
that boundary. The guard never substitutes a different direction or creates motion
from zero input. Joint margin (0.1 radians), velocity, and feedback checks still apply.

Palm and mouse parameters are `singularity_warning` (17), `singularity_halt` (25),
`singularity_horizon` (0.3 seconds), `singularity_step` (0.05 seconds), and
`singularity_hysteresis` (0.5). Invalid combinations fail closed. These are startup
parameters, not live tuning controls. The halt parameter cannot exceed 25.
Hysteresis delays exiting a zone; it never delays entering a more restrictive zone.

## Recovery is opt-in

Recovery is experimental and disabled by default. The simulation launcher exposes
`allow_recovery:=true`; physical launch defaults remain disabled. This does not
change MoveIt Servo's own stop behavior, limits, or collision checks. The HAND
condition metric is a geometric 6D Jacobian combining translation and rotation;
its numerical thresholds must not be assumed equivalent to Servo's metric.

In the recovery zone (25 by default), only commands improving condition at every
preview step can pass. `singularity_recovery_speed` defaults to 0.005 m/s and
cannot exceed 0.01. `singularity_recovery_ceiling` defaults to 28 and cannot be
raised above 28. At the ceiling, an actual singularity, invalid model/feedback,
or a violated joint margin, output remains zero. No automatic escape trajectory
or joint-limit override is provided. Recovery requires all of:

1. `allow_recovery` enabled when the node starts.
2. After entry into recovery, a false `std_msgs/Bool` on the node's private
   `~/recovery_hold` topic, followed by true heartbeats from a held operator control.
3. A new input reference and fresh hand/pointer movement.
4. Confirmed joint feedback and a passing full-vector preview.

The heartbeat must remain fresh within 0.2 seconds. A latched true value or a
single true message cannot keep recovery armed. Timeouts, zone transitions,
model changes, invalid feedback and rejected enabled recovery revoke the hold.
Release and press again to rearm. The input adapter must publish false on release
and true only while the operator holds its recovery control; a continuously
running command-line publisher is not an operator deadman. No such physical input
device is configured by this change. Mouse control also retains click/Space gating.

The topic names are `/palm_teleop/recovery_hold` and `/mouse_teleop/recovery_hold`.
The same shared guard supplies diagnostics and parameters for both input sources.
Rejected or scaled-away displacement is consumed, never queued for later replay.

## Startup

Physical startup still sends no trajectory. It now waits for the active robot
description and a fresh measured pose that passes the shared kinematic checks,
in addition to active controllers and dual-source feedback. A recovery-zone pose
does not pass startup even if recovery is later enabled. Reposition with the
reviewed operator procedure first; startup logs the reason while waiting.

Simulation retains its existing ready trajectory, stops Servo during that move,
and now checks the measured result before restarting Servo. An unsuitable result
fails initialization rather than enabling teleop. Direct simulation initial-state
configuration is deferred until the actual fake/Gazebo model and runtime can be
validated. The ordinary fake demo is xArm6; physical operation is Lite6.

## Validation boundary

Tests cover real Lite6 model predictions, speed scaling, hysteresis, recovery
direction/cap/ceiling, release-to-arm heartbeat expiry, model/feedback gating,
mouse regressions, and startup refusing unsuitable poses. Callback tests use
ROS message types and mocked nodes; they do not establish controller behavior.

Before enabling recovery on hardware, run the installed MoveIt Servo with fake
Lite6 hardware and compare both condition/status streams during approach, reverse,
and recovery. Verify achieved velocity, joint limits, and stop behavior. The
development container used for this change lacks `moveit_servo`, so this full
stack test and physical validation have not been performed. Do not raise Servo's
thresholds to make a rejected recovery command execute.

Offline checks without ROS:

```bash
PYTHONPATH=ros_ws/src/hand_teleop python3 -m unittest discover \
  -s ros_ws/src/hand_teleop/test -p test_zone_policy.py
```
