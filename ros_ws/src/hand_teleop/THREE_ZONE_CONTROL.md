# Three-zone control

The palm guard publishes JSON on `/palm_teleop/guard_status` with the current
condition number, final predicted condition, zone, applied speed scale, output
velocity, and rejection reason. Non-finite conditions are represented as null.
The preview uses the active URDF and the complete Cartesian velocity vector.
It reports predicted joint-limit margin and joint-speed usage internally.

These calculations are a local preview, not a collision checker or a guarantee
that MoveIt Servo uses the same Jacobian metric. Servo and hardware fault
protections remain authoritative.

Offline checks without ROS:

```bash
PYTHONPATH=ros_ws/src/hand_teleop python3 -m unittest discover \
  -s ros_ws/src/hand_teleop/test -p test_zone_policy.py
```
