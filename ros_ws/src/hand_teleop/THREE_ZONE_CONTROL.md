# Three-zone control

The palm guard publishes JSON on `/palm_teleop/guard_status` with the current
condition number, final predicted condition, zone, applied speed scale, output
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

Palm-node parameters are `singularity_warning` (17), `singularity_halt` (25),
`singularity_horizon` (0.3 seconds), `singularity_step` (0.05 seconds), and
`singularity_hysteresis` (0.5). Invalid combinations fail closed. These are startup
parameters, not live tuning controls. The halt parameter cannot exceed 25.
Hysteresis delays exiting a zone; it never delays entering a more restrictive zone.

Offline checks without ROS:

```bash
PYTHONPATH=ros_ws/src/hand_teleop python3 -m unittest discover \
  -s ros_ws/src/hand_teleop/test -p test_zone_policy.py
```
