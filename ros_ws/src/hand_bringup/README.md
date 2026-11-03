# HAND bringup

- `sim.launch.py`: fake hardware, hand or mouse input.
- `gazebo.launch.py`: Gazebo physics with mouse input.
- `hardware_mouse.launch.py [robot_ip:=ADDRESS]`: physical xArm 6, mouse only.

The hardware launch discovers a single xArm 6 on the local wired LAN by default;
an explicit controller IP overrides discovery. It has no automatic ready-pose movement. Driver activation enables motors. See
[the operating guide](../hand_teleop/README.md#mouse-control-of-the-physical-xarm-6)
for build, startup, and physical validation limits.
