# HAND bringup

- `sim.launch.py`: fake hardware, hand or mouse input.
- `gazebo.launch.py`: Gazebo physics with mouse input.
- `hardware_mouse.launch.py robot_ip:=ADDRESS`: physical Lite6, mouse only.

The Lite6 hardware launch requires an explicit controller IP and sends no
automatic ready-pose trajectory. The prior startup move was removed after
unexpected physical motion was reported; physical operation remains unvalidated.
Driver activation enables motors. See
[the operating guide](../hand_teleop/README.md#mouse-control-of-the-physical-lite6)
for startup behavior and physical validation limits.

The driver/controller communication patch is applied by
`bash /workspace/scripts/build_ros_workspace.sh`. Hardware startup requires
fresh successful driver reads as well as broadcaster feedback; a continuous
watchdog exits on feedback loss. Driver read/write failures latch transmission
off and require hardware reload. These changes have offline fault-injection
coverage and still require physical validation when the arm is available.
