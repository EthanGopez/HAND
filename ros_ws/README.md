# ROS 2 Workspace

This is the production ROS 2 Humble workspace for HAND.

- `src/xarm_ros2/` is the pinned UFACTORY source used by the linked
  `xArm6_manipulation` reference repository. It is tracked as a Git submodule,
  including its nested xArm C++ SDK.
- HAND-owned packages live under `src/`. `hand_interfaces`, `hand_teleop`,
  `hand_ee_target_tracker` and `hand_bringup` have code; the rest are still placeholders.
- `build/`, `install/`, and `log/` are local generated directories and are ignored by Git.

## Run xArm-6 in Gazebo with MoveIt

Inside the HAND dev container:

```bash
bash /workspace/scripts/bootstrap_ros_workspace.sh
bash /workspace/scripts/build_ros_workspace.sh
bash /workspace/scripts/launch_gazebo_xarm.sh
```

The launch script starts TigerVNC automatically. View the Gazebo/RViz desktop
using either:

- a browser at `http://localhost:6080/vnc.html?autoconnect=1&resize=scale`; or
- TigerVNC Viewer connected to `localhost:5901`.

Both display the container's `:1` desktop. Docker publishes `6080` on your
machine's `127.0.0.1` only (`runArgs` in `.devcontainer/devcontainer.json`, so it
works whether or not VS Code is attached; this needs a **Rebuild Container** once).
The desktop has no password, so keep it bound to localhost. VS Code forwards `5901`
automatically; otherwise add it from its **Ports** panel. This launch path uses
Gazebo's simulated controller and does not connect to a physical arm.

## Move the Gazebo arm to a point

```bash
bash /workspace/scripts/launch_gazebo_tracker.sh
ros2 run hand_ee_target_tracker send_goal 0.30 0.10 0.25
```

This starts the same Gazebo/MoveIt simulation plus MoveIt Servo and
`ee_target_tracker`, so a goal published on `ee_target/raw` moves the arm. See
`src/hand_bringup/README.md` and `docs/interfaces/ee_target_tracker.md`.

For the lighter fake-hardware MoveIt/RViz demo:

```bash
bash /workspace/scripts/launch_fake_xarm.sh
```

Start member onboarding with `onboarding/vha/README.md`.
