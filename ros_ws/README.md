# ROS 2 Workspace

This is the production ROS 2 Humble workspace for HAND.

- `src/xarm_ros2/` is the pinned UFACTORY source used by the linked
  `xArm6_manipulation` reference repository. It is tracked as a Git submodule,
  including its nested xArm C++ SDK.
- HAND-owned packages are scaffolded under `src/` but have not been generated yet.
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

Both display the container's `:1` desktop. VS Code should forward ports `5901`
and `6080` automatically; otherwise add them from its **Ports** panel. This
launch path uses Gazebo's simulated controller and does not connect to a
physical arm.

For the lighter fake-hardware MoveIt/RViz demo:

```bash
bash /workspace/scripts/launch_fake_xarm.sh
```

Start member onboarding with `onboarding/vha/README.md`.
