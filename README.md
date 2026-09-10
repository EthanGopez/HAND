# HAND

HAND is a wearable human-robot interface for teleoperating a UFactory xArm-6,
controlling a custom force-sensing claw through forearm EMG, and returning
contact information through unobtrusive haptic feedback.

Semester 1 is simulation-first and uses harmless vibration feedback. Future
electrotactile work is a separate, safety-gated research track.

## Subteams

1. **Vision, Host, and Arm Control (VHA)**: camera-based hand tracking,
   calibration, bounded motion commands, ROS 2, simulation, and xArm TCP/IP
   integration.
2. **Claw, PCB, and Robot-Side Mechatronics**: Onshape CAD, fabrication,
   servo and force sensing, embedded gripper control, and KiCad board design.
3. **EMG Wearable and Haptic Feedback**: MyoWare acquisition, intent
   detection, wireless communication, vibration cues, and wearable validation.

## VHA quick start

The ROS environment is based on the tested
[`xArm6_manipulation`](https://github.com/EthanGopez/xArm6_manipulation)
workspace: ROS 2 Humble, UFACTORY `xarm_ros2`, VS Code Dev Containers, and a
VNC/noVNC desktop for RViz.

1. Install a Docker-compatible engine and VS Code with Dev Containers.
2. Open this repository in VS Code.
3. Select **Dev Containers: Rebuild and Reopen in Container**.
4. In the container, run:

   ```bash
   bash /workspace/scripts/bootstrap_ros_workspace.sh
   bash /workspace/scripts/build_ros_workspace.sh
   bash /workspace/scripts/launch_gazebo_xarm.sh
   ```

5. View the graphical desktop using either method:

   - **Browser/noVNC:** open `http://localhost:6080/vnc.html?autoconnect=1&resize=scale`.
   - **TigerVNC Viewer:** connect the desktop app to `localhost:5901`.

Both connect to display `:1`, where Gazebo and RViz are launched. VS Code
should forward ports `5901` and `6080` automatically; if it does not, add them
from the **Ports** panel.

The UFACTORY `xarm_ros2` source is a Git submodule under
`ros_ws/src/xarm_ros2`. It is pinned to the exact revision referenced by
`xArm6_manipulation`; the bootstrap script initializes its nested SDK and
installs its ROS dependencies.

If Gazebo is too heavy for a member's laptop, use the lighter fake-hardware
MoveIt/RViz demo instead:

```bash
bash /workspace/scripts/launch_fake_xarm.sh
```

When cloning outside VS Code, include the submodules:

```bash
git clone --recurse-submodules <HAND repository URL>
```

## Safety boundary

- The default workflow uses either Gazebo control or fake hardware and must
  not receive a robot IP.
- Physical-arm operation requires a separate reviewed launch procedure.
- The claw is calibrated on fixtures, not fingers or other body parts.
- MyoWare is battery-powered while worn and disconnected from the user before
  USB programming or charging.
- Phase 1 contains no human-connected electrotactile stimulation circuitry.
- A custom PCB is necessary but not sufficient for any future stimulation
  system to be considered safe.
