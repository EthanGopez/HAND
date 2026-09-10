# Contributing to HAND

## Branches and reviews

- Use short branches such as `vha/camera-benchmark`, `claw/jaw-concept`, or
  `emg/threshold-notebook`.
- Keep generated ROS directories (`build`, `install`, and `log`) out of Git.
- Do not commit credentials, robot IP addresses, participant data, or large
  recordings.
- Use one owner at a time for a KiCad board layout; its design files are
  difficult to merge safely.
- Use Onshape versions and links rather than exporting a new CAD file after
  every edit.

## Hardware boundary

The repository begins in simulation and offline-data mode. No script should
connect to a physical xArm by default. Physical-arm launch and deployment
procedures must live behind an explicit, reviewed workflow.

