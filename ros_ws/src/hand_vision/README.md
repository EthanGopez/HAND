# `hand_vision`

Owner: Vision, Host, and Arm Control.

Reads a webcam or phone-camera stream, runs MediaPipe hand tracking, and publishes
one `HandLandmarks` message per frame for `hand_teleop`. It is the ROS version of
`vision_demos/hand_tracking_demo.py`.

## Tripwire

Tracking does not start just because a hand is in view. A palm centre has to enter
a circle in the middle of the frame first. Until then every message has
`tracked: false`, so `hand_teleop` holds the arm still. Once active, a hand anywhere
in frame keeps it active. After `lost_timeout` seconds with no hand it goes back to
waiting for the tripwire.

## Output

- Topic: `hand/landmarks`
- Type: `hand_interfaces/msg/HandLandmarks`
- Rate: one message per camera frame, capped at `rate`
- `header.stamp`: when the frame was read; `header.frame_id`: `camera`
- One hand only (MediaPipe runs with `num_hands=1`)
- Not tracked (tripwire waiting, no hand, or no frame): `tracked: false`, empty
  `handedness`, all 21 landmarks zero

## Parameters

| Parameter         | Default                                    | Meaning                                         |
|-------------------|--------------------------------------------|-------------------------------------------------|
| `source`          | `http://host.docker.internal:5000/video`   | Camera index (`0`), video file, or stream URL   |
| `model_path`      | `~/.cache/mediapipe/hand_landmarker.task`  | MediaPipe Hand Landmarker model                 |
| `mirror`          | `false`                                    | Flip frames horizontally (local webcams)        |
| `rate`            | `30.0`                                     | Max frames read per second                      |
| `tripwire_radius` | `0.15`                                     | Zone radius, fraction of the shorter frame side |
| `lost_timeout`    | `5.0`                                      | Seconds with no hand before going back to wait  |

## Run

Inside the dev container, after building the workspace:

```bash
apt-get install -y python3-pip  # if pip is missing
pip install mediapipe           # no rosdep key for it
ros2 run hand_vision vision_node
ros2 topic echo /hand/landmarks --field tracked
```

The model is downloaded the first time `vision_demos/hand_tracking_demo.py` runs.
Otherwise fetch it from the URL in the error message.

## Depends on

`hand_interfaces`, which defines `HandLandmarks`. It is on the
`ee-target-tracker-node` branch and not yet on `main`, so this package only builds
once that branch is merged.
