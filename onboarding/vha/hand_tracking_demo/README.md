# Hand Tracking Demo

This hardware-free exercise uses MediaPipe and OpenCV to detect 21 hand
landmarks from a laptop webcam, a video file, or a phone-camera stream. It is
deliberately independent of ROS: the first goal is to understand the camera
data and tracking-loss behavior.

## Setup

Use Python 3.10 or 3.11 from the repository root:

```bash
python3 -m venv .venv-vha
source .venv-vha/bin/activate
python -m pip install --upgrade pip
python -m pip install -r onboarding/vha/hand_tracking_demo/requirements.txt
```

On Windows PowerShell, activate with:

```powershell
.\.venv-vha\Scripts\Activate.ps1
```

## Run

Laptop webcam:

```bash
python onboarding/vha/hand_tracking_demo/hand_tracking_demo.py \
  --source 0 \
  --output hand_landmarks.csv
```

Prerecorded video:

```bash
python onboarding/vha/hand_tracking_demo/hand_tracking_demo.py \
  --source path/to/video.mp4 \
  --output hand_landmarks.csv
```

A phone-camera app that exposes an HTTP video URL can be used by passing that
URL to `--source`. The exact phone app is intentionally not prescribed.

The first run downloads Google's pinned Hand Landmarker model into the ignored
`models/` directory. Press `q` or Escape to stop.

## What to observe

1. Move one hand slowly and then quickly.
2. Move partly outside the frame.
3. Hide the hand completely for two seconds and bring it back.
4. Inspect `hand_landmarks.csv`.

The CSV contains one row per landmark while tracking is valid. Frames with no
detected hand receive one `tracked=false` row, making tracking loss explicit.
Coordinates `x` and `y` are normalized to the image; `z` is MediaPipe's
relative depth value and is not a metric distance.

## Completion check

- The display shows a labeled hand skeleton.
- A CSV is produced with landmark indices `0` through `20`.
- Hiding the hand produces `tracked=false` rows.
- The member can explain why this output must be calibrated, filtered, and
  bounded before it can influence a robot.

This demo never connects to ROS or the physical xArm.
