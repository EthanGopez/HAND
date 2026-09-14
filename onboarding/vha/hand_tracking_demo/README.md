# Hand Tracking Quick Demo

This hardware-free demo uses MediaPipe and OpenCV to detect 21 hand
landmarks from a laptop webcam, a video file, or a phone-camera stream. It is
deliberately independent of ROS: the first goal is to understand the camera
data and tracking-loss behavior. 

## Setup

Work through the setup steps provided in the Jupyter notebook 
`hand_tracking_onboarding.ipynb`. (you might need to restart your kernel after
cell 1 is run.)


## Run

Laptop webcam:

```bash
cd /Users/egopez/Cowork/HAND/onboarding/vha/hand_tracking_demo
source .venv/bin/activate
python hand_tracking_demo.py
```

If you want a custom output path, 
run:
```bash
python hand_tracking_demo.py --output recordings/test.csv
```



## What to observe

1. Move one hand slowly and then quickly.
2. Move partly outside the frame.
3. Hide the hand completely for two seconds and bring it back.
4. Inspect `hand_landmarks.csv`.

The CSV contains one row per landmark while tracking is valid. Frames with no
detected hand receive one `tracked=false` row, making tracking loss explicit.
Coordinates `x` and `y` are normalized to the image; `z` is MediaPipe's
relative depth value and is not a metric distance.

## Observe the following

- The display shows a labeled hand skeleton.
- A CSV is produced with landmark indices `0` through `20`.
- Hiding the hand produces `tracked=false` rows.

This demo never connects to ROS or the physical xArm.
