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



## Jitter smoothing (One Euro filter)

Landmarks are smoothed with a One Euro filter before they are drawn or written
to the CSV. It is exponential smoothing whose strength adapts to speed: a
still hand is heavily smoothed to remove jitter, while a fast-moving hand is
lightly smoothed so it does not lag. Each landmark's normalized and world
`x`, `y`, `z` gets its own filter, per hand (keyed by Left/Right). A hand's
filters reset when it leaves the frame.

```bash
python hand_tracking_demo.py --no-smoothing           # raw MediaPipe output, for comparison
python hand_tracking_demo.py --min-cutoff 0.5         # still hand jitters -> lower this
python hand_tracking_demo.py --beta 20                # fast motion lags -> raise this
```

Tune `--min-cutoff` first with your hand held still, then raise `--beta`
until quick movements stop trailing behind.

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
