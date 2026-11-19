# Notes: replacing the palm-width depth estimate with a PnP solve

**Status: not implemented, and deliberately so.** The demo keeps
`DEPTH_CONSTANT / palm width`, which is calibrated against a real tape-measured
palm and works. This note records what the alternative is, what it would buy,
and how to build it, so the decision can be revisited without redoing the
analysis. Numbers below come from `hand_landmarks.csv` in this repository
(2013 tracked frames, 640x480) and can be reproduced with
[`compare_depth_methods.py`](compare_depth_methods.py).

## What is wrong with the current estimate

`track_palms()` measures one projected distance — index MCP (landmark 5) to
pinky MCP (landmark 17) — and inverts it:

```python
palm_width = math.hypot(index_mcp[0] - pinky_mcp[0], index_mcp[1] - pinky_mcp[1])
depth_estimate = DEPTH_CONSTANT / max(palm_width, 1)
```

That distance is a *projection*. Rotate the hand about the forearm and the
projected width shrinks by cos(tilt) while the real distance to the camera has
not changed, so the estimate reads further away. In this recording the palm was
foreshortened by more than 25% in **411 of 2013 frames**, and across those
frames the estimate reads **+77%** high. Calibration cannot fix this; it is
built into using a single projected length.

A second cost: inverting one small number makes the result noisy. With the hand
held still, frame-to-frame depth noise is 1.0% median but 15.4% at p99.

## Why PnP is possible here

MediaPipe's Hand Landmarker returns two coordinate sets per hand, and the demo
already records both ([`write_result()`](hand_tracking_demo.py)):

| Output | CSV columns | What it is |
| --- | --- | --- |
| `hand_landmarks` | `pixel_x`, `pixel_y` | Where each point landed in the image |
| `hand_world_landmarks` | `world_x_m`, `world_y_m`, `world_z_m` | A metric 3D hand in metres, **origin at the hand's centre, camera translation discarded** |

The world landmarks cannot give distance on their own — that is what "translation
discarded" means, and it is measurable: the correlation between the hand's
apparent pixel size and its mean world z is **-0.10**, i.e. none.

But a metric 3D model plus its 2D projection plus a focal length is exactly the
input to a perspective-n-point solve. PnP recovers the translation that
MediaPipe threw away, using all 21 correspondences instead of one segment, and
recovers the hand's rotation at the same time — which is why foreshortening
stops being a bias and becomes something the solve accounts for.

## Two options, measured

Noise is the frame-to-frame change with the hand held still; bias is the median
reading on the 411 frames where the palm was turned away, against that method's
own overall median.

| Method | Median distance | Noise, median | Noise, p90 | Noise, p99 | Bias when rotated |
| --- | --- | --- | --- | --- | --- |
| palm-width (today) | 55.9 cm | 1.0% | 4.0% | 15.4% | **+77%** |
| scale-fit, 21 points | 38.9 cm | 0.5% | 1.9% | 4.6% | +25% |
| full PnP, 21 points | 41.2 cm | 0.5% | 2.0% | 5.1% | **+20%** |

**Option A — weak-perspective scale fit.** About six lines, no OpenCV solver.
Fit one scale `s` (pixels per metre) between the centred world x/y and the
centred pixel x/y, then `distance = focal_px / s`. This works because the world
landmarks are roughly rotated into the camera's frame, so their x/y components
foreshorten much as the pixels do and most of the rotation cancels. "Roughly" is
why this option keeps a slightly larger rotation bias than the full solve: the
residual alignment error is 13.8 degrees median (see the sanity-check section).

**Option B — full `cv2.solvePnP`.** Handles perspective properly rather than
assuming the hand is small relative to its distance, and hands back the hand's
orientation, which is useful on its own (wrist angle for the gripper, for
example). Needs `cv2.SOLVEPNP_SQPNP`, available in the installed OpenCV 4.5.4.

Both halve the median noise and cut the tail by 3x. Both cut the rotation bias
by roughly 4x. The extra cost is microseconds against MediaPipe's own inference.

## The scale catch — read this before switching

The three methods disagree on absolute distance: 55.9 cm versus about 41 cm.
That is not a bug in the solve. It is the two methods trusting different hands:

* `DEPTH_CONSTANT = 4613` encodes **your measured palm**, 8.5 cm.
* PnP inherits the scale of **MediaPipe's generic hand model**, whose index-to-pinky
  distance in this recording is 6.55 cm median.

8.5 / 6.55 = 1.30, and 55.9 / 41.2 = 1.36. The disagreement is almost entirely
that ratio. If the tape measure was right, the current estimate has the better
absolute scale and PnP would read about 30% short.

**So do not throw the calibration away.** Rescale the metric model to the real
hand before solving, and the result keeps your absolute scale *and* gains the
rotation invariance:

```python
model_palm_m = float(np.linalg.norm(obj[17] - obj[5]))
obj *= MEASURED_PALM_WIDTH_M / model_palm_m
```

Compute that correction **once** over a calibration window and hold it fixed.
The model's own palm width wobbles about 13% frame to frame on a body part that
does not change size, so applying it per frame would inject that noise straight
back into the output.

## Camera intrinsics

PnP needs a camera matrix. You already have the focal length for free: it is
implied by the calibration you did.

```
focal_px = DEPTH_CONSTANT / PALM_WIDTH_cm = 4613 / 8.5 = 542.7 px
```

That corresponds to a 61 degree horizontal field of view at 640x480, which is an
ordinary webcam, so the number is self-consistent.

```python
K = np.array([[focal_px, 0, frame_width / 2],
              [0, focal_px, frame_height / 2],
              [0, 0, 1]], dtype=float)
```

**The focal length is in pixels, so it scales with resolution.** Calibrated at
640 wide, a 1280-wide stream needs `focal_px * 2`. The demo pulls frames from
`http://host.docker.internal:5000/video`; if that source ever changes
resolution, both this and the current `DEPTH_CONSTANT` break silently. Record
the resolution alongside the constant. For anything better than a few percent,
run a real checkerboard calibration with `cv2.calibrateCamera` and keep the
distortion coefficients too.

## Implementation sketch

A drop-in replacement for the depth half of `track_palms()`. The x/y half does
not need to change; it already follows from the distance.

```python
import cv2
import numpy as np

CAMERA_FOCAL_PX = 542.7        # calibrated at 640x480 -- rescale with resolution
MEASURED_PALM_WIDTH_M = 0.085  # tape-measured index-to-pinky knuckle width


def palm_pose_m(landmarks, world_landmarks, width, height, scale_correction):
    """Palm centre in camera coordinates, in metres, or None if unsolvable."""
    obj = np.array([[p.x, p.y, p.z] for p in world_landmarks], dtype=float)
    img = np.array([[p.x * width, p.y * height] for p in landmarks], dtype=float)
    obj *= scale_correction          # generic model -> this operator's hand

    camera_matrix = np.array([[CAMERA_FOCAL_PX, 0, width / 2],
                              [0, CAMERA_FOCAL_PX, height / 2],
                              [0, 0, 1]], dtype=float)
    ok, rvec, tvec = cv2.solvePnP(obj, img, camera_matrix, None,
                                  flags=cv2.SOLVEPNP_SQPNP)
    if not ok or not np.isfinite(tvec).all() or tvec[2, 0] <= 0:
        return None                  # a failed solve is not a depth of zero
    return tuple(float(v) for v in tvec[:, 0]), rvec
```

Note that `obj` is rebuilt every frame. PnP normally takes a fixed rigid model,
but here the "model" is that frame's articulated hand, which is correct and
intended — you are solving where *this* hand pose sits, not fitting a canonical
one. The consequence is that a bad world-landmark regression corrupts the pose
directly, so keep the validity checks.

Sanity-check with **reprojection error, not with `rvec`**. The world landmarks
are only roughly camera-aligned: measured on this recording, `|rvec|` is 13.8
degrees median and 30 degrees at p95, so a near-identity rotation is not the
expected result and makes a useless test. Project the solved pose back into the
image instead:

```python
projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, None)
rms_px = float(np.sqrt(((projected.reshape(-1, 2) - img) ** 2).sum(axis=1).mean()))
```

On this recording that comes out at 6.1 px median, 10.2 px at p95, 25.6 px worst.
Rejecting above 15 px drops 1.2% of frames and catches the genuinely bad solves;
a threshold of 8 px would drop 18.5%, which is too aggressive. Recalibrate those
thresholds for your own camera — a large error across *every* frame points at
wrong intrinsics, a mirrored frame, or swapped point ordering rather than at a
bad hand.

## Gotchas

* **Mirroring.** The demo flips local-camera frames (`cv2.flip(frame, 1)`).
  Detection runs after the flip, so pixel and world landmarks agree today. If
  that order ever changes, they must still come from the same frame.
* **Principal point.** Assumed at the image centre. True for a plain webcam,
  false for a cropped, letterboxed, or digitally zoomed stream.
* **Distortion ignored.** `distCoeffs=None` is fine near the centre of a normal
  lens, not fine for a wide-angle phone camera.
* **Degenerate poses.** A hand seen exactly edge-on is close to planar and the
  solve is ill-conditioned. Consider `cv2.solvePnPRansac` if single landmarks
  occasionally fly off; with 21 points it is cheap insurance.
* **PnP reduces the rotation bias, it does not remove it.** The residual +20% on
  heavily rotated frames is the world-landmark regression itself degrading when
  the hand is hard to see. No solver fixes that.
* **Fix the units while you are in there.** `center_z` is currently centimetres
  while `center_x` and `center_y` are metres, in the same CSV row. `tvec` comes
  out in whatever unit `obj` uses, so a PnP version is naturally all metres.

## If this gets implemented

1. Re-run `compare_depth_methods.py` on a fresh recording to confirm the
   improvement holds for the new camera and operator.
2. `palm_center.csv` keeps its four columns, but `center_z` becomes metres.
   Anything that reads it needs the new scale.
3. In `ros_ws/src/hand_teleop`, `palm_teleop_node`'s `scale_x` is 0.0 today
   because depth was too noisy to differentiate — with a halved noise floor it
   becomes worth testing. `max_depth_jump` is expressed in CSV depth units and
   must be re-tuned for metres.
4. `PalmCsvReader.MAX_DEPTH` in `hand_vision` bounds depth at 1000; in metres
   that bound should drop to a few metres.
