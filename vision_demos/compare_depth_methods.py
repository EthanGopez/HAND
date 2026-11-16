#!/usr/bin/env python3
"""Compare depth estimators on a recording from hand_tracking_demo.py.

Reads a landmark CSV (the --output file of the demo, which stores both the
pixel landmarks and MediaPipe's metric world landmarks) and reports three
estimates of hand distance side by side:

  palm-width  what the demo does today: DEPTH_CONSTANT / pixel distance 5->17
  scale-fit   weak-perspective least-squares fit over all 21 points
  pnp         cv2.solvePnP over all 21 points

Nothing here changes the demo. It exists to measure whether a change is worth
making. See PNP_DEPTH_NOTES.md for what the numbers mean.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

# Implied by the demo's own calibration: DEPTH_CONSTANT / PALM_WIDTH in cm.
DEFAULT_FOCAL_PX = 4613.0 / 8.5
STILL_PX = 2.0        # wrist motion below this counts as "hand held still"
MAX_STEP_S = 0.1      # ignore pairs of frames further apart than this


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('csv_path', type=Path,
                        help='Landmark CSV written by hand_tracking_demo.py')
    parser.add_argument('--focal-px', type=float, default=DEFAULT_FOCAL_PX,
                        help=f'Focal length in pixels (default: {DEFAULT_FOCAL_PX:.1f})')
    parser.add_argument('--depth-constant', type=float, default=4613.0,
                        help='DEPTH_CONSTANT used by the recording (default: 4613)')
    parser.add_argument('--frame-size', type=int, nargs=2, metavar=('W', 'H'),
                        help='Frame size in pixels; inferred from the CSV if omitted')
    return parser.parse_args()


def load_frames(csv_path: Path) -> dict[int, dict[int, dict[str, str]]]:
    """Group tracked rows into {timestamp_ms: {landmark_index: row}}."""
    frames: dict[int, dict[int, dict[str, str]]] = defaultdict(dict)
    with csv_path.open(newline='') as handle:
        for row in csv.DictReader(handle):
            if row['tracked'] == 'true' and row['landmark_index']:
                frames[int(row['timestamp_ms'])][int(row['landmark_index'])] = row
    # Only frames with a complete hand can be compared across methods.
    return {ts: lm for ts, lm in sorted(frames.items()) if len(lm) == 21}


def infer_frame_size(landmarks: dict[int, dict[str, str]]) -> tuple[int, int]:
    """Recover the frame size from the normalized/pixel pair of any landmark."""
    for row in landmarks.values():
        nx, ny = float(row['normalized_x']), float(row['normalized_y'])
        if nx > 0.05 and ny > 0.05:
            return round(float(row['pixel_x']) / nx), round(float(row['pixel_y']) / ny)
    raise ValueError('Cannot infer frame size from this CSV')


def object_points(landmarks) -> np.ndarray:
    """The frame's metric hand model, in metres, from MediaPipe world landmarks."""
    return np.array([[float(landmarks[i][axis]) for axis in
                      ('world_x_m', 'world_y_m', 'world_z_m')] for i in range(21)], float)


def image_points(landmarks) -> np.ndarray:
    return np.array([[float(landmarks[i]['pixel_x']), float(landmarks[i]['pixel_y'])]
                     for i in range(21)], float)


def palm_width_depth(image, depth_constant) -> float:
    """Today's estimator: one projected distance, index MCP to pinky MCP."""
    return depth_constant / max(math.dist(image[5], image[17]), 1e-6)


def scale_fit_depth(obj, image, focal_px) -> float:
    """Weak-perspective fit: one scale s (px per metre) over all 21 points.

    The world landmarks are roughly rotated into the camera's frame, so their
    x/y components foreshorten much as the pixels do, and fitting pixels against
    them cancels most of the rotation rather than being biased by it. The
    alignment is only approximate (about 14 degrees residual on the sample
    recording), which is why this keeps a little more rotation bias than PnP.
    """
    centred_obj = obj - obj.mean(axis=0)
    centred_img = image - image.mean(axis=0)
    denominator = float((centred_obj[:, 0] ** 2 + centred_obj[:, 1] ** 2).sum())
    if denominator <= 0:
        return math.nan
    scale = float((centred_img[:, 0] * centred_obj[:, 0]
                   + centred_img[:, 1] * centred_obj[:, 1]).sum()) / denominator
    return (focal_px / scale) * 100.0 if scale > 0 else math.nan


def pnp_depth(obj, image, camera_matrix) -> tuple[float, float]:
    """Full perspective solve. Returns (depth in cm, rotation magnitude in deg)."""
    ok, rvec, tvec = cv2.solvePnP(obj, image, camera_matrix, None,
                                  flags=cv2.SOLVEPNP_SQPNP)
    if not ok:
        return math.nan, math.nan
    return float(tvec[2, 0]) * 100.0, math.degrees(float(np.linalg.norm(rvec)))


def foreshortening(obj) -> float:
    """How much of the real palm width the camera sees; 1.0 = square on."""
    span = obj[17] - obj[5]
    full = float(np.linalg.norm(span))
    return math.hypot(span[0], span[1]) / full if full > 0 else math.nan


def percentile(values, fraction) -> float:
    ordered = sorted(values)
    return ordered[int(fraction * (len(ordered) - 1))]


def main() -> None:
    args = parse_args()
    frames = load_frames(args.csv_path)
    if not frames:
        raise SystemExit(f'No tracked frames with 21 landmarks in {args.csv_path}')
    width, height = args.frame_size or infer_frame_size(next(iter(frames.values())))
    camera_matrix = np.array([[args.focal_px, 0, width / 2],
                              [0, args.focal_px, height / 2],
                              [0, 0, 1]], float)
    print(f'{len(frames)} complete frames, {width}x{height}, '
          f'focal {args.focal_px:.1f} px')

    results = []
    for timestamp, landmarks in frames.items():
        obj, image = object_points(landmarks), image_points(landmarks)
        depth, tilt = pnp_depth(obj, image, camera_matrix)
        results.append({
            'ts': timestamp,
            'wrist': image[0],
            'palm': palm_width_depth(image, args.depth_constant),
            'fit': scale_fit_depth(obj, image, args.focal_px),
            'pnp': depth,
            'tilt_deg': tilt,
            'foreshortening': foreshortening(obj),
        })

    methods = (('palm-width', 'palm'), ('scale-fit', 'fit'), ('pnp', 'pnp'))
    print('\nmedian reported distance')
    for name, key in methods:
        print(f'  {name:<11} {statistics.median([r[key] for r in results]):6.1f} cm')

    # Noise: consecutive frames where the hand did not move in the image.
    noise = {key: [] for _, key in methods}
    for first, second in zip(results, results[1:]):
        step = (second['ts'] - first['ts']) / 1000.0
        if not 0 < step < MAX_STEP_S:
            continue
        if math.dist(first['wrist'], second['wrist']) > STILL_PX:
            continue
        for _, key in methods:
            if first[key] > 0 and math.isfinite(first[key]) and math.isfinite(second[key]):
                noise[key].append(abs(second[key] - first[key]) / first[key])
    sample = len(noise['palm'])
    print(f'\nframe-to-frame noise with the hand held still (n={sample})')
    for name, key in methods:
        print(f'  {name:<11} median {100*statistics.median(noise[key]):5.1f}%'
              f'   p90 {100*percentile(noise[key], 0.90):6.1f}%'
              f'   p99 {100*percentile(noise[key], 0.99):7.1f}%')

    # Bias: frames where the palm is turned away from the camera.
    rotated = [r for r in results if r['foreshortening'] < 0.75]
    print(f'\nframes with the palm foreshortened by more than 25%: '
          f'{len(rotated)} of {len(results)}')
    for name, key in methods:
        overall = statistics.median([r[key] for r in results])
        turned = statistics.median([r[key] for r in rotated])
        print(f'  {name:<11} {turned:6.1f} cm there vs {overall:6.1f} cm overall'
              f'   ({100*(turned/overall-1):+.0f}%)')


if __name__ == '__main__':
    main()
