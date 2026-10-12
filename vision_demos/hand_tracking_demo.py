#!/usr/bin/env python3
"""Track hands in live video and record MediaPipe Tasks landmarks to CSV."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys
import time
from typing import Any
from urllib.request import urlopen
from collections import deque
import math

EMA_ALPHA = 0.8  # exponential moving average for smoothing landmark positions
# basically, constant is how much of the current frame's position is used,
# and 1-constant is how much of the previous frame's position is used
filtered_landmarks = {}
# this is a dictionary that will hold the filtered landmark positions for each hand index
trails = {}
# this is just a dictionary that will hold the trail of the palm center for each hand index, 
# for visual purposes so i know where the hand has been over the last few frames, and so that 
# i know the palm center is only tracking one hand when both are on screen

MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/latest/hand_landmarker.task"
)
DEFAULT_MODEL = Path.home() / ".cache" / "mediapipe" / "hand_landmarker.task"
DEFAULT_OUTPUT = Path("hand_landmarks.csv")

CSV_FIELDS = (
    "frame_index",
    "timestamp_ms",
    "tracked",
    "hand_index",
    "handedness",
    "handedness_score",
    "landmark_index",
    "normalized_x",
    "normalized_y",
    "normalized_z",
    "pixel_x",
    "pixel_y",
    "world_x_m",
    "world_y_m",
    "world_z_m",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        default="0",
        help="Camera index, video path, or phone-camera stream URL (default: 0).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"CSV destination (default: {DEFAULT_OUTPUT}).",
    )
    parser.add_argument(
        "--no-output",
        action="store_true",
        help="Run without recording a CSV file.",
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_MODEL,
        help="Path to the MediaPipe Hand Landmarker task model.",
    )
    parser.add_argument("--num-hands", type=int, default=2, choices=(1, 2))
    parser.add_argument(
        "--no-download",
        action="store_true",
        help="Fail instead of downloading the pinned model when it is missing.",
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Process without opening a window; useful for prerecorded tests.",
    )
    parser.add_argument(
        "--no-mirror",
        action="store_true",
        help="Do not mirror frames from a local camera.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        help="Stop after this many frames; useful for a quick smoke test.",
    )
    return parser.parse_args()


def resolve_source(value: str) -> int | str:
    """Treat an all-digit source as a local camera index."""
    return int(value) if value.isdigit() else value


def ensure_model(model_path: Path, allow_download: bool) -> None:
    """Download the same Hand Landmarker model used by the notebook."""
    if model_path.is_file():
        return
    if not allow_download:
        raise FileNotFoundError(
            f"MediaPipe model not found at {model_path}. Remove --no-download "
            "or place the model there."
        )

    model_path.parent.mkdir(parents=True, exist_ok=True)
    partial_path = model_path.with_suffix(model_path.suffix + ".part")
    print(f"Downloading MediaPipe model to {model_path} ...")
    try:
        with urlopen(MODEL_URL, timeout=60) as response, partial_path.open("wb") as file:
            while chunk := response.read(1024 * 1024):
                file.write(chunk)
        partial_path.replace(model_path)
    finally:
        if partial_path.exists():
            partial_path.unlink()

'''
def open_capture(source: int | str, cv2: Any) -> Any:
    """Open a video source, selecting AVFoundation for a macOS webcam."""
    if sys.platform == "darwin" and isinstance(source, int):
        # return cv2.VideoCapture(source, cv2.CAP_AVFOUNDATION)
        return cv2.VideoCapture('http://host.docker.internal:5000/video')
    return cv2.VideoCapture(source)
'''
# this is the new video capture source that i fixed sorta?
def open_capture(source, cv2):
    return cv2.VideoCapture('http://10.150.16.215:5000/video')

def warm_up_camera(capture: Any) -> bool:
    """Discard initial webcam frames while exposure and focus settle."""
    time.sleep(1)
    received_image = False
    for _ in range(30):
        ok, frame = capture.read()
        if ok and frame is not None and frame.size > 0 and frame.max() > 0:
            received_image = True
        time.sleep(0.03)
    return received_image

# this is where i started messing with shit
# 90% of what i wrote is comments tbh
# holy yap

def draw_result(frame: Any, result: Any, vision: Any, cv2: Any) -> None:
    """Draw the Tasks API landmarks and connections on one video frame."""
    height, width = frame.shape[:2]
    for hand_index, landmarks in enumerate(result.hand_landmarks):
        points = [
            (int(landmark.x * width), int(landmark.y * height))
            for landmark in landmarks
        ]
        global filtered_landmarks

        # had to swap all the point stuff to filtered_landmarks instead of points, 
        # because points is just the current frame's landmarks, and filtered_landmarks 
        # is the EMA smoothed version of the landmarks, which is what i want to use for 
        # drawing and calculating palm center and depth estimate and stuff.
        # that was hell :)
        if hand_index not in filtered_landmarks:
            filtered_landmarks[hand_index] = points.copy()
        else:
            filtered_landmarks[hand_index] = [
                (
                    # ema stuff
                    # current frame is weighted EMA_ALPHA, 
                    # previous frame is weighted 1-EMA_ALPHA
                    int(
                        EMA_ALPHA * current[0]
                        + (1 - EMA_ALPHA) * previous[0]
                    ),
                    int(
                        EMA_ALPHA * current[1]
                        + (1 - EMA_ALPHA) * previous[1]
                    ),
                )
                for current, previous in zip(
                    points,
                    filtered_landmarks[hand_index],
                )
            ]
        # calculate center of palm using these landmarks
        palm_indices = [0, 1, 5, 9, 13, 17]

        # these are calculated by adding x/y coords, then dividing by len(num of coords)
        # set x value of palm center
        center_x = int(
            sum(filtered_landmarks[hand_index][i][0] for i in palm_indices)
            / len(palm_indices)
        )
        # set y value of palm center
        center_y = int(
            sum(filtered_landmarks[hand_index][i][1] for i in palm_indices)
            / len(palm_indices)
        )
        # draw a circle at the palm center (green because green is good)
        cv2.circle(
            frame,
            (center_x, center_y),
            10,
            (0, 255, 0),
            -1,
        )
        if hand_index not in trails:
            trails[hand_index] = deque(maxlen=30)
        trails[hand_index].append((center_x, center_y))
        for i in range(1, len(trails[hand_index])):
            cv2.line(
                frame,
                trails[hand_index][i - 1],
                trails[hand_index][i],
                (255,255,0),
                2,
            )
        # these are the knuckles
        index_mcp = filtered_landmarks[hand_index][5]
        pinky_mcp = filtered_landmarks[hand_index][17]


        # maybe add a constant for palm width to better compute the
        # depth estimate. using real units for that may help to
        # know how far the hand is in real units.

        # this is how the width of the palm is calculated
        # (distance formula between two points)
        palm_width = math.sqrt(
            #index_mcp[0] is x value, while index_mcp[1] is y value, same for pinky_mcp
            # mcp stands for metacarpophalangeal joint, which is the knuckle joint
            (index_mcp[0] - pinky_mcp[0])**2 +
            (index_mcp[1] - pinky_mcp[1])**2
        )
        # this is a rough estimate of depth based on the width of the palm in pixels
        # the larger the palm width, the closer the hand is to the camera, and vice versa
        # useful bc if hand is far away, points are close together, which distorts the actual 
        # x, y, and z values of the landmarks, so if the hand is far the robot would think
        # the hand is shrinking or something, and if the hand is close the robot would think 
        # the hand is growing. using the depth as a multiplier, the xyz scales with the depth,
        # making the xyz more accurate to the actual size of the hand
        depth_estimate = 1000.0 / max(palm_width, 1)
        # note for later, i wanna do some testing to see what the depth estimate is at 
        # different distances from the camera, and then use that to make a more accurate 
        # depth estimate, using actual units like centimeters or inches, instead of 
        # just a rough estimate based on palm width in pixels.
        cv2.putText(
            frame,
            f"Depth: {depth_estimate:.1f}",
            (center_x + 20, center_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255,255,0),
            2,
        )
        # edited this to use filtered_landmarks instead of points, reduce jitter and such
        for connection in vision.HandLandmarksConnections.HAND_CONNECTIONS:
            cv2.line(
                frame,
                filtered_landmarks[hand_index][connection.start],
                filtered_landmarks[hand_index][connection.end],
                (80, 220, 80),
                2,
            )
        # side note fuck java this is so much better oml
        for landmark_index, point in enumerate(
            filtered_landmarks[hand_index]
        ):
            cv2.circle(frame, point, 4, (30, 30, 255), -1)
            if landmark_index in (0, 4, 8, 12, 16, 20):
                cv2.putText(
                    frame,
                    str(landmark_index),
                    (point[0] + 5, point[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

        handedness = result.handedness[hand_index][0]
        label = f"{handedness.category_name} {handedness.score:.2f}"
        anchor = filtered_landmarks[hand_index][0]
        cv2.putText(
            frame,
            label,
            (anchor[0], max(20, anchor[1] - 18)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (80, 220, 120),
            2,
            cv2.LINE_AA,
        )
    # didn't really mess with anything after this

def write_result(
    writer: csv.DictWriter[str] | None,
    result: Any,
    frame_index: int,
    timestamp_ms: int,
    frame_width: int,
    frame_height: int,
) -> int:
    """Write one row per landmark and return the number of rows written."""
    if writer is None:
        return 0
    if not result.hand_landmarks:
        writer.writerow(
            {
                "frame_index": frame_index,
                "timestamp_ms": timestamp_ms,
                "tracked": "false",
                "hand_index": -1,
            }
        )
        return 1

    rows_written = 0
    for hand_index, landmarks in enumerate(result.hand_landmarks):
        handedness = result.handedness[hand_index][0]
        world_landmarks = result.hand_world_landmarks[hand_index]
        for landmark_index, (landmark, world_landmark) in enumerate(
            zip(landmarks, world_landmarks)
        ):
            writer.writerow(
                {
                    "frame_index": frame_index,
                    "timestamp_ms": timestamp_ms,
                    "tracked": "true",
                    "hand_index": hand_index,
                    "handedness": handedness.category_name,
                    "handedness_score": f"{handedness.score:.6f}",
                    "landmark_index": landmark_index,
                    "normalized_x": f"{landmark.x:.8f}",
                    "normalized_y": f"{landmark.y:.8f}",
                    "normalized_z": f"{landmark.z:.8f}",
                    "pixel_x": int(landmark.x * frame_width),
                    "pixel_y": int(landmark.y * frame_height),
                    "world_x_m": f"{world_landmark.x:.8f}",
                    "world_y_m": f"{world_landmark.y:.8f}",
                    "world_z_m": f"{world_landmark.z:.8f}",
                }
            )
            rows_written += 1

    return rows_written


def first_index_tip_text(result: Any) -> str:
    """Summarize landmark 8 so data collection is visible while running."""
    if not result.hand_landmarks:
        return "Index tip: --"
    tip = result.hand_landmarks[0][8]
    return f"Index tip: x={tip.x:.3f} y={tip.y:.3f} z={tip.z:.3f}"


def draw_status(
    frame: Any,
    result: Any,
    fps: float,
    csv_rows: int,
    recording: bool,
    cv2: Any,
) -> None:
    """Show tracking and recording state in the live preview."""
    tracked = bool(result.hand_landmarks)
    status = f"{'TRACKED' if tracked else 'NO HAND'} | {fps:.1f} FPS"
    color = (80, 220, 120) if tracked else (40, 80, 255)
    recording_text = f"CSV rows: {csv_rows}" if recording else "CSV recording: off"

    lines = (
        (status, color),
        (first_index_tip_text(result), (255, 255, 255)),
        (recording_text, (255, 255, 255)),
        ("Press q or Esc to stop", (200, 200, 200)),
    )
    for line_index, (text, text_color) in enumerate(lines):
        cv2.putText(
            frame,
            text,
            (16, 32 + line_index * 27),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            text_color,
            2,
            cv2.LINE_AA,
        )


def run(args: argparse.Namespace) -> int:
    try:
        import cv2
        import mediapipe as mp
        from mediapipe.tasks import python
        from mediapipe.tasks.python import vision
    except ImportError as error:
        print(
            "Missing dependency. Activate the hand-tracking .venv and run "
            "`python -m pip install -r requirements.txt`.\n" + str(error),
            file=sys.stderr,
        )
        return 2

    ensure_model(args.model, allow_download=not args.no_download)
    source = resolve_source(args.source)
    is_local_camera = isinstance(source, int)
    capture = open_capture(source, cv2)
    if not capture.isOpened():
        print(f"Could not open video source: {args.source}", file=sys.stderr)
        return 2

    if is_local_camera and not warm_up_camera(capture):
        capture.release()
        print(
            "The webcam opened but returned only empty or black frames. "
            "Check camera permissions and close other camera apps.",
            file=sys.stderr,
        )
        return 2

    output_path = None if args.no_output else args.output
    output_file = None
    writer = None
    if output_path is not None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_file = output_path.open("w", newline="", encoding="utf-8")
        writer = csv.DictWriter(output_file, fieldnames=CSV_FIELDS)
        writer.writeheader()
        output_file.flush()

    options = vision.HandLandmarkerOptions(
        base_options=python.BaseOptions(model_asset_path=str(args.model)),
        running_mode=vision.RunningMode.VIDEO,
        num_hands=args.num_hands,
        min_hand_detection_confidence=0.5,
        min_hand_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    frame_index = 0
    csv_rows = 0
    last_timestamp_ms = -1
    previous_frame_time = time.perf_counter()
    smoothed_fps = 0.0

    print(f"MediaPipe {mp.__version__}; model: {args.model}")
    if output_path is not None:
        print(f"Recording live landmark rows to: {output_path.resolve()}")
    print("Press q or Esc in the video window to stop.")

    try:
        with vision.HandLandmarker.create_from_options(options) as landmarker:
            while True:
                ok, frame = capture.read()
                if not ok or frame is None:
                    break
                if is_local_camera and not args.no_mirror:
                    frame = cv2.flip(frame, 1)

                timestamp_ms = max(
                    last_timestamp_ms + 1,
                    time.monotonic_ns() // 1_000_000,
                )
                last_timestamp_ms = timestamp_ms
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(
                    image_format=mp.ImageFormat.SRGB,
                    data=rgb_frame,
                )
                result = landmarker.detect_for_video(mp_image, timestamp_ms)

                height, width = frame.shape[:2]
                csv_rows += write_result(
                    writer,
                    result,
                    frame_index,
                    timestamp_ms,
                    width,
                    height,
                )
                if output_file is not None:
                    # Let another program inspect the CSV while capture continues.
                    output_file.flush()

                now = time.perf_counter()
                instantaneous_fps = 1.0 / max(now - previous_frame_time, 1e-9)
                previous_frame_time = now
                smoothed_fps = (
                    instantaneous_fps
                    if smoothed_fps == 0.0
                    else 0.9 * smoothed_fps + 0.1 * instantaneous_fps
                )

                if not args.no_display:
                    draw_result(frame, result, vision, cv2)
                    draw_status(
                        frame,
                        result,
                        smoothed_fps,
                        csv_rows,
                        output_path is not None,
                        cv2,
                    )
                    cv2.imshow("HAND MediaPipe Tasks live tracking", frame)
                    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                        break

                if frame_index % 30 == 0:
                    print(
                        f"frame={frame_index:<6} "
                        f"hands={len(result.hand_landmarks)} "
                        f"csv_rows={csv_rows:<8} "
                        f"{first_index_tip_text(result)}"
                    )

                frame_index += 1
                if args.max_frames is not None and frame_index >= args.max_frames:
                    break
    finally:
        capture.release()
        if output_file is not None:
            output_file.close()
        if not args.no_display:
            cv2.destroyAllWindows()

    print(f"Processed {frame_index} frames.")
    if output_path is not None:
        print(f"Wrote {csv_rows} CSV rows to {output_path.resolve()}.")
    return 0


def main() -> int:
    return run(parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
