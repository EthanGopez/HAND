#!/usr/bin/env python3
"""Display and record MediaPipe hand landmarks from an ordinary camera."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys
import time
from typing import Any
from urllib.request import urlopen


MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/1/hand_landmarker.task"
)
DEFAULT_MODEL = Path(__file__).parent / "models" / "hand_landmarker.task"

# MediaPipe's 21-landmark hand topology.
HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (0, 17), (17, 18), (18, 19), (19, 20),
)

CSV_FIELDS = (
    "frame_index",
    "timestamp_ms",
    "tracked",
    "hand_index",
    "handedness",
    "handedness_score",
    "landmark_index",
    "x",
    "y",
    "z",
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
        help="Optional CSV path for recorded landmarks and tracking-loss rows.",
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_MODEL,
        help="Path to the MediaPipe Hand Landmarker task model.",
    )
    parser.add_argument("--num-hands", type=int, default=1, choices=(1, 2))
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


def draw_result(frame: Any, result: Any, cv2: Any) -> None:
    height, width = frame.shape[:2]
    for hand_index, landmarks in enumerate(result.hand_landmarks):
        points = [
            (int(landmark.x * width), int(landmark.y * height))
            for landmark in landmarks
        ]
        for start, end in HAND_CONNECTIONS:
            cv2.line(frame, points[start], points[end], (80, 220, 120), 2)
        for landmark_index, point in enumerate(points):
            cv2.circle(frame, point, 4, (40, 90, 255), -1)
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
        anchor = points[0]
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


def write_result(
    writer: csv.DictWriter[str] | None,
    result: Any,
    frame_index: int,
    timestamp_ms: int,
) -> None:
    if writer is None:
        return
    if not result.hand_landmarks:
        writer.writerow(
            {
                "frame_index": frame_index,
                "timestamp_ms": timestamp_ms,
                "tracked": "false",
                "hand_index": -1,
            }
        )
        return

    for hand_index, landmarks in enumerate(result.hand_landmarks):
        handedness = result.handedness[hand_index][0]
        for landmark_index, landmark in enumerate(landmarks):
            writer.writerow(
                {
                    "frame_index": frame_index,
                    "timestamp_ms": timestamp_ms,
                    "tracked": "true",
                    "hand_index": hand_index,
                    "handedness": handedness.category_name,
                    "handedness_score": f"{handedness.score:.6f}",
                    "landmark_index": landmark_index,
                    "x": f"{landmark.x:.8f}",
                    "y": f"{landmark.y:.8f}",
                    "z": f"{landmark.z:.8f}",
                }
            )


def run(args: argparse.Namespace) -> int:
    try:
        import cv2
        import mediapipe as mp
    except ImportError as error:
        print(
            "Missing dependency. Activate the VHA environment and install "
            "requirements.txt.\n" + str(error),
            file=sys.stderr,
        )
        return 2

    ensure_model(args.model, allow_download=not args.no_download)
    source = resolve_source(args.source)
    capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        print(f"Could not open video source: {args.source}", file=sys.stderr)
        return 2

    output_file = None
    writer = None
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        output_file = args.output.open("w", newline="", encoding="utf-8")
        writer = csv.DictWriter(output_file, fieldnames=CSV_FIELDS)
        writer.writeheader()

    options = mp.tasks.vision.HandLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path=str(args.model)),
        running_mode=mp.tasks.vision.RunningMode.VIDEO,
        num_hands=args.num_hands,
        min_hand_detection_confidence=0.5,
        min_hand_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    is_local_camera = isinstance(source, int)
    frame_index = 0
    last_timestamp_ms = -1

    try:
        with mp.tasks.vision.HandLandmarker.create_from_options(options) as landmarker:
            while True:
                ok, frame = capture.read()
                if not ok:
                    break
                if is_local_camera and not args.no_mirror:
                    frame = cv2.flip(frame, 1)

                timestamp_ms = max(last_timestamp_ms + 1, time.monotonic_ns() // 1_000_000)
                last_timestamp_ms = timestamp_ms
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
                result = landmarker.detect_for_video(mp_image, timestamp_ms)

                write_result(writer, result, frame_index, timestamp_ms)
                if not args.no_display:
                    draw_result(frame, result, cv2)
                    status = "TRACKED" if result.hand_landmarks else "NO HAND"
                    color = (80, 220, 120) if result.hand_landmarks else (40, 80, 255)
                    cv2.putText(
                        frame,
                        status,
                        (16, 32),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        color,
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.imshow("HAND MediaPipe onboarding — q to quit", frame)
                    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                        break

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
    if args.output:
        print(f"Wrote landmark data to {args.output}.")
    return 0


def main() -> int:
    return run(parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
