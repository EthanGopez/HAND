#!/usr/bin/env python3
"""Generate deterministic, fictional EMG-envelope data for onboarding."""

from __future__ import annotations

import csv
import math
from pathlib import Path
import random


SAMPLE_RATE_HZ = 50
DURATION_S = 40
SEED = 20260909
OUTPUT_PATH = Path(__file__).with_name("synthetic_emg_envelope.csv")

FLEX_INTERVALS = (
    (6.0, 9.0, "flex_1"),
    (15.0, 18.5, "flex_2"),
    (25.0, 29.0, "flex_3"),
    (34.0, 37.0, "flex_4"),
)
ARTIFACT_CENTERS_S = (12.2, 31.5)


def flex_segment(time_s: float) -> tuple[float, str] | None:
    for start_s, end_s, name in FLEX_INTERVALS:
        if start_s <= time_s < end_s:
            ramp_up = min(1.0, (time_s - start_s) / 0.25)
            ramp_down = min(1.0, (end_s - time_s) / 0.35)
            return min(ramp_up, ramp_down), name
    return None


def artifact_amplitude(time_s: float) -> float:
    return sum(
        0.72 * math.exp(-0.5 * ((time_s - center_s) / 0.07) ** 2)
        for center_s in ARTIFACT_CENTERS_S
    )


def generate_rows() -> list[dict[str, str]]:
    rng = random.Random(SEED)
    rows = []
    for sample_index in range(SAMPLE_RATE_HZ * DURATION_S):
        time_s = sample_index / SAMPLE_RATE_HZ
        baseline = 0.38 + 0.025 * math.sin(2 * math.pi * time_s / 24.0)
        noise = rng.gauss(0.0, 0.025)
        envelope_v = baseline + noise
        label = "rest"
        event = "rest"

        segment = flex_segment(time_s)
        if segment is not None:
            ramp, event = segment
            activation = 1.02 + 0.12 * math.sin(2 * math.pi * 1.4 * time_s)
            envelope_v += ramp * activation + rng.gauss(0.0, 0.06)
            label = "flex"
        else:
            artifact = artifact_amplitude(time_s)
            envelope_v += artifact
            if artifact > 0.08:
                event = "motion_artifact"

        envelope_v = min(3.3, max(0.0, envelope_v))
        rows.append(
            {
                "time_s": f"{time_s:.2f}",
                "envelope_v": f"{envelope_v:.4f}",
                "label": label,
                "event": event,
            }
        )
    return rows


def main() -> None:
    with OUTPUT_PATH.open("w", newline="", encoding="utf-8") as output_file:
        writer = csv.DictWriter(
            output_file,
            fieldnames=("time_s", "envelope_v", "label", "event"),
        )
        writer.writeheader()
        writer.writerows(generate_rows())
    print(f"Wrote {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
