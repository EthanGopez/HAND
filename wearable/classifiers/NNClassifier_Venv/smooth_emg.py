"""Stream two-channel EMG data through a trailing moving average.

Usage: python smooth_emg.py input.txt output.txt
Input/output format: grip5 0.104762,0.151502
Windows reset on label changes. Initial rows use the samples available so far.
"""

import argparse
import math
from collections import deque
from pathlib import Path


def smooth_emg(input_path, output_path, window_size=25):
    if window_size < 1:
        raise ValueError("Window size must be positive.")
    if Path(input_path).resolve() == Path(output_path).resolve():
        raise ValueError("Input and output must be different files.")

    window = deque(maxlen=window_size)
    previous_label = None

    with open(input_path, encoding="utf-8-sig") as source, open(
        output_path, "w", encoding="utf-8"
    ) as destination:
        for line_number, line in enumerate(source, start=1):
            if not line.strip():
                continue
            try:
                label, values = line.split(maxsplit=1)
                voltage1, voltage2 = map(float, values.split(","))
                if not all(map(math.isfinite, (voltage1, voltage2))):
                    raise ValueError("Nonfinite voltage")
            except ValueError as error:
                raise ValueError(
                    f"Invalid data on line {line_number}: {line.strip()!r}"
                ) from error

            # Avoid averaging samples from different grip segments together.
            if label != previous_label:
                window.clear()
                previous_label = label

            window.append((voltage1, voltage2))
            average1 = math.fsum(sample[0] for sample in window) / len(window)
            average2 = math.fsum(sample[1] for sample in window) / len(window)
            destination.write(f"{label} {average1:.6f},{average2:.6f}\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="Original text file")
    parser.add_argument("output", help="Smoothed output text file")
    parser.add_argument("--window", type=int, default=250, help="Window size (default: 25)")
    args = parser.parse_args()
    smooth_emg(args.input, args.output, args.window)
    print(f"Saved smoothed data to {args.output}")