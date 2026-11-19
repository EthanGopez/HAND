#!/usr/bin/env python3
"""Write a synthetic palm CSV (all centimetres) for replay tests with no camera.

The hand traces a slow circle in the image plane while moving toward and away
from the camera, at 25 Hz, so every axis of the arm gets exercised. It is made
up, not a recording of a real hand.

    python3 vision_demos/make_synthetic_palm_csv.py vision_demos/synthetic_cm.csv
"""

import argparse
import csv
import math


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('output')
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--radius', type=float, default=6.0, help='circle radius, cm')
    parser.add_argument('--period', type=float, default=10.0, help='seconds per circle')
    parser.add_argument('--depth-swing', type=float, default=5.0, help='+/- cm')
    args = parser.parse_args()

    rate = 25.0
    with open(args.output, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['timestamp_ms', 'center_x', 'center_y', 'center_z'])
        for i in range(int(args.seconds * rate)):
            t = i / rate
            phase = 2 * math.pi * t / args.period
            writer.writerow([
                1_000_000 + int(t * 1000),
                f'{args.radius * math.sin(phase):.4f}',
                f'{args.radius * (math.cos(phase) - 1):.4f}',
                f'{40.0 + args.depth_swing * math.sin(phase / 2):.4f}',
            ])
    print(f'Wrote {args.output}')


if __name__ == '__main__':
    main()
