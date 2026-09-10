# Synthetic EMG Data

`synthetic_emg_envelope.csv` is a deterministic, fictional MyoWare-style
analog-envelope recording for software onboarding. It contains 40 seconds at
50 Hz with labeled rest/flex intervals, baseline drift, measurement noise, and
two short motion artifacts during rest.

| Column | Meaning |
| --- | --- |
| `time_s` | Time from the start of the recording, in seconds |
| `envelope_v` | Synthetic analog-envelope voltage, clipped to 0–3.3 V |
| `label` | Ground truth: `rest` or `flex` |
| `event` | Segment name or `motion_artifact` |

This is **not raw EMG**, but it should suffice to teach plotting,
smoothing, calibration, thresholds, hysteresis, and error measurement.

Run `generate_synthetic_emg.py` to reproduce the CSV exactly.
