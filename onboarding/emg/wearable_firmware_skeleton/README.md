# Wearable Firmware Skeleton

This PlatformIO/ESP32 exercise exposes the basic shape of the wearable's
sense-decide-act loop without requiring a board, electrodes, or a vibration
motor. Each hardware-facing stage is stubbed:

- `adc_sampler` synthesizes an EMG-envelope-like waveform instead of reading
  a real ADC pin;
- `intent_classifier` buffers a window of samples and thresholds their mean
  into `REST`, `OPEN`, `CLOSE`, or `PINCH`;
- `comm_link` serializes the classified intent over USB serial instead of a
  real radio link; and
- `vibration_feedback` maps intent to a PWM duty cycle on a GPIO pin instead
  of a real haptic motor.

`main.cpp` wires these together: sample, buffer, classify, transmit, buzz —
on a fixed interval, forever.

## Compile check

1. Install [PlatformIO](https://platformio.org/) (CLI or the VS Code
   extension).
2. From this directory, run:

   ```sh
   pio run
   ```

   This builds for the `esp32dev` target and does not require a board
   connected or USB upload — `pio run` alone never touches a port.

## Code-reading exercise

Find where the program answers each question:

1. How often is a new EMG sample taken, and how many samples make up one
   classification window?
2. What turns a raw sample buffer into one of the four intent labels?
3. What would need to change in `adc_sampler.cpp` to read a real MyoWare
   envelope instead of a synthetic one?
4. What would need to change in `comm_link.cpp` to send over BLE instead of
   serial?
5. Which module(s) would the calibration/thresholding logic from the
   [`beginner_emg_analysis`](../analysis/beginner_emg_analysis.ipynb) notebook
   eventually replace?

## Hardware warning

The pin numbers, sample rate, and classification thresholds are placeholders.
A successful build does **not** authorize wiring a real vibration motor or
electrodes. When real hardware is introduced, keep the wearable
battery-powered and isolated from USB/charging while worn, per
[`firmware/wearable_controller`](../../../firmware/wearable_controller/README.md).
