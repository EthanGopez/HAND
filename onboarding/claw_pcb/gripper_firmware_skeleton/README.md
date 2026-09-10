# Gripper Firmware Skeleton

This Arduino-ESP32 exercise exposes the basic shape of a safe local gripper
controller without requiring a board, servo, or force sensor. It includes:

- explicit `OPEN`, `CLOSE`, `STOP`, `STATUS`, and `HELP` serial commands;
- incremental motion rather than an instantaneous jump;
- local pulse-width limits;
- a force/contact threshold;
- a motion timeout; and
- a stopped state at boot.

## Compile check

1. Install Arduino IDE and **esp32 by Espressif Systems** version 3.x.
2. Open `gripper_firmware_skeleton.ino`.
3. Select **ESP32S3 Dev Module**.
4. Click **Verify**. Do not upload during the hardware-free onboarding meeting.

The sketch uses the ESP32 core's built-in LEDC PWM and ADC APIs, so no servo
library is required.

## Code-reading exercise

Find where the program answers each question:

1. What state does it enter at boot?
2. What prevents a pulse outside the configured servo range?
3. What stops closing when contact is detected?
4. What happens if motion takes too long?
5. Which values must be calibrated before hardware use?

Then make one harmless edit: change `CONTACT_THRESHOLD_MV` or
`MOTION_TIMEOUT_MS`, compile again, and explain the behavioral tradeoff.

## Hardware warning

The GPIO numbers, pulse widths, force threshold, direction, power path, and
mechanical limits are placeholders. A successful compile does **not** authorize
connecting or moving a real actuator. Servos must not be powered from the
development board's logic rail.
