ESP32 model package

Model: 2 RMS inputs -> 16 ReLU neurons -> 6 softmax outputs
Window: 100 sample pairs
Preprocessing: envelope (preserve signal level)
Class names: rest, power_grip, two_finger, three_finger, pointing_index, open_hand
Verified matching replay windows: 1277

1. Install Arduino IDE and ESP32 board support from Espressif Systems.
   https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html
2. LIVE analog input: open grip_classifier/grip_classifier.ino.
   Check EMG_PIN_A/B against your board's ADC pinout. Original ESP32 defaults:
   GPIO 34/35; other chips use provisional GPIO 1/2, which MUST be checked.
   Default readings are 12-bit ADC counts. Your training data must match these
   units, gain, preprocessing, sample rate and channel order.
   Sampling uses a separate FreeRTOS task at a nominal 500 Hz per channel.
   One-shot ADC reads are sequential and timing has scheduling jitter; this is
   a prototype, not a hardware-clocked acquisition system. Gaps are reported
   and incomplete windows are discarded. For raw EMG bandwidth above 250 Hz,
   choose higher-rate acquisition with appropriate anti-aliasing and change
   training/window sizes accordingly.
   Connect conditioned/amplified sensor outputs, not bare body electrodes.
   Keep inputs within your board's ADC range; use an appropriate battery-powered
   or isolated body-connected acquisition setup.
3. Select your actual ESP32 board and COM port, then upload.
4. To test matching recorded data instead, upload the SEPARATE sketch:
   grip_classifier_replay/grip_classifier_replay.ino.
   Close Serial Monitor. In your Python environment, install pyserial if needed:
   python -m pip install pyserial
5. Test the board with the generated replay.py:
   python replay.py --port COM5 --count 20
   Replace COM5 with the actual port. Use the same Python environment as training.

The PC sends sample windows; the ESP32 computes RMS, scaling and predictions.
That replay sketch uses USB input. The MAIN grip_classifier sketch reads analog
pins and only uses serial to print results. Neither sketch controls motors.
Public-data weights need retraining on your own sensor recordings for live use.
Packaging checks numerical agreement; it does not validate gesture-name mappings
or cross-session classification accuracy. Correct source labels before training.

After each retraining, run package_esp32.py again and upload the regenerated
sketch. Generated package files and the ZIP are replaced on each run. Changes
made by hand inside this generated package will be overwritten for those files.
Keep custom firmware edits in a separate project.
