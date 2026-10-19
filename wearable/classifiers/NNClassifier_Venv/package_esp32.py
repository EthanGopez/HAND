"""Package the model exported by train.py as an Arduino ESP32 sketch.

Put this script beside train.py and run it after each successful training run:
    python package_esp32.py

It reads firmware/grip_classifier/model_data.h and output/replay_test.npz.
It creates a live analog sketch, a separate USB replay sketch, and a ZIP.
It does not flash a board, retrain a model, or correct source gesture labels.
Requires NumPy, which your training environment already contains.
"""
import argparse
import ast
import json
import re
import shutil
import sys
import zipfile
from pathlib import Path
import numpy as np

# Firmware and replay templates are embedded so this is one standalone file.
SKETCH = r'''// Replay-first sketch. Receives one WINDOW-sample two-channel window over USB.
// Later replace the serial sample source with a timed sensor acquisition source.
// Data acquisition/collection firmware should remain a separate sketch.
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "inference.h"

float samples[WINDOW][2];
int count=0;
char line[96];
int used=0;
bool overflow=false;

void consumeLine() {
  line[used]='\0';
  if (strcmp(line,"RESET")==0) { count=0; Serial.println("READY"); return; }
  float a,b; char extra;
  if (sscanf(line,"%f,%f %c",&a,&b,&extra)!=2 || !isfinite(a) || !isfinite(b)) {
    count=0; Serial.println("ERROR"); return;
  }
  samples[count][0]=a; samples[count][1]=b; ++count;
  if (count==WINDOW) {
    float feature[2], probability[CLASSES];
    extractFeatures(samples,feature);
    const int label=predictGrip(feature,probability);
    Serial.print("RESULT,"); Serial.print(label);
    for(int k=0;k<CLASSES;++k) { Serial.print(','); Serial.print(probability[k],7); }
    Serial.println(); count=0;
  }
}
void setup() { Serial.begin(115200); Serial.println("READY"); }
void loop() {
  while(Serial.available()) {
    char c=Serial.read();
    if(c=='\r') continue;
    if(c=='\n') {
      if(overflow) {count=0;Serial.println("ERROR");} else consumeLine();
      used=0; overflow=false;
    } else if(used<sizeof(line)-1 && !overflow) line[used++]=c;
    else overflow=true;
  }
}
'''
LIVE_SKETCH = r'''// Live two-channel analog classifier. Serial is OUTPUT only.
// Requires matching two-channel RMS training and a generated model_data.h.
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "inference.h"

static constexpr int EMG_PIN_A = 14;
static constexpr int EMG_PIN_B = 13;
static constexpr uint32_t SAMPLE_RATE_HZ = 500; // Per channel; match training!
static constexpr uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE_HZ;
static constexpr int HOP_SAMPLES = 25; // New prediction every 50 ms at 500 Hz.
// Your OWN training recordings must use the same pins/order, units,
// ADC resolution/attenuation, sample rate and raw/envelope processing.
// Public-data weights are not calibrated to your live sensor/ADC setup.
static_assert(NN_INPUTS == 2, "This live sketch requires two RMS inputs.");
static_assert(1000 % SAMPLE_RATE_HZ == 0, "Choose a tick-representable sample period.");

struct WindowMessage {
  float samples[NN_WINDOW][2];
  uint32_t sequence;
  uint32_t detectedGaps;
};
QueueHandle_t windowQueue = nullptr;

float readChannel(int pin) {
  return float(analogRead(pin));
}

void samplingTask(void*) {
  // This task owns its ring buffer, so inference cannot change its samples.
  float ring[NN_WINDOW][2];
  WindowMessage message;
  int writeIndex = 0, filled = 0, sincePrediction = 0;
  uint32_t previousMicros = 0, sequence = 0, gaps = 0;
  bool havePrevious = false;
  const TickType_t periodTicks = pdMS_TO_TICKS(1000 / SAMPLE_RATE_HZ);
  TickType_t wakeTime = xTaskGetTickCount();

  while (true) {
    uint32_t now = micros();
    if (havePrevious && uint32_t(now - previousMicros) > SAMPLE_PERIOD_US * 3 / 2) {
      // Discard an incomplete window after a substantial timing gap.
      // Do not insert invented samples or burst-read to catch up.
      ++gaps;
      filled = 0; writeIndex = 0; sincePrediction = 0;
      wakeTime = xTaskGetTickCount();
    }
    havePrevious = true; previousMicros = now;
    // ADC reads are sequential, not exactly simultaneous.
    ring[writeIndex][0] = readChannel(EMG_PIN_A);
    ring[writeIndex][1] = readChannel(EMG_PIN_B);
    writeIndex = (writeIndex + 1) % NN_WINDOW;
    if (filled < NN_WINDOW) ++filled;
    ++sincePrediction;

    if (filled == NN_WINDOW && sincePrediction >= HOP_SAMPLES) {
      sincePrediction = 0;
      for (int t = 0; t < NN_WINDOW; ++t) {
        int index = (writeIndex + t) % NN_WINDOW;
        message.samples[t][0] = ring[index][0];
        message.samples[t][1] = ring[index][1];
      }
      message.sequence = ++sequence;
      message.detectedGaps = gaps;
      // Keep only the latest complete window if inference falls behind.
      xQueueOverwrite(windowQueue, &message);
    }
    vTaskDelayUntil(&wakeTime, periodTicks);
  }
}

void setup() {
  Serial.begin(9600);
  if (pdMS_TO_TICKS(1000 / SAMPLE_RATE_HZ) < 1) {
    Serial.println("ERROR: FreeRTOS tick is too slow for the requested sample rate.");
    while (true) delay(1000);
  }
  windowQueue = xQueueCreate(1, sizeof(WindowMessage));
  if (windowQueue == nullptr) {
    Serial.println("ERROR: could not allocate the sample-window queue.");
    while (true) delay(1000);
  }
  if (xTaskCreate(samplingTask, "emg_sample", 8192, nullptr, 2, nullptr) != pdPASS) {
    Serial.println("ERROR: could not start sampling task.");
    while (true) delay(1000);
  }
}

void loop() {
  WindowMessage message;
  if (xQueueReceive(windowQueue, &message, pdMS_TO_TICKS(10)) == pdTRUE) {
    float feature[2], probability[NN_CLASSES];
    extractFeatures(message.samples, feature);
    int label = predictGrip(feature, probability);
    // Sampling runs independently of inference and USB printing.
    Serial.printf("%lu,%s,score=%.4f,RMS_A=%.3f,RMS_B=%.3f,gaps=%lu\n",
                  (unsigned long)message.sequence, NN_CLASS_NAMES[label], probability[label],
                  feature[0], feature[1], (unsigned long)message.detectedGaps);
  }
}
'''

INFERENCE = r'''#pragma once
#include <math.h>
#include "model_data.h"

inline void extractFeatures(const float samples[WINDOW][2], float feature[2]) {
  for (int c=0; c<2; ++c) {
    float mean=0.0f, sum=0.0f;
    if (CENTER_RAW) {
      for (int t=0; t<WINDOW; ++t) mean+=samples[t][c];
      mean/=WINDOW;
    }
    for (int t=0; t<WINDOW; ++t) {
      float v=samples[t][c]-mean;
      sum+=v*v;
    }
    feature[c]=sqrtf(sum/WINDOW);
  }
}

inline int predictGrip(const float feature[2], float probability[CLASSES]) {
  float x[INPUTS], h[HIDDEN];
  for (int i=0; i<INPUTS; ++i) x[i]=(feature[i]-MEAN[i])/SCALE[i];
  for (int j=0; j<HIDDEN; ++j) {
    float z=B1[j];
    for (int i=0; i<INPUTS; ++i) z+=x[i]*W1[i][j];
    h[j]=fmaxf(0.0f,z);
  }
  for (int k=0; k<CLASSES; ++k) {
    probability[k]=B2[k];
    for (int j=0; j<HIDDEN; ++j) probability[k]+=h[j]*W2[j][k];
  }
  int best=0;
  for (int k=1; k<CLASSES; ++k) if (probability[k]>probability[best]) best=k;
  const float maximum=probability[best];
  float total=0.0f;
  for (int k=0; k<CLASSES; ++k) { probability[k]=expf(probability[k]-maximum); total+=probability[k]; }
  for (int k=0; k<CLASSES; ++k) probability[k]/=total;
  return best;
}
'''
REPLAY = r'''"""Send held-out DATASET samples to the ESP32 and compare PC/board probabilities."""
import argparse
import time
import numpy as np
import serial
from pathlib import Path

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--port',required=True,help='e.g. COM5')
p.add_argument('--count',type=int,default=10)
a=p.parse_args()
data=np.load(Path(__file__).resolve().parent/'output/replay_test.npz')
with serial.Serial(a.port,115200,timeout=.2,write_timeout=5) as s:
    time.sleep(2)
    s.reset_input_buffer(); s.write(b'RESET\n')
    deadline=time.monotonic()+5
    while s.readline().strip()!=b'READY':
        if time.monotonic()>deadline: raise TimeoutError('Board not ready. Close Serial Monitor and verify the sketch/port.')
    for i in range(min(a.count,len(data['windows']))):
        for row in data['windows'][i]:
            s.write(f'{row[0]:.9g},{row[1]:.9g}\n'.encode())
            time.sleep(.002)  # replay pacing; not a sensor sampling timer
        deadline=time.monotonic()+5
        while True:
            line=s.readline().decode(errors='replace').strip()
            if line=='ERROR': raise RuntimeError('Board rejected a sample.')
            if line.startswith('RESULT,'): break
            if time.monotonic()>deadline: raise TimeoutError('No prediction received.')
        fields=line.split(','); board=int(fields[1]); probs=np.array(fields[2:],dtype=float)
        expected=int(data['predictions'][i])
        error=np.max(np.abs(probs-data['probabilities'][i]))
        print(f'Window {i}: true={data["labels"][i]}, PC={expected}, ESP32={board}, probability error={error:.2g}')
        if board!=expected or error>1e-4: raise AssertionError('PC/ESP32 mismatch: check that model_data.h matches this training run.')
print('Replay passed. This verifies deployment parity, not live-sensor accuracy.')
'''


def read_model(path):
    """Read the numeric header format generated by our train.py."""
    text = path.read_text(encoding='utf-8')
    dims = {}
    for name in ('INPUTS', 'HIDDEN', 'CLASSES', 'WINDOW'):
        match = re.search(r'\b' + name + r'\s*=\s*(\d+)', text)
        if not match:
            raise ValueError(f'Model header is missing {name}. Use the header exported by train.py.')
        dims[name] = int(match.group(1))
    if dims['INPUTS'] != 2:
        raise ValueError('This firmware supports exactly two RMS inputs.')
    if dims['HIDDEN'] < 1 or dims['CLASSES'] < 2 or dims['WINDOW'] < 1:
        raise ValueError('Invalid model dimensions.')
    match = re.search(r'\bCENTER_RAW\s*=\s*(true|false)\s*;', text)
    if not match:
        raise ValueError('Model header is missing CENTER_RAW.')
    center = match.group(1) == 'true'
    arrays = {}
    shapes = {'MEAN': (2,), 'SCALE': (2,), 'W1': (2, dims['HIDDEN']),
              'B1': (dims['HIDDEN'],), 'W2': (dims['HIDDEN'], dims['CLASSES']),
              'B2': (dims['CLASSES'],)}
    for name, shape in shapes.items():
        match = re.search(r'\b' + name + r'\s*((?:\[\d+\])+)' + r'\s*=\s*(\{.*?\})\s*;', text, re.S)
        if not match:
            raise ValueError(f'Cannot read exported array {name}.')
        declared = tuple(map(int, re.findall(r'\d+', match.group(1))))
        numeric = re.sub(r'(?<=[\d.])[fF]\b', '', match.group(2))
        numeric = numeric.replace('{', '[').replace('}', ']')
        array = np.asarray(ast.literal_eval(numeric), dtype=np.float64)
        if declared != shape or array.shape != shape or not np.isfinite(array).all():
            raise ValueError(f'{name} has invalid values or dimensions; expected {shape}.')
        arrays[name] = array
    if np.any(arrays['SCALE'] <= 0):
        raise ValueError('SCALE values must be positive.')
    match = re.search(r'\bCLASS_NAMES\s*\[\d+\]\s*=\s*\{(.*?)\}\s*;', text, re.S)
    if not match:
        raise ValueError('Model header is missing CLASS_NAMES.')
    names = json.loads('[' + match.group(1) + ']')
    if len(names) != dims['CLASSES'] or not all(isinstance(n, str) for n in names):
        raise ValueError('Class names do not match CLASSES.')
    return text, dims, center, arrays, names


def verify_replay(path, dims, center, arrays):
    """Reject replay data from a different training run before packaging."""
    with np.load(path, allow_pickle=False) as data:
        for key in ('windows', 'labels', 'predictions', 'probabilities', 'mode'):
            if key not in data:
                raise ValueError(f'Replay file is missing {key}.')
        windows = data['windows']
        if windows.ndim != 3 or windows.shape[1:] != (dims['WINDOW'], 2) or not len(windows):
            raise ValueError('Replay window dimensions do not match the model.')
        if not np.isfinite(windows).all():
            raise ValueError('Replay windows contain non-finite values.')
        expected_mode = 'raw' if center else 'envelope'
        if str(data['mode'].item()) != expected_mode:
            raise ValueError('Replay preprocessing mode does not match the model.')
        x = windows.astype(np.float64)
        if center:
            x = x - x.mean(axis=1, keepdims=True)
        rms = np.sqrt(np.mean(x*x, axis=1))
        scaled = (rms - arrays['MEAN']) / arrays['SCALE']
        hidden = np.maximum(0, scaled @ arrays['W1'] + arrays['B1'])
        logits = hidden @ arrays['W2'] + arrays['B2']
        logits -= logits.max(axis=1, keepdims=True)
        probabilities = np.exp(logits)
        probabilities /= probabilities.sum(axis=1, keepdims=True)
        reference = data['probabilities']
        if reference.shape != probabilities.shape or not np.allclose(probabilities, reference, atol=1e-4, rtol=0):
            raise ValueError('Model and replay probabilities do not match. Use files from the SAME training run.')
        if data['predictions'].shape != (len(windows),) or not np.array_equal(probabilities.argmax(axis=1), data['predictions']):
            raise ValueError('Model and replay class predictions do not match.')
        if data['labels'].shape != (len(windows),):
            raise ValueError('Replay labels have invalid dimensions.')
        return len(windows)


def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=root/'firmware/grip_classifier/model_data.h')
    parser.add_argument('--replay', type=Path, default=root/'output/replay_test.npz')
    parser.add_argument('--out', type=Path, default=root/'esp32_package', help='Generated package directory')
    args = parser.parse_args()
    if not args.model.is_file():
        parser.error(f'Model not found: {args.model}. Run train.py first, or supply --model PATH.')
    if not args.replay.is_file():
        parser.error(f'Replay data not found: {args.replay}. Run train.py first, or supply --replay PATH.')
    try:
        header, dims, center, arrays, names = read_model(args.model)
        count = verify_replay(args.replay, dims, center, arrays)
    except (ValueError, OSError, SyntaxError, KeyError) as exc:
        parser.error(str(exc))

    out = args.out.resolve()
    sketch_dir = out/'grip_classifier'
    replay_dest = out/'output/replay_test.npz'
    # Avoid overwriting a source file if someone supplies an unusual --out.
    destination_paths = [sketch_dir/'model_data.h', replay_dest,
                         sketch_dir/'grip_classifier.ino', sketch_dir/'inference.h',
                         out/'replay.py', out/'README.txt']
    if any(source.resolve() == dest for source in (args.model, args.replay)
           for dest in destination_paths):
        parser.error('--out must be separate from the source model and replay files.')
    sketch_dir.mkdir(parents=True, exist_ok=True)
    replay_dest.parent.mkdir(parents=True, exist_ok=True)
    # Arduino defines B1 as a binary constant. Prefix all generated model
    # identifiers in BOTH files to avoid colliding with platform symbols.
    # Validation above still reads the original header produced by train.py.
    model_symbols = ('INPUTS', 'HIDDEN', 'CLASSES', 'WINDOW', 'CENTER_RAW',
                     'CLASS_NAMES', 'MEAN', 'SCALE', 'W1', 'B1', 'W2', 'B2')
    pattern = r'\b(' + '|'.join(model_symbols) + r')\b'
    def prefix_symbols(text):
        return re.sub(pattern, lambda match: 'NN_' + match.group(0), text)
    (sketch_dir/'model_data.h').write_text(prefix_symbols(header), encoding='utf-8')
    (sketch_dir/'inference.h').write_text(prefix_symbols(INFERENCE), encoding='utf-8')
    (sketch_dir/'grip_classifier.ino').write_text(prefix_symbols(LIVE_SKETCH), encoding='utf-8')
    replay_sketch_dir = out/'grip_classifier_replay'
    replay_sketch_dir.mkdir(parents=True, exist_ok=True)
    (replay_sketch_dir/'grip_classifier_replay.ino').write_text(prefix_symbols(SKETCH), encoding='utf-8')
    (replay_sketch_dir/'model_data.h').write_text(prefix_symbols(header), encoding='utf-8')
    (replay_sketch_dir/'inference.h').write_text(prefix_symbols(INFERENCE), encoding='utf-8')
    (out/'replay.py').write_text(REPLAY, encoding='utf-8')
    shutil.copyfile(args.replay, replay_dest)
    instructions = f'''ESP32 model package

Model: {dims['INPUTS']} RMS inputs -> {dims['HIDDEN']} ReLU neurons -> {dims['CLASSES']} softmax outputs
Window: {dims['WINDOW']} sample pairs
Preprocessing: {'raw (subtract window mean)' if center else 'envelope (preserve signal level)'}
Class names: {', '.join(names)}
Verified matching replay windows: {count}

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
'''
    (out/'README.txt').write_text(instructions, encoding='utf-8')
    # Only archive the generated files; do not include unrelated files in --out.
    files = ['grip_classifier/grip_classifier.ino', 'grip_classifier/inference.h',
             'grip_classifier/model_data.h',
             'grip_classifier_replay/grip_classifier_replay.ino',
             'grip_classifier_replay/inference.h', 'grip_classifier_replay/model_data.h',
             'replay.py', 'output/replay_test.npz', 'README.txt']
    archive = out.parent/(out.name + '.zip')
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as zipped:
        for relative in files:
            zipped.write(out/relative, relative)
    print(f'Checked model/replay agreement for {count} windows.')
    print('Classes:', ', '.join(names))
    print('Open in Arduino IDE:', sketch_dir/'grip_classifier.ino')
    print('ZIP package:', archive)
    print('Select your ESP32 board and port, then click Upload.')


if __name__ == '__main__':
    main()
