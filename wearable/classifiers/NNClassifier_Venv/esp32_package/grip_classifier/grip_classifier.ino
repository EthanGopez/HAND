// Live two-channel analog classifier. Serial is OUTPUT only.
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
