#include <Arduino.h>

#include "config.h"
#include "adc_sampler.h"
#include "intent_classifier.h"
#include "comm_link.h"
#include "vibration_feedback.h"

namespace {
uint16_t window[kWindowSize];
uint16_t windowIndex = 0;
uint32_t lastSampleMs = 0;
}  // namespace

void setup() {
  adcSamplerInit();
  intentClassifierInit();
  commInit();
  vibrationInit();
}

void loop() {
  uint32_t now = millis();
  if (now - lastSampleMs < kSampleIntervalMs) {
    return;
  }
  lastSampleMs = now;

  // Only channel 0 feeds the classifier window for now; other channels are
  // sampled so the ADC stage exercises all configured pins.
  uint16_t sample = adcSamplerRead(kAdcPins[0]);
  for (uint8_t ch = 1; ch < kNumEmgChannels; ch++) {
    adcSamplerRead(kAdcPins[ch]);
  }

  window[windowIndex++] = sample;
  if (windowIndex < kWindowSize) {
    return;
  }
  windowIndex = 0;

  IntentClass intent = classifyIntent(window, kWindowSize);
  commSendIntent(intent);
  vibrationPulse(intent);
}
