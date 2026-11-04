#include "intent_classifier.h"

void intentClassifierInit() {
  // Real version would load model weights / calibration data here.
}

IntentClass classifyIntent(const uint16_t* samples, size_t numSamples) {
  if (numSamples == 0) return IntentClass::kRest;

  uint32_t sum = 0;
  for (size_t i = 0; i < numSamples; i++) {
    sum += samples[i];
  }
  float mean = (float)sum / numSamples;

  if (mean < 2200.0f) return IntentClass::kRest;
  if (mean < 2700.0f) return IntentClass::kOpen;
  if (mean < 3200.0f) return IntentClass::kClose;
  return IntentClass::kPinch;
}
