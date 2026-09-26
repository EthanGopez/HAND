#pragma once

#include <Arduino.h>

enum class IntentClass : uint8_t {
  kRest = 0,
  kOpen,
  kClose,
  kPinch,
};

// Stubbed classifier: real version will run a trained model over the
// windowed EMG features. For now, threshold on mean amplitude so the rest
// of the pipeline (comm + vibration) has something to react to.
void intentClassifierInit();
IntentClass classifyIntent(const uint16_t* samples, size_t numSamples);
