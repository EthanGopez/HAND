#pragma once

#include <Arduino.h>
#include "intent_classifier.h"

// Stubbed haptic output: real version drives a vibration motor (via PWM)
// mounted on the wearable. No motor is required for this to compile or run
// on-device; it just writes to the configured pin.
void vibrationInit();
void vibrationPulse(IntentClass intent);
