#include "vibration_feedback.h"
#include "config.h"

namespace {
constexpr uint8_t kPwmChannel = 0;
constexpr uint32_t kPwmFreqHz = 5000;
constexpr uint8_t kPwmResolutionBits = 8;

uint8_t dutyForIntent(IntentClass intent) {
  switch (intent) {
    case IntentClass::kRest:  return 0;
    case IntentClass::kOpen:  return 90;
    case IntentClass::kClose: return 160;
    case IntentClass::kPinch: return 255;
  }
  return 0;
}
}  // namespace

void vibrationInit() {
  ledcSetup(kPwmChannel, kPwmFreqHz, kPwmResolutionBits);
  ledcAttachPin(kVibrationPin, kPwmChannel);
}

void vibrationPulse(IntentClass intent) {
  ledcWrite(kPwmChannel, dutyForIntent(intent));
}
