#include "GripperController.h"

#include <algorithm>

#include "config.h"

namespace hand {

bool GripperController::begin() {
  pinMode(config::FORCE_SENSOR_PIN, INPUT);
  analogReadResolution(12);

  if (!ledcAttach(
          config::SERVO_PWM_PIN,
          config::SERVO_FREQUENCY_HZ,
          config::SERVO_PWM_RESOLUTION_BITS)) {
    setFault("could not attach servo PWM pin");
    return false;
  }

  pulse_width_us_ = config::SERVO_OPEN_US;
  ledcWrite(config::SERVO_PWM_PIN, 0);
  state_ = GripperState::STOPPED;
  return true;
}

void GripperController::commandOpen() {
  if (state_ != GripperState::FAULT) {
    beginMotion(GripperState::OPENING);
  }
}

void GripperController::commandClose() {
  if (state_ != GripperState::FAULT) {
    beginMotion(GripperState::CLOSING);
  }
}

void GripperController::stop() {
  if (state_ != GripperState::FAULT) {
    state_ = GripperState::STOPPED;
  }
}

void GripperController::update() {
  const uint32_t now_ms = millis();
  force_mv_ = analogReadMilliVolts(config::FORCE_SENSOR_PIN);

  if (state_ != GripperState::OPENING && state_ != GripperState::CLOSING) {
    return;
  }
  if (now_ms - motion_started_ms_ > config::MOTION_TIMEOUT_MS) {
    setFault("motion timeout");
    return;
  }
  if (now_ms - last_update_ms_ < config::CONTROL_PERIOD_MS) {
    return;
  }
  last_update_ms_ = now_ms;

  if (state_ == GripperState::CLOSING &&
      force_mv_ >= config::CONTACT_THRESHOLD_MV) {
    state_ = GripperState::CONTACT;
    return;
  }

  if (state_ == GripperState::OPENING) {
    const uint32_t next = pulse_width_us_ + config::STEP_SIZE_US;
    pulse_width_us_ = static_cast<uint16_t>(
        std::min<uint32_t>(next, config::SERVO_OPEN_US));
    writeServoPulse(pulse_width_us_);
    if (pulse_width_us_ == config::SERVO_OPEN_US) {
      state_ = GripperState::OPEN;
    }
    return;
  }

  const int32_t next = static_cast<int32_t>(pulse_width_us_) -
                       static_cast<int32_t>(config::STEP_SIZE_US);
  pulse_width_us_ = static_cast<uint16_t>(
      std::max<int32_t>(next, config::SERVO_CLOSED_US));
  writeServoPulse(pulse_width_us_);
  if (pulse_width_us_ == config::SERVO_CLOSED_US) {
    state_ = GripperState::CLOSED;
  }
}

GripperState GripperController::state() const { return state_; }

const char* GripperController::stateName() const {
  switch (state_) {
    case GripperState::STOPPED:
      return "STOPPED";
    case GripperState::OPENING:
      return "OPENING";
    case GripperState::CLOSING:
      return "CLOSING";
    case GripperState::OPEN:
      return "OPEN";
    case GripperState::CLOSED:
      return "CLOSED";
    case GripperState::CONTACT:
      return "CONTACT";
    case GripperState::FAULT:
      return "FAULT";
  }
  return "UNKNOWN";
}

uint16_t GripperController::pulseWidthUs() const { return pulse_width_us_; }

uint32_t GripperController::forceMillivolts() const { return force_mv_; }

void GripperController::beginMotion(GripperState next_state) {
  state_ = next_state;
  motion_started_ms_ = millis();
  last_update_ms_ = 0;
}

void GripperController::writeServoPulse(uint16_t pulse_width_us) {
  pulse_width_us = std::max<uint16_t>(
      config::SERVO_MIN_US,
      std::min<uint16_t>(pulse_width_us, config::SERVO_MAX_US));
  const uint32_t max_duty = (1UL << config::SERVO_PWM_RESOLUTION_BITS) - 1UL;
  const uint32_t duty =
      (static_cast<uint32_t>(pulse_width_us) *
       config::SERVO_FREQUENCY_HZ * max_duty) /
      1000000UL;
  if (!ledcWrite(config::SERVO_PWM_PIN, duty)) {
    setFault("could not write servo PWM");
  }
}

void GripperController::setFault(const char* reason) {
  state_ = GripperState::FAULT;
  Serial.print("FAULT: ");
  Serial.println(reason);
}

}  // namespace hand
