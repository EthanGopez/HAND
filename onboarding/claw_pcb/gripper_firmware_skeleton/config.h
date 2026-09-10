#pragma once

#include <Arduino.h>

namespace hand {
namespace config {

// Training placeholders only. Verify against the selected board and schematic.
constexpr uint8_t SERVO_PWM_PIN = 4;
constexpr uint8_t FORCE_SENSOR_PIN = 1;

constexpr uint32_t SERVO_FREQUENCY_HZ = 50;
constexpr uint8_t SERVO_PWM_RESOLUTION_BITS = 14;
constexpr uint16_t SERVO_MIN_US = 900;
constexpr uint16_t SERVO_MAX_US = 2100;
constexpr uint16_t SERVO_OPEN_US = 1700;
constexpr uint16_t SERVO_CLOSED_US = 1100;

constexpr uint32_t CONTROL_PERIOD_MS = 20;
constexpr uint16_t STEP_SIZE_US = 12;
constexpr uint32_t MOTION_TIMEOUT_MS = 2500;
constexpr uint32_t CONTACT_THRESHOLD_MV = 1800;

}  // namespace config
}  // namespace hand
