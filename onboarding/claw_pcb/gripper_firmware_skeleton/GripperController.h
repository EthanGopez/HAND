#pragma once

#include <Arduino.h>

namespace hand {

enum class GripperState {
  STOPPED,
  OPENING,
  CLOSING,
  OPEN,
  CLOSED,
  CONTACT,
  FAULT,
};

class GripperController {
 public:
  bool begin();
  void commandOpen();
  void commandClose();
  void stop();
  void update();

  GripperState state() const;
  const char* stateName() const;
  uint16_t pulseWidthUs() const;
  uint32_t forceMillivolts() const;

 private:
  void beginMotion(GripperState next_state);
  void writeServoPulse(uint16_t pulse_width_us);
  void setFault(const char* reason);

  GripperState state_ = GripperState::STOPPED;
  uint16_t pulse_width_us_ = 0;
  uint32_t force_mv_ = 0;
  uint32_t motion_started_ms_ = 0;
  uint32_t last_update_ms_ = 0;
};

}  // namespace hand
