#pragma once

// Pipeline timing
constexpr uint32_t kSampleIntervalMs = 5;    // ~200 Hz sampling
constexpr uint8_t  kNumEmgChannels   = 2;
constexpr uint16_t kWindowSize       = 40;   // samples per classification window

// Pin stand-ins (not wired to real hardware yet)
constexpr uint8_t kAdcPins[kNumEmgChannels] = {34, 35};
constexpr uint8_t kVibrationPin             = 25;
