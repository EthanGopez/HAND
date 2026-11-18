#include "adc_sampler.h"

void adcSamplerInit() {
  // Real version would call analogReadResolution()/attenuation setup here.
}

uint16_t adcSamplerRead(uint8_t channel) {
  // Synthetic signal: a slow sine carrier plus per-channel jitter, scaled
  // into the 12-bit ADC range so downstream code sees realistic values.
  float t = millis() / 1000.0f;
  float base = 2048.0f + 1024.0f * sinf(t * 2.0f + channel);
  float noise = (float)(esp_random() % 200) - 100.0f;
  float value = base + noise;
  if (value < 0) value = 0;
  if (value > 4095) value = 4095;
  return (uint16_t)value;
}
