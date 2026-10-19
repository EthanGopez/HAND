#pragma once

#include <Arduino.h>
#include "config.h"

// Stubbed ADC front-end: no electrodes attached yet, so this synthesizes a
// plausible EMG-like signal per channel instead of calling analogRead().
void adcSamplerInit();
uint16_t adcSamplerRead(uint8_t channel);
