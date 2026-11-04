#pragma once

#include <Arduino.h>
#include "intent_classifier.h"

// Stubbed communication layer: real version will be BLE (or UART to a
// companion app). For now, just serialize over the USB serial port so the
// pipeline is observable without any radio hardware.
void commInit();
void commSendIntent(IntentClass intent);
