#include "comm_link.h"

namespace {
const char* intentName(IntentClass intent) {
  switch (intent) {
    case IntentClass::kRest:  return "rest";
    case IntentClass::kOpen:  return "open";
    case IntentClass::kClose: return "close";
    case IntentClass::kPinch: return "pinch";
  }
  return "unknown";
}
}  // namespace

void commInit() {
  Serial.begin(115200);
}

void commSendIntent(IntentClass intent) {
  Serial.printf("{\"intent\":\"%s\",\"t\":%lu}\n", intentName(intent), millis());
}
