#include "GripperController.h"

hand::GripperController gripper;
String command_buffer;

void printHelp() {
  Serial.println("Commands: OPEN, CLOSE, STOP, STATUS, HELP");
}

void printStatus() {
  Serial.print("state=");
  Serial.print(gripper.stateName());
  Serial.print(" pulse_us=");
  Serial.print(gripper.pulseWidthUs());
  Serial.print(" force_mv=");
  Serial.println(gripper.forceMillivolts());
}

void handleCommand(String command) {
  command.trim();
  command.toUpperCase();

  if (command == "OPEN") {
    gripper.commandOpen();
  } else if (command == "CLOSE") {
    gripper.commandClose();
  } else if (command == "STOP") {
    gripper.stop();
  } else if (command == "STATUS") {
    printStatus();
  } else if (command == "HELP") {
    printHelp();
  } else if (command.length() > 0) {
    Serial.print("Unknown command: ");
    Serial.println(command);
    printHelp();
  }
}

void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.println("HAND gripper onboarding skeleton");
  if (!gripper.begin()) {
    Serial.println("Initialization failed; controller remains in FAULT.");
  }
  printHelp();
  printStatus();
}

void loop() {
  while (Serial.available() > 0) {
    const char next = static_cast<char>(Serial.read());
    if (next == '\n' || next == '\r') {
      if (command_buffer.length() > 0) {
        handleCommand(command_buffer);
        command_buffer = "";
      }
    } else if (command_buffer.length() < 64) {
      command_buffer += next;
    }
  }

  gripper.update();
  delay(1);
}
