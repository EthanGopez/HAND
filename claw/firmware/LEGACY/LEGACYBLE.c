
#include <Arduino.h>
#include <NimBLEDevice.h>

NimBLECharacteristic* dataCharacteristic;

const char* SERVICE_UUID =
    "12345678-1234-1234-1234-123456789000";

const char* CHARACTERISTIC_UUID =
    "12345678-1234-1234-1234-123456789001";

void setup() {
  Serial.begin(115200);
  delay(1000);

  NimBLEDevice::init("BLE_Test");

  NimBLEServer* server = NimBLEDevice::createServer();

  NimBLEService* service =
      server->createService(SERVICE_UUID);

  dataCharacteristic = service->createCharacteristic(
      CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  dataCharacteristic->setValue("Waiting");
  service->start();

  NimBLEAdvertising* advertising =
      NimBLEDevice::getAdvertising();

  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setName("BLE_Test");
  advertising->start();

  Serial.println("BLE ready. Connect with nRF Connect.");
}

void loop() {
  static int counter = 0;

  if (NimBLEDevice::getServer()->getConnectedCount() > 0) {
    String message = "Hello BLE " + String(counter++);

    dataCharacteristic->setValue(message.c_str());
    dataCharacteristic->notify();

    Serial.println(message);
    delay(1000);
  } else {
    delay(100);
  }
}
