#include <SCServo.h>

SMS_STS st;

#define RX_PIN 44//D7
#define TX_PIN 43//D6

const u8 OLD_ID = 1;   // what it is now (factory default)
const u8 NEW_ID = 2;   // what you want it to be: change this for each servo

void setup() {
  Serial.begin(115200);
  Serial2.begin(1000000, SERIAL_8N1, RX_PIN, TX_PIN);
  st.pSerial = &Serial2;
  delay(1000);

  if (st.FeedBack(OLD_ID) == -1) {
    Serial.println("No servo answered at the old ID - check wiring and power");
    return;
  }

  st.unLockEprom(OLD_ID);                 // allow permanent settings to change
  st.writeByte(OLD_ID, SMS_STS_ID, NEW_ID);
  st.LockEprom(NEW_ID);                   // lock again, using the NEW id

  delay(500);
  if (st.FeedBack(NEW_ID) != -1)
    Serial.printf("Done. Servo now answers at ID %d\n", NEW_ID);
  else
    Serial.println("Changed, but no reply at the new ID - power cycle and try again");
}

void loop() {}