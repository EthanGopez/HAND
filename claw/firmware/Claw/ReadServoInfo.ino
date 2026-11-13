#include <SCServo.h>

SMS_STS st;

//#define RX_PIN 16
//#define TX_PIN 17
#define S_RXD 1
#define S_TXD 2


const int NUM_SERVOS = 3;
u8 ids[NUM_SERVOS] = {1, 2, 3};

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(3000); 
  Serial.println("Aight if this  workign correctly this should read the torque,load, and position");

}

void loop() {
  static uint32_t last = 0;
  if (millis() - last < 100) return;   // 10 readings per second
  last = millis();

  for (int i = 0; i < NUM_SERVOS; i++) {
    if (st.FeedBack(ids[i]) != -1) {
      int pos = st.ReadPos(-1);       // 0-4095 should be the values for ST servos
      int load = st.ReadLoad(-1);      // motor drive level, -1000 to 1000
      int current = st.ReadCurrent(-1);   // motor current; this is what appearantly should be closer to the torque, rather than load
      Serial.printf("ID %d | pos=%d  load=%d  current=%d\n", ids[i], pos, load, current);
    } else {
      Serial.printf("ID %d | no reply\n", ids[i]);
    }
  }
  Serial.println();
}
