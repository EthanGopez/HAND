#include <SCServo.h>

SMS_STS st;

#define RX_PIN 16
#define TX_PIN 17
#define INPUT_PIN 34          // potentiometer for testing (ADC1 pin)

const int NUM_SERVOS = 3;
u8 ids[NUM_SERVOS] = {1, 2, 3};

// Per-servo range: where each servo goes at input = min and input = max.
// Swap the two numbers to reverse a servo's direction.
s16 posAtMin[NUM_SERVOS] = {1000, 1000, 3000};   // servo 3 is mirrored
s16 posAtMax[NUM_SERVOS] = {3000, 3000, 1000};

const int IN_MIN = 0;         // input range (calibrate to your real input)
const int IN_MAX = 4095;

float smoothed = 0;

void printTorqueInfo() {
  if (st.FeedBack(SERVO_ID) != -1) {
    Serial.printf("load=%d  current=%d\n", st.ReadLoad(-1), st.ReadCurrent(-1));
  } else {
    Serial.println("No reply");
  }
}
void setup() {
  Serial.begin(115200);
  Serial2.begin(1000000, SERIAL_8N1, RX_PIN, TX_PIN);
  st.pSerial = &Serial2;
  delay(1000);

  // Check that every servo answers
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (st.FeedBack(ids[i]) == -1)
      Serial.printf("Servo %d: no reply\n", ids[i]);
    else
      Serial.printf("Servo %d found, pos=%d\n", ids[i], st.ReadPos(-1));
  }
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last < 20) return;      // update ~50 Hz
  last = millis();

  int raw = analogRead(INPUT_PIN);
  smoothed = 0.8 * smoothed + 0.2 * raw;

  s16 pos[NUM_SERVOS];
  u16 spd[NUM_SERVOS];
  u8  acc[NUM_SERVOS];

  for (int i = 0; i < NUM_SERVOS; i++) {
    long p = map((long)smoothed, IN_MIN, IN_MAX, posAtMin[i], posAtMax[i]);
    pos[i] = constrain(p, min(posAtMin[i], posAtMax[i]), max(posAtMin[i], posAtMax[i]));
    spd[i] = 2000;
    acc[i] = 50;
  }

  st.SyncWritePosEx(ids, NUM_SERVOS, pos, spd, acc);   // one packet, all three move together
}