#include <SCServo.h>

SMS_STS st;


// Change pin numbers to match board (GPIO based)
#define S_RXD 44
#define S_TXD 43

// Set up servos

const int NUM_SERVOS = 3;

const uint8_t SERVO_IDS[NUM_SERVOS] = {1, 2, 3};

// Set up poses

struct GripPose {
  int16_t  pos[NUM_SERVOS];  // target position per servo (same order as SERVO_IDS)
  uint16_t speed;            // tune later
  uint8_t  acc;              // tune later
};

const GripPose POSES[] = {

  // 5 poses including rest

  //  servo1  servo2  servo3   speed  acc
  { { 4095,   4095,   4095 },  3400,  50 },  // pose 0 (placeholder)
  { { 4095,   4095,   4095 },  3400,  50 },  // pose 1 (placeholder)
  { { 4095,   4095,   4095 },  3400,  50 },  // pose 2 (placeholder)
  { { 4095,   4095,   4095 },  3400,  50 },  // pose 3 (placeholder)
  { { 4095,   4095,   4095 },  3400,  50 },  // pose 4 (placeholder)
  
  // ... more poses to come
};

const int NUM_POSES = sizeof(POSES) / sizeof(POSES[0]);

// Claw State

volatile int posNum = 0;
int lastSentPos = -1;

// Custom Function

void updateGrip(int pos) {
  if (pos < 0 || pos >= NUM_POSES) {
    Serial.printf("updateGrip: pose %d out of range (0-%d)\n", pos, NUM_POSES - 1);
    return;
  }

  const GripPose &p = POSES[pos];
  for (int i = 0; i < NUM_SERVOS; i++) {
    st.WritePosEx(SERVO_IDS[i], p.pos[i], p.speed, p.acc);
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
}

void loop() {

  int requested = posNum;

  // Only updates clawre when grip position changes

  if (requested != lastSentPos) {
    updateGrip(requested);
    lastSentPos = requested;  
  }

}
