#include <SCServo.h>

SMS_STS st;   // default: waits for replies

#define TX_PIN 44
#define RX_PIN 43

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, RX_PIN, TX_PIN);
  st.pSerial = &Serial1;
  delay(3000);

  if (st.FeedBack(1) != -1)
    Serial.printf("REPLY OK  pos=%d volt=%d temp=%dC\n",
                  st.ReadPos(-1), st.ReadVoltage(-1), st.ReadTemper(-1));
  else
    Serial.println("No reply");

  
}

void loop() {
  st.WritePosEx(1, 1800, 500, 30);
  int thing = st.FeedBack(1);
  Serial.println(thing);
  delay(1500);
  st.WritePosEx(1, 2300, 500, 30);
  int thing1 = st.FeedBack(1);
  Serial.println("pos 2" + thing1);
  delay(1500);
  st.WritePosEx(1, 2048, 500, 30);
  int thing2 = st.FeedBack(1);
  Serial.println("pos 3" + thing2);
  delay(1500);
  }