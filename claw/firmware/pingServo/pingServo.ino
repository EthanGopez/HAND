#include <SCServo.h>
SMS_STS st;
int TEST_ID = 3;  // Servo ID to be tested
void setup()
{
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, RX, TX); // Custom serial port
  st.pSerial = &Serial1;
  while(!Serial1) {}
}

void loop()
{
  int ID = st.Ping(TEST_ID);  //Ping the servo with the specified ID; return -1 if ping fails
  if(ID!=-1){
    Serial.print("Servo ID:");
    Serial.println(ID, DEC);
    delay(100);
  } else {
    Serial.println("Ping servo ID error!");
    delay(2000);
  }
}