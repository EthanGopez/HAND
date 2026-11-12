//permanently set servo id

#include <SCServo.h>
SMS_STS st;
int ID_ChangeFrom = 1;   // The original ID of the servo whose ID is to be changed. The factory default is 1
int ID_Changeto      = 2;   // The new ID
void setup() {
  Serial1.begin(1000000);
  st.pSerial = &Serial1;
  while(!Serial1) {}

  st.unLockEprom(ID_ChangeFrom); //Unlock EPROM-SAFE
  st.writeByte(ID_ChangeFrom, SMS_STS_ID, ID_Changeto);//Change ID
  st.LockEprom(ID_Changeto); // EPROM-SAFE locked
}

void loop() {
}
