void setup() {
  Serial.begin(9600);
}

void loop() {
  //switch the print with whatever needs to be printed out
  Serial.println(millis());
  //add a timeout as necessary to avoid writing an excessive amount
  delay(1000);
}
