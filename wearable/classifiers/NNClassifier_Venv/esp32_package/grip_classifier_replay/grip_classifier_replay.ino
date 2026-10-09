// Replay-first sketch. Receives one NN_WINDOW-sample two-channel window over USB.
// Later replace the serial sample source with a timed sensor acquisition source.
// Data acquisition/collection firmware should remain a separate sketch.
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "inference.h"

float samples[NN_WINDOW][2];
int count=0;
char line[96];
int used=0;
bool overflow=false;

void consumeLine() {
  line[used]='\0';
  if (strcmp(line,"RESET")==0) { count=0; Serial.println("READY"); return; }
  float a,b; char extra;
  if (sscanf(line,"%f,%f %c",&a,&b,&extra)!=2 || !isfinite(a) || !isfinite(b)) {
    count=0; Serial.println("ERROR"); return;
  }
  samples[count][0]=a; samples[count][1]=b; ++count;
  if (count==NN_WINDOW) {
    float feature[2], probability[NN_CLASSES];
    extractFeatures(samples,feature);
    const int label=predictGrip(feature,probability);
    Serial.print("RESULT,"); Serial.print(label);
    for(int k=0;k<NN_CLASSES;++k) { Serial.print(','); Serial.print(probability[k],7); }
    Serial.println(); count=0;
  }
}
void setup() { Serial.begin(115200); Serial.println("READY"); }
void loop() {
  while(Serial.available()) {
    char c=Serial.read();
    if(c=='\r') continue;
    if(c=='\n') {
      if(overflow) {count=0;Serial.println("ERROR");} else consumeLine();
      used=0; overflow=false;
    } else if(used<sizeof(line)-1 && !overflow) line[used++]=c;
    else overflow=true;
  }
}
