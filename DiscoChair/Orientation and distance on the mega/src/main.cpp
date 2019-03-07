#include <Arduino.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int randomvalue = random(0,10);
  Serial.print("testvalue, ");
  Serial.println(randomvalue);
  delay(500);
}
