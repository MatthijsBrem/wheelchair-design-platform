#include <Arduino.h>

int innumerator = 127;
bool increase;
bool vibration_enabled = false;
int VIB_PIN = A0;

void vibration_pattern()
{
  if (increase)
  {
    innumerator += 10;
  }
  else
  {
    innumerator -= 10;
  }

  if (innumerator > 255)
  {
    increase = false;
  }
  else if (innumerator < 127)
  {
    increase = true;
  }
}

void setup()
{

  // put your setup code here, to run once:
}

void loop()
{
  char command = Serial.read();
  if (command == '1')
  {
    Serial.println("Turning on vibration...");
    vibration_enabled = true;
  }
  else if (command == '0')
  {
    Serial.println("Turning off vibration ...");
    vibration_enabled = false;
    analogWrite(VIB_PIN, 0);
  }
  if (vibration_enabled)
  {
    vibration_pattern();
    analogWrite(VIB_PIN, innumerator);
  }
  // put your main code here, to run repeatedly:
  delay(50);
}
