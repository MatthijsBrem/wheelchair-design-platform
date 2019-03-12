#include <Arduino.h>

// A small helper
void error(const __FlashStringHelper *err)
{
  Serial.println(err);
  while (1)
    ;
}

/*****************************
      orientation senor
******************************
*/
//orientation sensor libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

String orientationId = "orientation-a54b";

//intiali the orientation sensor
#define BNO055_SAMPLERATE_DELAY_MS (200)
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Initializes BNO055 sensor
void initSensor(void)
{
  if (!bno.begin())
  {
    error(F("No BNO055 detected. Check your wiring or I2C ADDR!"));
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}

// get the orientation and print it
void orientation()
{
  // Get Quaternion data (no 'Gimbal Lock' like with Euler angles)
  imu::Quaternion quat = bno.getQuat();
  float quatX = quat.x();
  float quatY = quat.y();
  float quatZ = quat.z();
  Serial.print(orientationId);
  Serial.print(",");
  Serial.print(quatX);
  Serial.print(",");
  Serial.print(quatY);
  Serial.print(",");
  Serial.println(quatZ);
}

void setup()
{
  // Setup the BNO055 sensor
  initSensor();
}

void loop()
{
  orientation();
  delay(500);
}
