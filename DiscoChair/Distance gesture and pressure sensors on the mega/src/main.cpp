#include <Arduino.h>

// A small helper
void error(const __FlashStringHelper *err)
{
  Serial.println(err);
  while (1)
    ;
}

/*****************************
      Distance sensor
******************************
*/
#define IR_PIN A0 // Setting up pin to receive voltage from IR

String distanceId = "distance-64a6";

int value, prev_value = -10000;       // int values (read from analog port, both the current and the previous)
int deviation = 0;                    // setting the minimum deviation between the measurements (0 by default)
                                      // up to 512 (although that is pretty useless)
double voltage_value, distance_value; // Converted to Voltage

double convert_to_distance(double voltage)
{
  if (voltage_value < 0.35 || voltage_value > 2.85) // We will ignore values outside the range of measurement, this will happen around 20 - 150cm
    return (0);

  /* General model Exp2:
     f(x) = a*exp(b*x) + c*exp(d*x)
Coefficients (with 95% confidence bounds):
       a =        4498  (-1.731e+04, 2.63e+04)
       b =      -6.351  (-12.72, 0.02085)
       c =       104.9  (61.24, 148.5)
       d =     -0.6928  (-0.9048, -0.4808)

Goodness of fit:
  SSE: 9.915
  R-square: 0.9974
  Adjusted R-square: 0.9955
  RMSE: 1.574

 */

  double a = 4498;
  double b = -6.351;
  double c = 104.9;
  double d = -0.6928;

  return (a * exp(b * voltage) + c * exp(d * voltage));
}

void distance()
{
  value = analogRead(IR_PIN); // reading our analog voltage, careful we only have 10 bit resolution so each
                              // measurement step is only 5V ÷ 1024, so our result will be 0 - 1023

  // if value is within the range of [ previous - σ , previous + σ], ignore it (if value is relatively the same)
  // this will help with having data ocuppy your buffer that is not a significant deviation.
  if (value >= (prev_value - deviation) && value <= (prev_value + deviation))
    return;

  voltage_value = double((value * 5)) / 1023; // converting to voltage [ 0, 5] v.

  distance_value = convert_to_distance(voltage_value); // getting actual distance value(cm) (careful using this, accuracy may not be ideal)
                                                       // due to the functioning of the sensor, once you're closer than around 20 cm, it will
                                                       // start predicting higher distances again. Be careful with this, this is something you can
                                                       // solve with software, however. (if previous results are close to 20 and its going down)
                                                       // then do something.... to ignore results, perhaps.

  if (distance_value < 20 || distance_value > 150) // We will ignore values outside the range of measurement, this will happen around 2.7 -0.4 v
    return;
  /*
    Serial.print("Distance: ");
    Serial.print(value);
    Serial.print(" (0 - 1023) steps,  ");
    Serial.print(voltage_value);
    Serial.print(" (v),  ");
    */
  Serial.println();
  Serial.print(distanceId);
  Serial.print(",");
  Serial.println(distance_value);

  prev_value = value; // Here we have the previous saved variable.
}

/*****************************
      Gesture sensor
******************************
*/

// insert gesture varriables Here
#include "Adafruit_APDS9960.h"
uint8_t gesture = 0;


Adafruit_APDS9960 apds;

String gestureID = "gesturesensorgroup4-cc12";

void gestureSensor()
{

  gesture = apds.readGesture(); // Read gesture into the variable

   // Processing captured gesture, if any.
   if(gesture == APDS9960_DOWN) {
     Serial.println();
     Serial.print(gestureID);
     Serial.print(",");
     Serial.println("down");
   }
   if(gesture == APDS9960_UP){
     Serial.println();
     Serial.print(gestureID);
     Serial.print(",");
    Serial.println("up");
  }
   if(gesture == APDS9960_LEFT){
     Serial.println();
     Serial.print(gestureID);
     Serial.print(",");
    Serial.println("left");
 }
   if(gesture == APDS9960_RIGHT){
     Serial.println();
     Serial.print(gestureID);
     Serial.print(",");
   Serial.println("right");
 }
  int gesture_value = 1;


}
/*****************************
      Pressure sensor
******************************
*/

// insert gesture varriables here

String pressureID = "pressuresensorsgroup4-1865";

void pressure()
{
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);


  if(!apds.begin()){            // Begining the work period of the sensor
      Serial.println("Failed to initialize Sensor! Please check your wiring.");
    }
  else Serial.println("Gesture Sensor initialized!");


  //gesture mode will be entered once proximity mode senses something close
  apds.enableProximity(true);   // Enabling proximity detection
apds.enableGesture(true); // Enabling Gesture detection
  // distance sensor_t  pinMode(IR_PIN, INPUT);                // setting pinmode to read analog value

  deviation = 10; // since there's a bit of a drift in the values if you put the same object over a certain period
                  // we ignore a divergence of around 1 percent around the previous value.
}

void loop()
{
  gestureSensor();
  delay(500);
//  distance();
  delay(500);
//  pressure();
  delay(500);
}
