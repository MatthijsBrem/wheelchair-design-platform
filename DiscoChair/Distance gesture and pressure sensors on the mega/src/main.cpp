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
  if (gesture == APDS9960_DOWN)
  {
    Serial.println();
    Serial.print(gestureID);
    Serial.print(",");
    Serial.println("down");
  }
  if (gesture == APDS9960_UP)
  {
    Serial.println();
    Serial.print(gestureID);
    Serial.print(",");
    Serial.println("up");
  }
  if (gesture == APDS9960_LEFT)
  {
    Serial.println();
    Serial.print(gestureID);
    Serial.print(",");
    Serial.println("left");
  }
  if (gesture == APDS9960_RIGHT)
  {
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
#define PRESSURE_PIN  A1 // Setting up pin to receive voltage PRESSURE 1
int value_Pressure_1, prev_value_Pressure_1 = -10000;     // int values (read from analog port, both the current and the previous)
int deviationPressure = 0;                                // setting the minimum deviation between the measurements (0 by default)
                                                          // up to 512 (although that is pretty useless)
double voltage_value_Pressure_1, newton_value_Pressure_1; // Converted to Voltage

#define PRESSURE_PIN  A2 // Setting up pin to receive voltage PRESSURE 2
int value_Pressure_2, prev_value_Pressure_2 = -10000;     // int values (read from analog port, both the current and the previous)
                                                          // up to 512 (although that is pretty useless)
double voltage_value_Pressure_2, newton_value_Pressure_2; // Converted to Voltage

#define PRESSURE_PIN  A3 // Setting up pin to receive voltage PRESSURE 3
int value_Pressure_3, prev_value_Pressure_3 = -10000;     // int values (read from analog port, both the current and the previous)
                                                          // up to 512 (although that is pretty useless)
double voltage_value_Pressure_3, newton_value_Pressure_3; // Converted to Voltage

#define PRESSURE_PIN  A4 // Setting up pin to receive voltage PRESSURE 4
int value_Pressure_4, prev_value_Pressure_4 = -10000;     // int values (read from analog port, both the current and the previous)
                                                          // up to 512 (although that is pretty useless)
double voltage_value_Pressure_4, newton_value_Pressure_4; // Converted to Voltage

String pressureID = "pressuresensorsgroup4-1865";

double convert_to_newtons(double voltage)
{
  /* General fitting model Exp2:
     f(x) = a*exp(b*x) + c*exp(d*x)
     Coefficients (with 95% confidence bounds):
       a =     0.01419  (0.01163, 0.01676)
       b =      0.9523  (0.8922, 1.012)
       c =    -0.01461  (-0.02317, -0.006043)
       d =      -2.231  (-6.605, 2.142)
       Goodness of fit:
       SSE: 7.906e-06
       R-square: 0.9999
       Adjusted R-square: 0.9997
       RMSE: 0.001988
   */
  double a = 0.01419;
  double b = 0.9523;
  double c = -0.01461;
  double d = -2.231;

  return ((a * exp(b * voltage) + c * exp(d * voltage)) * 9.81); // the result of the fit is in KgF to convert to newton we simply
                                                                 // multiply by 9.81, if you want data in KgF, remove the final multiplication!
}

String pressureID = "pressuresensorsgroup4-1865";

double convert_to_newtons(double voltage)
{
  /* General fitting model Exp2:
     f(x) = a*exp(b*x) + c*exp(d*x)
     Coefficients (with 95% confidence bounds):
       a =     0.01419  (0.01163, 0.01676)
       b =      0.9523  (0.8922, 1.012)
       c =    -0.01461  (-0.02317, -0.006043)
       d =      -2.231  (-6.605, 2.142)
       Goodness of fit:
       SSE: 7.906e-06
       R-square: 0.9999
       Adjusted R-square: 0.9997
       RMSE: 0.001988
   */
  double a = 0.01419;
  double b = 0.9523;
  double c = -0.01461;
  double d = -2.231;

  return ((a * exp(b * voltage) + c * exp(d * voltage)) * 9.81); // the result of the fit is in KgF to convert to newton we simply
                                                                 // multiply by 9.81, if you want data in KgF, remove the final multiplication!
}

void pressure()
{
  value_Pressure_1 = analogRead(PRESSURE_PIN); // reading our analog voltage, careful we only have 10 bit resolution so each
                                               // measurement step is only 5V ÷ 1024, so our result will be 0 - 1023

  // if value is within the range of [ previous - σ , previous + σ], ignore it (if value is relatively the same)
  // this will help with having data ocuppy your buffer that is not a significant deviation.
  if (value_Pressure_1 >= (prev_value_Pressure_1 - deviationPressure) && value_Pressure_1 <= (prev_value_Pressure_1 + deviationPressure))
    return;

  voltage_value_Pressure_1 = double((value_Pressure_1 * 5)) / 1023;       // converting to voltage [ 0, 5] v.
  newton_value_Pressure_1 = convert_to_newtons(voltage_value_Pressure_1); // getting actual force value (careful using this, accuracy may not be ideal)
                                                                          // sensitivity after 1Kgf and before 0.06kgf is limited, you can lower the deviation
                                                                          // for some improvements
  Serial.print("Pressure_1: ");
  Serial.print(value_Pressure_1);
  Serial.print(" (0 - 1023) steps,  ");
  Serial.print(voltage_value_Pressure_1);
  Serial.print(" (v),  ");
  Serial.print(newton_value_Pressure_1);
  Serial.println(" N.");

  prev_value_Pressure_1 = value_Pressure_1;

  prev_value_Pressure_2 = value_Pressure_2; // PRESSURE SENSOR 2

    value_Pressure_2 = analogRead(PRESSURE_PIN); // reading our analog voltage, careful we only have 10 bit resolution so each
                                                 // measurement step is only 5V ÷ 1024, so our result will be 0 - 1023

    // if value is within the range of [ previous - σ , previous + σ], ignore it (if value is relatively the same)
    // this will help with having data ocuppy your buffer that is not a significant deviation.
    if (value_Pressure_2 >= (prev_value_Pressure_2 - deviationPressure) && value_Pressure_2 <= (prev_value_Pressure_2 + deviationPressure))
      return;

    voltage_value_Pressure_2 = double((value_Pressure_2 * 5)) / 1023;       // converting to voltage [ 0, 5] v.
    newton_value_Pressure_2 = convert_to_newtons(voltage_value_Pressure_2); // getting actual force value (careful using this, accuracy may not be ideal)
                                                                            // sensitivity after 1Kgf and before 0.06kgf is limited, you can lower the deviation
                                                                            // for some improvements
    Serial.print("Pressure_2: ");
    Serial.print(value_Pressure_2);
    Serial.print(" (0 - 1023) steps,  ");
    Serial.print(voltage_value_Pressure_2);
    Serial.print(" (v),  ");
    Serial.print(newton_value_Pressure_2);
    Serial.println(" N.");


  prev_value_Pressure_3 = value_Pressure_3; // PRESSURE SENSOR 3
  value_Pressure_3 = analogRead(PRESSURE_PIN); // reading our analog voltage, careful we only have 10 bit resolution so each
                                                 // measurement step is only 5V ÷ 1024, so our result will be 0 - 1023

    // if value is within the range of [ previous - σ , previous + σ], ignore it (if value is relatively the same)
    // this will help with having data ocuppy your buffer that is not a significant deviation.
    if (value_Pressure_3 >= (prev_value_Pressure_3 - deviationPressure) && value_Pressure_3 <= (prev_value_Pressure_3 + deviationPressure))
      return;

    voltage_value_Pressure_3 = double((value_Pressure_3 * 5)) / 1023;       // converting to voltage [ 0, 5] v.
    newton_value_Pressure_3 = convert_to_newtons(voltage_value_Pressure_3); // getting actual force value (careful using this, accuracy may not be ideal)
                                                                            // sensitivity after 1Kgf and before 0.06kgf is limited, you can lower the deviation
                                                                            // for some improvements
    Serial.print("Pressure_3: ");
    Serial.print(value_Pressure_3);
    Serial.print(" (0 - 1023) steps,  ");
    Serial.print(voltage_value_Pressure_3);
    Serial.print(" (v),  ");
    Serial.print(newton_value_Pressure_3);
    Serial.println(" N.");

    prev_value_Pressure_4 = value_Pressure_4; // PRESSURE SENSOR 4
  value_Pressure_4 = analogRead(PRESSURE_PIN); // reading our analog voltage, careful we only have 10 bit resolution so each
                                                 // measurement step is only 5V ÷ 1024, so our result will be 0 - 1023

    // if value is within the range of [ previous - σ , previous + σ], ignore it (if value is relatively the same)
    // this will help with having data ocuppy your buffer that is not a significant deviation.
    if (value_Pressure_4 >= (prev_value_Pressure_4 - deviationPressure) && value_Pressure_4 <= (prev_value_Pressure_4 + deviationPressure))
      return;

    voltage_value_Pressure_4 = double((value_Pressure_4 * 5)) / 1023;       // converting to voltage [ 0, 5] v.
    newton_value_Pressure_4 = convert_to_newtons(voltage_value_Pressure_4); // getting actual force value (careful using this, accuracy may not be ideal)
                                                                            // sensitivity after 1Kgf and before 0.06kgf is limited, you can lower the deviation
                                                                            // for some improvements
    Serial.print("Pressure_4: ");
    Serial.print(value_Pressure_4);
    Serial.print(" (0 - 1023) steps,  ");
    Serial.print(voltage_value_Pressure_4);
    Serial.print(" (v),  ");
    Serial.print(newton_value_Pressure_4);
    Serial.println(" N.");

}


void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  if (!apds.begin())
  { // Begining the work period of the sensor
    Serial.println("Failed to initialize Sensor! Please check your wiring.");
  }
  else
    Serial.println("Gesture Sensor initialized!");

  //gesture mode will be entered once proximity mode senses something close
  apds.enableProximity(true); // Enabling proximity detection
  apds.enableGesture(true);   // Enabling Gesture detection

  Serial.println("Pressure sensors begin here!");
  pinMode(PRESSURE_PIN, INPUT); // setting pinmode to read analog value
  // distance sensor_t  pinMode(IR_PIN, INPUT);                // setting pinmode to read analog value

  deviation = 10; // since there's a bit of a drift in the values if you put the same object over a certain period

  // we ignore a divergence of around 1 percent around the previous value.
}

void loop()
{
  gestureSensor();
  delay(100);
  //  distance();
  //  delay(500);
  pressure();
  delay(100);
}
