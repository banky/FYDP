#   Copyright Beach Cleaning Automated
#
#   Author: Johvonna Murray-Bradshaw

"""
    This module is used to transfer serial data between the NVIDIA Jetson and Arduino
"""

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);

#define M_PI 3.14159265358979323846
 
void setup(void) 
{
  Serial.begin(9600);
 //Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}
 
void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);

  float x_angle, y_angle, z_angle;
   
  x_angle = event.orientation.x;
  y_angle = event.orientation.y;
  z_angle = event.orientation.z;
  

  /* Display the floating point data */
  //X
  if (x_angle > 180)
    Serial.print(M_PI / 180 * (x_angle - 360));
  else
    Serial.print(M_PI / 180 * x_angle);

  Serial.print(",");

  //Y
  if (y_angle > 180)
    Serial.print(M_PI / 180 * (y_angle - 360));
  else
    Serial.print(M_PI / 180 * y_angle);

  Serial.print(",");

  //Z
  if (z_angle > 180)
    Serial.println(M_PI / 180 * (z_angle - 360));
  else
    Serial.println(M_PI / 180 * z_angle);

  
  delay(100);
}