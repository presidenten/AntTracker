#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// Get the mag declination of your location here:
// http://www.magnetic-declination.com/ and insert in # defined in next line
#define DECLINATION  6.8 // In degrees
 
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

uint8_t Initialise_Compass() {
  if(!mag.begin()) {
    return 0;
  }

 return 1;
}

float GetMagHeading() {
  // Read Magnetometer
  sensors_event_t event; 
  mag.getEvent(&event);
  
  float fHeading = RadToDeg(atan2(event.magnetic.y, event.magnetic.x));  //All in degrees now
  
  fHeading += DECLINATION;  // Add magnetic declination

  fHeading = Normalise_360(fHeading);
 
  return fHeading;
}

int16_t Normalise_360(int16_t arg) {
  if (arg < 0) arg += 360;
  if (arg > 359) arg -= 360;
  return arg;
}

float RadToDeg (float _Rad) {
  return _Rad * 180 / PI;  
}

