#include "definitions.h"

void  PositionServos(float Az, float El, float hmHdg) {
  int16_t pntAz = pointAz(Az, hmHdg);  // Remap Azimuth into a 180 degree window with home-heading centre, negative behind us

  // If the craft moves out of this restricted field of reference, servos should wait at the most recent in-field position

  if ((pntAz >= llAz) && (pntAz <= ulAz) && (El >= llEl) && (El <= ulEl)) { // Our restricted FOV
    LastGoodpntAz = pntAz;
    LastGoodEl = El;
  }

  azPWM = map(LastGoodpntAz, ulAz, llAz, MinAzPWM, MaxAzPWM);  // Map the az / el to the servo PWMs
  elPWM = map(LastGoodEl, llEl, ulEl, MinElPWM, MaxElPWM);     // Servos happen to be mounted such that action is reversed

  azServo.writeMicroseconds(azPWM);
  elServo.writeMicroseconds(elPWM);

  #if defined TimeToTestServos
    u8g.firstPage();
    do {
      u8g.drawStr( 0, 12, " az = "); u8g.setPrintPos(45, 12); u8g.print(azPWM); u8g.print(" "); u8g.print((int)Az); u8g.write(0xBA);
      u8g.drawStr( 0, 32, " el = "); u8g.setPrintPos(45, 32); u8g.print(elPWM); u8g.print(" "); u8g.print((int)El); u8g.write(0xBA);
    } while ( u8g.nextPage() );
  #endif
}


int16_t pointAz(int16_t Azin, int16_t rhmHdg)  {
  // Remap the craft's absolute Az from the home position to the 180 degree azimuth aperture facing us, and behind us
  // 0 degrees on the left and 180 on the right, same but mirrored and negative behind us 180 on the right 0 on the left.
  // Home (antenna) heading is at the centre (90 deg) of the 180 degree forward azimuth aperture
  // pointAz is the heading to the craft from the home position, and thus where to point the antenna,
  //  RELATIVE to our frame of reference.

  int16_t pntAz = Azin;

  pntAz = pntAz - rhmHdg + SERVOMIDDLE;    // Make heading relative to SERVOMIDDLE degrees in front of us
  if (pntAz < 0) pntAz = SERVOMIDDLE - pntAz; // Correct the SERVOMIDDLE deg boundary if necessary

  if (pntAz > 360) pntAz = pntAz - 360; // All the way round to positive territory again

  return pntAz;
}



#if defined TimeToTestServos
void TestServos() {
  PositionServos(SERVOMIDDLE, 0, SERVOMIDDLE); 
  delay(2000);
  PositionServos(0, 0, SERVOMIDDLE); 
  delay(5000);
  PositionServos(SERVOMIDDLE, 0, SERVOMIDDLE);  
  delay(2000);
  PositionServos(270, 0, SERVOMIDDLE);  
  delay(5000);
  PositionServos(SERVOMIDDLE, 0, SERVOMIDDLE); 
  delay(2000);
  PositionServos(SERVOMIDDLE, 45, SERVOMIDDLE);  
  delay(2000);
  PositionServos(SERVOMIDDLE, 90, SERVOMIDDLE);  
  delay(5000);
  PositionServos(0, 90, SERVOMIDDLE); 
  delay(2000);
  PositionServos(SERVOMIDDLE, 90, SERVOMIDDLE); 
  delay(2000);
  PositionServos(270, 90, SERVOMIDDLE); 
  delay(2000);
  PositionServos(SERVOMIDDLE, 0, SERVOMIDDLE);   
  delay(5000);
}
#endif




