
// ------------
// Arduino pins
// ------------
#define azPWM_Pin    5     // d5 azimuth servo
#define elPWM_Pin    6     // d6 elevation servo

#define SetHomePin   9     // d9 button

#define  StatusLed   10    // d10 - Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
#define  BoardLed    13    // d13


// --------------------
// Application specific
// --------------------
#define minDist      3     // dist from home before tracking starts

// ------
// Servos
// ------
#define SERVOMIDDLE  135   // Angle corresponding to middle of servo throw

#define llAz         0
#define ulAz         2*SERVOMIDDLE
#define llEl         0
#define ulEl         90

#define MinAzPWM     500
#define MaxAzPWM     2375

#define MinElPWM     1410
#define MaxElPWM     2390


// ------------
// Testing
// -------
// #define TimeToTestServos      
