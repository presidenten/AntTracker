#include "definitions.h"

#include <SoftwareSerial.h>
SoftwareSerial mavSerial(8, 7); // RX | TX

#include <GCS_MAVLink.h>
#include <Servo.h>

#include <U8glib.h> // Oled lib
U8GLIB_SSD1306_128X32 u8g(U8G_I2C_OPT_NONE);  // Oled I2C

uint8_t  ledState = LOW; 
uint32_t led_millis = 0;
uint32_t startup_millis = 0;

bool  mavGood = false;
bool  gpsGood = false;
bool  homGood = false;    
bool  homeInitialised = false;
bool  new_GPS_data = false;

uint32_t hb_millis = 0;
uint16_t  hb_count = 0;

//  variables for servos
int16_t azPWM = 0;
int16_t elPWM = 0;
int16_t LastGoodpntAz = SERVOMIDDLE;
int16_t LastGoodEl = 0;

bool ft = true;

// 3D Location vectors
struct Location {
  float lat; 
  float lon;
  float alt;
  float hdg;
};

struct Location hom = {
  0,0,0,0
}; // home location

struct Location cur = {
  0,0,0,0
}; // current location
  
struct Vector {
  float    az;                     
  float    el;                     
  int32_t  dist;
};

// Vector for home-to-current location
struct Vector hc_vector = {
  SERVOMIDDLE, 0, 0
};
  
// Servo class declarations
Servo azServo; // Azimuth
Servo elServo; // Elevation

// ----------------
// Mavlink Messages
// ----------------
// Message #0  HEARTHBEAT 

// Message #24  GPS_RAW_INT 
uint8_t    ap_fixtype = 3;            // 0= No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
uint8_t    ap_sat_visible = 0;        // numbers of visible satelites
uint8_t    ap_gps_status = 0;         // (ap_sat_visible*10) + ap_fixtype; 
int32_t    ap_latitude = 0;           // 7 assumed decimal places
int32_t    ap_longitude = 0;          // 7 assumed decimal places
int32_t    ap_amsl24 = 0;             // 1000 = 1m 

// Message GLOBAL_POSITION_INT ( #33 ) (Filtered)
int32_t    ap_lat;            // Latitude, expressed as degrees * 1E7
int32_t    ap_lon;            // Longitude, expressed as degrees * 1E7
int32_t    ap_amsl33;         // Altitude above mean sea level (millimeters)
int32_t    ap_alt_ag;         // Altitude above ground (millimeters)
uint16_t   ap_hdg;           // Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees


void setup()
{
  u8g.setFont(u8g_font_unifont);
  u8g.setColorIndex(1); // Instructs the display to draw with a pixel on. 
  
  u8g.firstPage();
  do {  
    u8g.drawStr( 0, 22, "Starting up...");
  }while( u8g.nextPage() );
  
  mavSerial.begin(57600);
  delay(500); 
  
  startup_millis = millis();
  pinMode(SetHomePin, INPUT_PULLUP); 
  pinMode(StatusLed, OUTPUT );  
  pinMode(BoardLed, OUTPUT);     // Board LED mimics status led
  digitalWrite(BoardLed, HIGH);  // Logic is reversed! Initialse off    

  azServo.attach(azPWM_Pin);
  elServo.attach(elPWM_Pin);
  
  PositionServos(SERVOMIDDLE, 0, SERVOMIDDLE);   // Intialise servos to az=SERVOMIDDLE, el=0, hom.hdg = SERVOMIDDLE;
  
  #if defined TimeToTestServos
    TestServos();   // Fine tune MaxPWM and MinPWM in Servos module
  #endif


  bool cpsGood = Initialise_Compass();  // Check if we have a compass on the Tracker_Compass
  
  if (!(cpsGood)) {    
    u8g.firstPage();
    do {  
      u8g.drawStr( 0, 22, "No compass...");
    } while( u8g.nextPage() );
    while (1) delay (1000);  // Wait here forever
  }
  
  u8g.firstPage();
  do {  
    u8g.drawStr( 0, 22, "Press button to set direction");
  } while( u8g.nextPage() );


  // Wait for button press
  uint8_t saveMag = 0;
  while (saveMag == 0) {
    hom.hdg = GetMagHeading();
      
    u8g.firstPage();
    do {  
      u8g.drawStr( 0, 22, "homhdg: ");u8g.setPrintPos(65, 22);u8g.print(hom.hdg);
    } while( u8g.nextPage() );
    
    saveMag = digitalRead(SetHomePin);
    delay(10);
  }
  
  u8g.firstPage();
  do {  
      u8g.drawStr( 0, 12, "Center is set to:");u8g.setPrintPos(65, 12);u8g.print(hom.lat);u8g.write(0xBA);
      u8g.drawStr( 0, 32, "      ");u8g.setPrintPos(65, 32);u8g.print(hom.hdg);
  } while( u8g.nextPage() );

  
  delay(3000);
}


void loop()  {
  MavLink_Receive(); // Get Mavlink Data

  if (mavGood && (millis() - hb_millis >= 8000)){
    mavGood = false;   // If no heartbeat for 8 seconds then link timed out 
    gpsGood = false;
    
    u8g.firstPage();
    do {  
      u8g.drawStr( 0, 22, "No heartbeat");
    } while( u8g.nextPage() );
  }
    
  ServiceTheStatusLed();
    
  if (gpsGood==1 && ft) {
    ft=false;
    if (homeInitialised == 0) {
      u8g.firstPage();
      do {  
        u8g.drawStr( 0, 22, "GPS Lock! Press btn 2 start");
      } while( u8g.nextPage() );
    }
    else {
      u8g.firstPage();
      do {  
        u8g.drawStr( 0, 22, "GPS Lock again...");
      } while( u8g.nextPage() );
    }
  }

  if (mavGood && homeInitialised && new_GPS_data) {  //  every time there is new GPS data from mavlink
    GetAzEl(hom, cur);
    if (hc_vector.dist >= minDist) PositionServos(hc_vector.az, hc_vector.el, hom.hdg);
    new_GPS_data = false;
  }
 
  uint8_t SetHomeState = digitalRead(SetHomePin);   // Check if home button is pushed

  // If no compass use home established when 3D+ lock established, homGood = 1
  if ((SetHomeState == 0) && (gpsGood) && (!homeInitialised)){       
    hom.lat = cur.lat;
    hom.lon = cur.lon;
    hom.alt = cur.alt;
    
    homeInitialised = true;
    u8g.firstPage();
    do {
      u8g.drawStr( 0, 12, "HLat = ");u8g.setPrintPos(65, 12);u8g.print(hom.lat);u8g.write(0xBA);
      u8g.drawStr( 0, 32, "HLon = ");u8g.setPrintPos(65, 32);u8g.print(hom.lon);u8g.write(0xBA);
    } while( u8g.nextPage() ); 
    delay(3000);
  }  
}

uint8_t len;
void MavLink_Receive() { 
  mavlink_message_t msg;
  mavlink_status_t status;

  while(mavSerial.available()) {
    uint8_t c = mavSerial.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:    // #0   http://mavlink.org/messages/common
          mavlink_msg_heartbeat_get_type(&msg);
          mavlink_msg_heartbeat_get_autopilot(&msg);
          mavlink_msg_heartbeat_get_base_mode(&msg);
          mavlink_msg_heartbeat_get_custom_mode(&msg);
          mavlink_msg_heartbeat_get_system_status(&msg);
          mavlink_msg_heartbeat_get_mavlink_version(&msg);
          hb_millis=millis(); 

          if(!mavGood) {
            hb_count++; 
            if((hb_count >= 3) || (homeInitialised)) {  // If 3 heartbeats or 1 hb && previously connected, we are connected
              mavGood=true;
              hb_count=0;
            }
          }
          break;
     
        case MAVLINK_MSG_ID_GPS_RAW_INT:          // #24
          if (!mavGood) break;        
          ap_fixtype = mavlink_msg_gps_raw_int_get_fix_type(&msg);                   // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix
          ap_sat_visible =  mavlink_msg_gps_raw_int_get_satellites_visible(&msg);    // number of visible satelites
          ap_gps_status = (ap_sat_visible*10) + ap_fixtype; 
          if(ap_fixtype > 2)  {
            ap_latitude = mavlink_msg_gps_raw_int_get_lat(&msg);
            ap_longitude = mavlink_msg_gps_raw_int_get_lon(&msg);
            ap_amsl24 = mavlink_msg_gps_raw_int_get_alt(&msg);             // 1m =1000 
          }
          break;
 
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:     // #33
          if ((!mavGood) || (ap_fixtype < 3)) break;  
          // We have a 3D Lock - change to 4 if you want 3D plus
          
          ap_lat = mavlink_msg_global_position_int_get_lat(&msg);             // Latitude, expressed as degrees * 1E7
          ap_lon = mavlink_msg_global_position_int_get_lon(&msg);             // Pitch angle (rad, -pi..+pi)
          ap_amsl33 = mavlink_msg_global_position_int_get_alt(&msg);          // x Supposedly altitude above mean sea level (millimeters)
          ap_alt_ag = mavlink_msg_global_position_int_get_relative_alt(&msg); // Altitude above ground (millimeters)

          if (!gpsGood) {
            gpsGood = true;
          }
          new_GPS_data = true;
          
          if (!homGood) {
            homGood = true;
            
            hom.lat = (float)ap_lat / 1E7;
            hom.lon = (float)ap_lon / 1E7;
            hom.alt = (float)ap_amsl24 / 1E3;
            hom.hdg = (float)ap_hdg / 100;
            
            u8g.firstPage();
            do {
              u8g.drawStr( 0, 12, " Lat = ");u8g.setPrintPos(65, 12);u8g.print(hom.lat);u8g.write(0xBA);
              u8g.drawStr( 0, 32, " Lon = ");u8g.setPrintPos(65, 32);u8g.print(hom.lon);u8g.write(0xBA);
            } while( u8g.nextPage() ); 
          } 

          cur.lat =  (float)ap_lat / 1E7;
          cur.lon = (float)ap_lon / 1E7;
          cur.alt = ap_amsl24 / 1E3;
          cur.hdg = ap_hdg / 100;
                            
          break;
      }
    }
  }
}


void ServiceTheStatusLed() {
  if (gpsGood) {
    if (homeInitialised) 
      ledState = HIGH;
    else 
      BlinkLed(100);
    }
  else 
     if (mavGood) 
       BlinkLed(1300);
     else
       ledState = LOW;
       
  digitalWrite(StatusLed, ledState);  
  digitalWrite(BoardLed, !ledState);
}


void BlinkLed(uint16_t rate) {
  unsigned long cMillis = millis();
  if (cMillis - led_millis >= rate) {    // blink period
    led_millis = cMillis;
    if (ledState == LOW) {
      ledState = HIGH;
    }   
    else {
      ledState = LOW;
    } 
  }
}

