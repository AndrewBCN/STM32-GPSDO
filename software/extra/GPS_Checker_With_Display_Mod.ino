/*******************************************************************************************************
  GPS Checker for u-Blox Neo-M8 receiver
  Original code by Stuart Robinson
  UBX command sending based on code by Brad Burleson
  Repurposed and refactored by André Balsa, June 2021

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

// GPS Checker with I2C SSD1306 display, Hardware Serial on STM32 MCU, u-Blox Neo-M8 GPS receiver

/*******************************************************************************************************
  Program Operation -  This program is a portable GPS checker with display option. It uses an SSD1306 or
  SH1106 128x64 I2C OLED display. It reads the GPS for 5 seconds and copies the characters from the GPS
  to the serial monitor, this is an example printout from a working GPS that has just been powered on:

      Wait GPS Fix 5 seconds
      
      $GNGSA,A,3,32,08,14,,,,,,,,,,1.98,1.06,1.67*17
      $GNGSA,A,3,88,81,87,,,,,,,,,,1.98,1.06,1.67*1D
      $GPGSV,4,1,14,01,85,030,,03,61,253,19,04,16,188,,08,18,173,19*70
      $GPGSV,4,2,14,14,13,269,12,17,35,309,,19,13,321,14,21,67,115,*71
      $GPGSV,4,3,14,22,86,009,,28,17,287,,31,07,104,,32,24,046,27*7D
      $GPGSV,4,4,14,36,30,150,,49,34,184,*79
      $GLGSV,3,1,09,65,69,045,22,66,43,213,,72,22,039,,73,07,012,*6B
      $GLGSV,3,2,09,80,05,326,,81,52,322,13,82,04,320,,87,27,139,24*60
      $GLGSV,3,3,09,88,70,136,30*5C
      $GNGLL,4833.64651,N,00746.91564,E,171944.00,A,A*7F
      $GNRMC,171945.00,A,4833.64644,N,00746.91603,E,1.298,,060621,,,A*60
      $GNVTG,,T,,M,1.298,N,2.403,K,A*3A
      $GNGGA,171945.00,4833.64644,N,00746.91603,E,1,05,1.17,139.2,M,47.3,M,,*46
      
      
      Fix time 918mS
      New GPS Fix Lat,48.560776,Lon,7.781934,Alt,139.2m,Sats,5,HDOP,1.17,Time,17:19:45,Date,6/6/2021
 
  The printout is from a u-Blox Neo M8 receiver. The data from the GPS is also fed into
  the TinyGPS++ library and if there is no fix a message is printed on the serial monitor.

  During setup the GPS receiver is reconfigured in stationary mode and with a higher baud rate.
  When the program detects that the GPS has a fix, it prints the Latitude, Longitude, Altitude, Number
  of satellites in use, the HDOP value, time and date to the serial monitor. If the I2C OLED display is
  attached that is updated as well. Display is assumed to be on I2C address 0x3C.

  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/

#define Program_Version "V0.03"

#include <TinyGPS++.h>                             //get library here > http://arduiniana.org/libraries/tinygpsplus/
TinyGPSPlus gps;                                   //create the TinyGPS++ object

#include <Wire.h> // I2C

#include <U8x8lib.h>                                      //get library here >  https://github.com/olikraus/u8g2 
U8X8_SSD1306_128X64_NONAME_HW_I2C disp(U8X8_PIN_NONE);    //use this line for standard 0.96" SSD1306

float GPSLat;                                      //Latitude from GPS
float GPSLon;                                      //Longitude from GPS
float GPSAlt;                                      //Altitude from GPS
uint8_t GPSSats;                                   //number of GPS satellites in use
uint32_t GPSHdop;                                  //HDOP from GPS
uint8_t hours, mins, secs, day, month;
uint16_t year;
uint32_t startGetFixmS;
uint32_t endFixmS;

void loop()
{
  if (gpsWaitFix(5)) // wait 5 seconds for fix
  {
    Serial.println();
    Serial.println();
    Serial.print(F("Fix time "));
    Serial.print(endFixmS - startGetFixmS);
    Serial.println(F("mS"));

    GPSLat = gps.location.lat();
    GPSLon = gps.location.lng();
    GPSAlt = gps.altitude.meters();
    GPSSats = gps.satellites.value();
    GPSHdop = gps.hdop.value();

    hours = gps.time.hour();
    mins = gps.time.minute();
    secs = gps.time.second();
    day = gps.date.day();
    month = gps.date.month();
    year = gps.date.year();

    printGPSfix();
    displayscreen1();
    startGetFixmS = millis();    //have a fix, next thing that happens is checking for a fix, so restart timer
  }
  else
  {
    disp.clearLine(0);
    disp.setCursor(0, 0);
    disp.print(F("No GPS Fix "));
    disp.print( (millis() - startGetFixmS) / 1000 );
    Serial.println();
    Serial.println();
    Serial.print(F("Timeout - No GPS Fix "));
    Serial.print( (millis() - startGetFixmS) / 1000 );
    Serial.println(F("s"));
  }
}


bool gpsWaitFix(uint16_t waitSecs)
{
  //waits a specified number of seconds for a fix, returns true for good fix

  uint32_t endwaitmS;
  uint8_t GPSchar;

  Serial.print(F("Wait GPS Fix "));
  Serial.print(waitSecs);
  Serial.println(F(" seconds"));

  endwaitmS = millis() + (waitSecs * 1000);

  while (millis() < endwaitmS)
  {
    if (Serial1.available() > 0)
    {
      GPSchar = Serial1.read();
      gps.encode(GPSchar);
      Serial.write(GPSchar);
    }

    if (gps.location.isUpdated() && gps.altitude.isUpdated() && gps.date.isUpdated())
    {
      endFixmS = millis();                                //record the time when we got a GPS fix
      return true;
    }
  }

  return false;
}


void printGPSfix()
{
  float tempfloat;

  Serial.print(F("New GPS Fix "));

  tempfloat = ( (float) GPSHdop / 100);

  Serial.print(F("Lat,"));
  Serial.print(GPSLat, 6);
  Serial.print(F(",Lon,"));
  Serial.print(GPSLon, 6);
  Serial.print(F(",Alt,"));
  Serial.print(GPSAlt, 1);
  Serial.print(F("m,Sats,"));
  Serial.print(GPSSats);
  Serial.print(F(",HDOP,"));
  Serial.print(tempfloat, 2);
  Serial.print(F(",Time,"));

  if (hours < 10)
  {
    Serial.print(F("0"));
  }

  Serial.print(hours);
  Serial.print(F(":"));

  if (mins < 10)
  {
    Serial.print(F("0"));
  }

  Serial.print(mins);
  Serial.print(F(":"));

  if (secs < 10)
  {
    Serial.print(F("0"));
  }

  Serial.print(secs);
  Serial.print(F(",Date,"));

  Serial.print(day);
  Serial.print(F("/"));
  Serial.print(month);
  Serial.print(F("/"));
  Serial.print(year);

  Serial.println();
  Serial.println();
}


void displayscreen1()
{
  //show GPS data on display
  float tempfloat;
  tempfloat = ( (float) GPSHdop / 100);

  disp.clearLine(0);
  disp.clearLine(1);
  disp.setCursor(0, 1);
  disp.print(GPSLat, 6);
  disp.clearLine(2);
  disp.setCursor(0, 2);
  disp.print(GPSLon, 6);
  disp.clearLine(3);
  disp.setCursor(0, 3);
  disp.print(GPSAlt);
  disp.print(F("m"));
  disp.clearLine(4);
  disp.setCursor(0, 4);
  disp.print(F("Sats "));
  disp.print(GPSSats);
  disp.clearLine(5);
  disp.setCursor(0, 5);
  disp.print(F("HDOP "));
  disp.print(tempfloat);

  disp.clearLine(6);
  disp.setCursor(0, 6);

  if (hours < 10)
  {
    disp.print(F("0"));
  }

  disp.print(hours);
  disp.print(F(":"));

  if (mins < 10)
  {
    disp.print(F("0"));
  }

  disp.print(mins);
  disp.print(F(":"));

  if (secs < 10)
  {
    disp.print(F("0"));
  }

  disp.print(secs);
  disp.print(F("  "));

  disp.clearLine(7);
  disp.setCursor(0, 7);

  disp.print(day);
  disp.print(F("/"));
  disp.print(month);
  disp.print(F("/"));
  disp.print(year);
}

void setup()
{
  
  Serial1.begin(9600);  // serial to GPS module
  Serial.begin(115200); // USB serial
  // Reconfigure the GPS receiver
  // first send the $PUBX configuration commands
  delay(5000); // give everything a moment to stabilize
  Serial.println("GPS checker program started");
  Serial.println("Sending $PUBX commands to GPS");
  Serial1.print("$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n"); // disable all VTG messages (useless since we are stationary)
  Serial1.print("$PUBX,41,1,0003,0003,38400,0*24\r\n"); // set GPS baud rate to 38400 in/out protocols NMEA+UBX
  Serial1.flush();                              // empty the buffer
  delay(100);                                   // give it a moment
  Serial1.end();                                // close serial port
  Serial1.begin(38400);                         // re-open at new rate
  delay(5000);
  // second, send the proprietary UBX configuration commands
  Serial.println("Now sending UBX commands to GPS");
  ubxconfig();

  Serial.println();
  Serial.print(F(__TIME__));
  Serial.print(F(" "));
  Serial.println(F(__DATE__));
  Serial.println(F(Program_Version));
  Serial.println();

  disp.begin();
  disp.setFont(u8x8_font_chroma48medium8_r);
  disp.clear();
  disp.setCursor(0, 0);
  disp.print(F("Display Ready"));

  Serial.println(F("29_GPS_Checker_Display Starting"));
  Serial.println();

  startGetFixmS = millis();
}

void ubxconfig() // based on code by Brad Burleson
{
  // send UBX commands to set optimal configuration for GPSDO use
  // we are going to change a single parameter from default by
  // setting the navigation mode to "stationary"
  
  bool gps_set_success = false; // flag setting GPS configuration success
  
  // This UBX command sets stationary mode and confirms it
  Serial.println("Setting u-Blox M8 receiver navigation mode to stationary: ");
  uint8_t setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x53};
  while(!gps_set_success)
  {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    Serial.println();
    Serial.println("UBX command sent, waiting for UBX ACK... ");
    gps_set_success=getUBX_ACK(setNav);
    if (gps_set_success) 
      Serial.println("Success: UBX ACK received! ");
    else
      Serial.println("Oops, something went wrong here... ");
  }
}

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial1.write(MSG[i]);
    Serial.print(MSG[i], HEX);
  }
  Serial1.println();
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  Serial.print(" * Reading ACK response: ");
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;  // header
  ackPacket[1] = 0x62;  // header
  ackPacket[2] = 0x05;  // class
  ackPacket[3] = 0x01;  // id
  ackPacket[4] = 0x02;  // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];  // ACK class
  ackPacket[7] = MSG[3];  // ACK id
  ackPacket[8] = 0;   // CK_A
  ackPacket[9] = 0;   // CK_B
 
  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      Serial.println(" (SUCCESS!)");
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      Serial.println(" (FAILED!)");
      return false;
    }
 
    // Make sure data is available to read
    if (Serial1.available()) {
      b = Serial1.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        Serial.print(b, HEX);
      } 
      else {
        ackByteID = 0;  // Reset and look again, invalid order
      }
 
    }
  }
}
