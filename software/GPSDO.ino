/*******************************************************************************************************
  GPSDO by André Balsa, May 2021
  reuses excellent code by Stuart Robinson - 05/04/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

// GPSDO with optional I2C SSD1306 display, Hardware Serial on STM32 MCU

/*******************************************************************************************************
  Program Operation -  This program is a portable GPS checker with display option. It uses an SSD1306 or
  SH1106 128x64 I2C OLED display. It reads the GPS for 5 seconds and copies the characters from the GPS
  to the serial monitor, this is an example printout from a working GPS that has just been powered on;
   
  GPSDO Starting
  Wait GPS Fix 5 seconds
  Timeout - No GPS Fix 5s
  Wait GPS Fix 5 seconds
  $PGACK,103*40
  $PGACK,105*46
  $PMTK011,MTKGPS*08
  $PMTK010,001*2E
  $PMTK010,00æ*2D
  $GPGGA,235942.800,,,,,0,0,,,M,,M,,*4B
  $GPGSA,A,1,,,,,,,,,,,,,,,*1E
  $GPRMC,235942.800,V,,,,,0.00,0.00,050180,,,N*42
  $GPVTG,0.00,T,,M,0.00,N,0.00,K,N*32
  $GPGSV,1,1,03,30,,,43,07,,,43,05,,,38*70

  Timeout - No GPS Fix 5s
  Wait GPS Fix 5 seconds

  That printout is from a Meadiatek GPS, the Ublox ones are similar. The data from the GPS is also fed into
  the TinyGPS++ library and if there is no fix a message is printed on the serial monitor.

  When the program detects that the GPS has a fix, it prints the Latitude, Longitude, Altitude, Number
  of satellites in use, the HDOP value, time and date to the serial monitor. If the I2C OLED display is
  attached that is updated as well. Display is assumed to be on I2C address 0x3C.

  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/



/*******************************************************************************************************
  Program Operation -  This program is a portable GPS checker and display. It uses an SSD1306 or SH1106
  128x64 I2C OLED display. At startup the program starts checking the data coming from the GPS for a
  valid fix. It reads the GPS for 5 seconds and if there is no fix, prints a message on the serial monitor
  and updates the seconds without a fix on the display. During this time the data coming from the GPS is
  copied to the serial monitor also.

  When the program detects that the GPS has a fix, it prints the Latitude, Longitude, Altitude, Number
  of satellites in use, the HDOP value, time and date to the serial monitor. If the I2C OLED display is
  attached that is updated as well. Display is assumed to be on I2C address 0x3C.

  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/

#define Program_Name "GPSDO"
#define Program_Version "v0.02c"
#define authorname "André Balsa reusing code from Stuart Robinson"

#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x02000000)
#error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x02000000"
#endif

#include <TinyGPS++.h>                             // get library here > http://arduiniana.org/libraries/tinygpsplus/
TinyGPSPlus gps;                                   // create the TinyGPS++ object

#include <Wire.h>                                  // Hardware I2C library on STM32

#include <U8x8lib.h>                                      // get library here >  https://github.com/olikraus/u8g2 
U8X8_SSD1306_128X64_NONAME_HW_I2C disp(U8X8_PIN_NONE);    // use this line for standard 0.96" SSD1306

#include <Adafruit_MCP4725.h>                      // MCP4725 Adafruit library
Adafruit_MCP4725 dac;
const uint16_t default_DAC_output = 2593; // this varies from OCXO to OCXO
uint16_t adjusted_DAC_output;             // we adjust this value to close the frequency locked loop
volatile bool must_adjust_DAC = false;

#define VctlInputPin PB0
int adcVctl = 0;                      // Vctl read by ADC pin PB0

// LEDs

// Blue onboard LED blinks to indicate ISR is working
#define blueledpin  PC13    // Blue onboard LED is on PC13 on STM32F411CEU6 Black Pill

// Yellow extra LED is off, on or blinking to indicate some GPSDO status
#define yellowledpin PB1   // Yellow LED on PB1
volatile int yellow_led_state = 2;  // global variable 0=off 1=on 2=1Hz blink


// GPS data
float GPSLat;                                      // Latitude from GPS
float GPSLon;                                      // Longitude from GPS
float GPSAlt;                                      // Altitude from GPS
uint8_t GPSSats;                                   // number of GPS satellites in use
uint32_t GPSHdop;                                  // HDOP from GPS
uint8_t hours, mins, secs, day, month;
uint16_t year;
uint32_t startGetFixmS;
uint32_t endFixmS;

// Uptime data
volatile uint8_t  uphours = 0;
volatile uint8_t  upminutes = 0;
volatile uint8_t  upseconds = 0;
volatile uint16_t updays = 0;
volatile bool halfsecond = false;

/* OCXO frequency measurement */
volatile uint32_t fcount=0, previousfcount=0, calcfreqint=10000000;

/* Moving average frequency variables
   Basically we store the counter captures for 10 and 100 seconds.
   When the buffers are full, the average frequency is quite simply
   the difference between the oldest and newest data divided by the size
   of the buffer.
   Each second, when the buffers are full, we overwrite the oldest data
   with the newest data and calculate each average frequency.
 */
volatile uint32_t circbuf_ten[11]; // 10+1 seconds circular buffer
volatile uint32_t circbuf_hun[101]; // 100+1 seconds circular buffer

volatile uint32_t cbiten_newest=0; // index to oldest, newest data
volatile uint32_t cbihun_newest=0;

volatile bool cbTen_full=false, cbHun_full=false;  // flag when buffer full
double avgften, avgfhun; // average frequency calculated once the buffer is full


// Interrupt Service Routine for the 2Hz timer
void Timer_ISR_2Hz(void) // WARNING! Do not attempt I2C communication inside the ISR

{ // Toggle pin. 2hz toogle --> 1Hz pulse, perfect 50% duty cycle
  digitalWrite(blueledpin, !digitalRead(blueledpin));

  halfsecond = !halfsecond; // true @ 1Hz

  // read TIM2->CCR3 twice per second and if it has changed, calculate OCXO frequency

  fcount = TIM2->CCR3;
  
  if ((fcount > 4000000000) && (fcount < 4010000000) && halfsecond) must_adjust_DAC = true; else must_adjust_DAC = false; // once every 429s
  
  if (fcount < 4280000000) { // if we are way below wraparound value (2^32)
    if (fcount > previousfcount) {  // if we have a new count - that happens once per second
      if (((fcount - previousfcount) > 9999800) && ((fcount - previousfcount) < 10000200)) { // if we have a valid fcount, otherwise it's discarded
        logfcount();  // save fcount in the ring buffers
        calcfreqint = fcount - previousfcount; // the difference is exactly the OCXO frequency in Hz
        // previousfcount = fcount;
      }
      previousfcount = fcount;
    }
  } else { // prepare for wraparound every 429 seconds
    // TIM2->CCR3 = 0x0; // clear CCR3 (no need to stop counter) perhaps this is not needed
    cbTen_full=false; cbHun_full=false; // we also need to refill the ring buffers
    cbiten_newest=0; cbihun_newest=0;
    previousfcount = 0;
  }                           

  switch (yellow_led_state)
  {
    case 0:
      // turn off led
      digitalWrite(yellowledpin, LOW);
      break;
    case 1:
      // turn on led
      digitalWrite(yellowledpin, HIGH);
      break;
    case 2:
      // blink led
      digitalWrite(yellowledpin, !digitalRead(yellowledpin));
      break;
    default:
      // default is to turn off led
      digitalWrite(yellowledpin, LOW);
      break; 
  }
  
  // Uptime clock - in days, hours, minutes, seconds
  if (halfsecond)
  {
      if (++upseconds > 59)
      {
          upseconds = 0;
          if (++upminutes > 59)
          {
              upminutes = 0;
              if (++uphours > 23)
              {
                  uphours = 0;
                  ++updays;
              }
          }
      }
  }  
}

void logfcount() // called once per second from ISR to update all the ring buffers
{
  // 10 seconds buffer
  circbuf_ten[cbiten_newest]=fcount;
  cbiten_newest++;
  if (cbiten_newest > 10) {
     cbTen_full=true; // that only needs to happen once, when the buffer fills up for the first time
     cbiten_newest = 0;   // (wrap around)
  }
  // 100 seconds buffer
  circbuf_hun[cbihun_newest]=fcount;
  cbihun_newest++;
  if (cbihun_newest > 100) {
     cbHun_full=true; // that only needs to happen once, when the buffer fills up for the first time
     cbihun_newest = 0;   // (wrap around)
  }
}


void setup()
{
  // Setup 2Hz Timer
  HardwareTimer *tim2Hz = new HardwareTimer(TIM9);
  
  // configure blueledpin in output mode
  pinMode(blueledpin, OUTPUT);

  // configure yellow_led_pin in output mode
  pinMode(yellowledpin, OUTPUT);    
  
  tim2Hz->setOverflow(2, HERTZ_FORMAT); // 2 Hz
  tim2Hz->attachInterrupt(Timer_ISR_2Hz);
  tim2Hz->resume();

  // Setup serial interfaces
  Serial1.begin(9600);  // Hardware serial to GPS module
  Serial.begin(115200); // USB serial

  Serial.println();
  Serial.print(F(__TIME__));
  Serial.print(F(" "));
  Serial.println(F(__DATE__));
  Serial.println(F(Program_Name));
  Serial.println(F(Program_Version));
  Serial.println();

  // Setup OLED I2C display
  // Note that u8x8 library initializes I2C hardware interface
  disp.setBusClock(400000L); // try to avoid display locking up
  disp.begin();
  disp.setFont(u8x8_font_chroma48medium8_r);
  disp.clear();
  disp.setCursor(0, 0);
  disp.print(F("GPSDO - v0.02c"));

  // Initialize I2C again (not sure this is needed, though)
  Wire.begin();
  // try setting a higher I2C clock speed
  Wire.setClock(400000L);  

  // Setup I2C DAC, read voltage on PA2
  adjusted_DAC_output = default_DAC_output; // initial DAC value
  dac.begin(0x60);
  // Output Vctl to DAC, but do not write to DAC EEPROM 
  dac.setVoltage(adjusted_DAC_output, false); // min=0 max=4096 so 2048 should be 1/2 Vdd = approx. 1.65V
  analogReadResolution(12); // make sure we read 12 bit values

  Serial.println(F("GPSDO Starting"));
  Serial.println();

  startGetFixmS = millis();

  // Setup and start Timer 2 which measures OCXO frequency
  
  // setup pin used as ETR (10MHz external clock from OCXO)
  pinMode(PA15, INPUT_PULLUP);    // setup PA15 as input pin
  pinModeAF(PA15, GPIO_AF1_TIM2); // setup PA15 as TIM2 channel 1 / ETR
  
  // setup Timer 2 in input capture mode, active input channel 3
  // to latch counter value on rising edge

  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
  HardwareTimer *FreqMeasTim = new HardwareTimer(TIM2);
  
  // Configure rising edge detection to measure frequency
  FreqMeasTim->setMode(3, TIMER_INPUT_CAPTURE_RISING, PB10);

  // Configure 32-bit auto-reload register (ARR) with maximum possible value
  TIM2->ARR = 0xffffffff; // count to 2^32, then wraparound (approximately every 429 seconds)

  // select external clock source mode 2 by writing ECE=1 in the TIM2_SMCR register
  TIM2->SMCR |= TIM_SMCR_ECE; // 0x4000
  
  // start the timer
  FreqMeasTim->resume();
}

void pinModeAF(int ulPin, uint32_t Alternate)
{
   int pn = digitalPinToPinName(ulPin);

   if (STM_PIN(pn) < 8) {
      LL_GPIO_SetAFPin_0_7( get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn), Alternate);
   } else {
      LL_GPIO_SetAFPin_8_15(get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn), Alternate);
   }

   LL_GPIO_SetPinMode(get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn), LL_GPIO_MODE_ALTERNATE);
}


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

    adcVctl = analogRead(VctlInputPin);

    calcavg(); // calculate frequency averages

    printGPSDOstats();
    displayscreen1();
    startGetFixmS = millis();    //have a fix, next thing that happens is checking for a fix, so restart timer
  }
  else
  {
    disp.clearLine(1);
    disp.setCursor(0, 1);
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


void printGPSDOstats() 
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
  float Vctl = (float(adcVctl)/4096) * 3.3;
  Serial.print("Vctl: ");
  Serial.print(Vctl);
  Serial.print("  DAC: ");
  Serial.println(adjusted_DAC_output);

  // OCXO frequency measurements
  Serial.println();
  Serial.print(F("Counter: "));
  Serial.print(TIM2->CCR3);
  Serial.print(F(" Frequency: "));
  Serial.print(calcfreqint);
  Serial.print(F(" Hz"));
  Serial.println();
  Serial.print("10s Frequency Avg: ");
  Serial.print(avgften,1);
  Serial.print(F(" Hz"));
  Serial.println();
  Serial.print("100s Frequency Avg: ");
  Serial.print(avgfhun,2);
  Serial.print(F(" Hz"));
  Serial.println();  
  
  Serial.println();
  Serial.println();
}


void displayscreen1()
{
  //show GPS data on OLED display
  float tempfloat;
  disp.clearLine(1); // clear error message, if any

  // Latitude
  //disp.clearLine(2);
  disp.setCursor(0, 2);
  disp.print(GPSLat, 6);
  // Longitude
  //disp.clearLine(3);
  disp.setCursor(0, 3);
  disp.print(GPSLon, 6);
  // Altitude and Satellites
  //disp.clearLine(4);
  disp.setCursor(0, 4);
  disp.print(GPSAlt);
  disp.print(F("m"));
  disp.setCursor(9, 4);
  disp.print(F("Sats "));
  disp.print(GPSSats);
  if (GPSSats < 10) disp.print(F(" ")); // clear possible digit when sats >= 10
  // HDOP
  //disp.clearLine(5);
  disp.setCursor(0, 5);
  disp.print(F("HDOP "));
  tempfloat = ((float) GPSHdop / 100);
  disp.print(tempfloat);

  // Time
  //disp.clearLine(6);
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

  // Date
  //disp.clearLine(7);
  disp.setCursor(0, 7);

  disp.print(day);
  disp.print(F("/"));
  disp.print(month);
  disp.print(F("/"));
  disp.print(year);
}

void calcavg() {
  // Calculate the OCXO frequency to 1 or 2 decimal places only when the respective buffers are full
  
  if (cbTen_full) { // we want (latest fcount - oldest fcount) / 10
    
    uint32_t latfcount, oldfcount; // latest fcount, oldest fcount stored in ring buffer

    // latest fcount is always circbuf_ten[cbiten_newest-1]
    // except when cbiten_newest is zero
    // oldest fcount is always circbuf_ten[cbiten_newest] when buffer is full

    if (cbiten_newest == 0) latfcount = circbuf_ten[10];
    else latfcount = circbuf_ten[cbiten_newest-1];
    oldfcount = circbuf_ten[cbiten_newest];
    
    avgften = double(latfcount - oldfcount)/10.0;
    // oldest fcount is always circbuf_ten[cbiten_newest-2]
    // except when cbiten_newest is <2 (zero or 1)
    
  }
   
  if (cbHun_full) { // we want (latest fcount - oldest fcount) / 100
    
    uint32_t latfcount, oldfcount;

    // latest fcount is always circbuf_hun[cbihun_newest-1]
    // except when cbihun_newest is zero
    // oldest fcount is always circbuf_hun[cbihun_newest] when buffer is full

    if (cbihun_newest == 0) latfcount = circbuf_hun[100];
    else latfcount = circbuf_hun[cbihun_newest-1];
    oldfcount = circbuf_hun[cbihun_newest];
    
    avgfhun = double(latfcount - oldfcount)/100.0;
    // oldest fcount is always circbuf_ten[cbiten_newest-2]
    // except when cbiten_newest is <2 (zero or 1)
  } 
}
