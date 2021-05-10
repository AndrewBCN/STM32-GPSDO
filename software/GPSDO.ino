/*
  GPSDO V.001m
  First attempt at Bluetooth interface using HardwareSerial Serial6
  OCXO frequency measurement actually WORKS!
  10s, 100s averages WORK!
  Discards bad fcounts, avoids counter wraparound
  Code cleanup / beautifying (much more needed)
  Timer interrupt, various sensors, I2C MCP4725 12-bit DAC, and GPS read basic
  This version adds OCXO frequency moving average over 10 seconds, 100 seconds
  and 1000 seconds, making good use of the ample 128kB RAM available
  in the STM32F411CEU6 MCU.
  Preparing to close the loop...

  Display:
    - SSD1306 128x64 I2C OLED display and uptime clock
    - U8x8 library, which does not lock up even after many hours

  I2C sensors:
    - AHT10 temperature humidity sensor using AdaFruit AHTX0 library

  SPI sensors:
    - BMP280 using PA4

  Internal sensors:
    - VREF
    - VBAT
    - TEMP
  
  Uses TinyGPS++ library to read GPS data and report position, date and UTC time on USB serial

  Assumes GPS RX and TX are connected to UART1 (pins A9-TX1 and A10-RX1 on Black Pill MCU).

  Note the use of custom fields because Neo-8M module differs from Neo-6M module and sends
  non-standard NMEA sequences
  
  Also this sketch shows how to configure HardwareTimer to execute an interrupt service routine
  at regular intervals. ISR toggles pin.
  Once configured, there is only CPU load for ISRs.
  
*/

// Includes
// --------

#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x02000000)
#error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x02000000"
#endif

#include <TinyGPS++.h>

// I2C sensor and DAC
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_MCP4725.h>

// BMP280 - SPI
#include <SPI.h>
#include <Adafruit_BMP280.h>

// Internal sensors
// #include "stm32yyxx_ll_adc.h" // redundant include, not required

// OLED Display
#include <U8x8lib.h>

// Objects and variables
// ---------------------

//  UART               RX   TX
HardwareSerial Serial2(PA3, PA2);   // Serial to Bluetooth module

TinyGPSPlus gps;  // GPS object

// U8g2 Constructor - hardware I2C, U8x8 API, no reset pin
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE); 

const int VctlInputPin = PB0; // read Vctl on PB0 analog input pin

Adafruit_AHTX0 aht;
Adafruit_MCP4725 dac;

#define BMP280_CS   (PA4)
Adafruit_BMP280 bmp(BMP280_CS);        // hardware SPI, use PA4 as Chip Select
const uint16_t PressureOffset = 1350;  // that offset must be calculated for your sensor/location

#define blueledpin  PC13    // Blue user LED is on PC13 on STM32F411CEU6 Black Pill
#define yellow_led_pin PB1  // Yellow LED on PB1

volatile int yellow_led_state = 2;  // global variable 0=off 1=on 2=1Hz blink

volatile float ocxo_current = 0;

const uint16_t default_DAC_output = 2593; // this varies from OCXO to OCXO
uint16_t adjusted_DAC_output;             // we adjust this value to close the frequency locked loop
volatile bool must_adjust_DAC = false;

volatile uint8_t  hours = 0;
volatile uint8_t  minutes = 0;
volatile uint8_t  seconds = 0;
volatile uint16_t days = 0;
volatile bool halfsecond = false;
volatile char GPSStr[3] = "00";
volatile char SecondsStr[3] = "00";
volatile char minutesStr[3] = "00";
volatile char hoursStr[3] = "00";

volatile bool gpsLock = false;
volatile uint32_t sats = 0;
char uptime[9] = "00:00:00";
char updays[9] = "000 days";
char GMTtime[9] = "00:00:00";

/* Internal sensor contants
   Values available in datasheet
 */
#define CALX_TEMP 25
#define V25       760
#define AVG_SLOPE 2500
#define VREFINT   1210

/* Analog read resolution */
#define LL_ADC_RESOLUTION LL_ADC_RESOLUTION_12B
#define ADC_RANGE 4096

/* OCXO frequency measurement */
volatile uint32_t fcount=0, previousfcount=0, calcfreqint=10000000;
uint32_t channel;

/* Moving average stuff
   Basically we store the counter captures for 10, 100 and 1000 seconds.
   When the buffers are full, the average frequency is quite simply
   the difference between the oldest and newest data divided by the size
   of the buffer.
   Each second, when the buffers are full, we overwrite the oldest data
   with the newest data and recalculate each average frequency.
 */
volatile uint32_t circbuf_ten[11]; // 10+1 seconds circular buffer
volatile uint32_t circbuf_hun[101]; // 100+1 seconds circular buffer
// volatile uint32_t circbuf_tho[1000]; // 1000 seconds circular buffer
volatile uint32_t cbiten_newest=0; // index to oldest, newest data
volatile uint32_t cbihun_newest=0;
// volatile uint32_t cbitho_newest=0;
volatile bool cbTen_full=false, cbHun_full=false; //, cbTho_full=false; // flag when buffer full
double avgften, avgfhun; //, avgftho; // average frequency calculated once the buffer is full


// Interrupt Service Routine for the 2Hz timer
// -------------------------------------------

void Update_IT_callback(void) // WARNING! Do not attempt I2C communication inside the ISR

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
      digitalWrite(yellow_led_pin, LOW);
      break;
    case 1:
      // turn on led
      digitalWrite(yellow_led_pin, HIGH);
      break;
    case 2:
      // blink led
      digitalWrite(yellow_led_pin, !digitalRead(yellow_led_pin));
      break;
    default:
      // default is to turn off led
      digitalWrite(yellow_led_pin, LOW);
      break; 
  }
  
  // uptime clock - in days, hours, minutes, seconds
  if (halfsecond)
  {
      if (++seconds > 59)
      {
          seconds = 0;
          if (++minutes > 59)
          {
              minutes = 0;
              if (++hours > 23)
              {
                  hours = 0;
                  ++days;
                  //if (days = 1) printOneDay();
                  //else printDays();  
              }
              // printHours();
          }
          // printMinutes();
      }
      // printSeconds();
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
  // 1000 seconds buffer
/*  circbuf_tho[cbitho_newest]=fcount;
  cbitho_newest++;
  if (cbitho_newest > 9) {
     cbTho_full=true; // that only needs to happen once, when the buffer fills up for the first time
     cbitho_newest = 0;   // (wrap around)
  } */
}

// Setup
// -----

void setup()
{
  adjusted_DAC_output = default_DAC_output; // initial DAC value
  
  TIM_TypeDef *Instance = TIM9;  // tested with TIM3, TIM11, TIM9 on STM32F411CEU6 Black Pill board

  // Instantiate HardwareTimer object. Thanks to 'new' instanciation, HardwareTimer is not destructed when setup() function is finished.
  HardwareTimer *MyTim = new HardwareTimer(Instance);

  // configure blueledpin in output mode
  pinMode(blueledpin, OUTPUT);

  // configure yellow_led_pin in output mode
  pinMode(yellow_led_pin, OUTPUT);
  
  MyTim->setOverflow(2, HERTZ_FORMAT); // 2 Hz
  MyTim->attachInterrupt(Update_IT_callback);
  MyTim->resume();

  Serial1.begin(9600);  // serial to GPS module
  Serial2.begin(9600);  // serial to Bluetooth module
  
  Serial.begin(115200); // USB serial

  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();  
  
  Serial.println("Waiting for GPS signal acquisition (30 seconds to 15 minutes)...");

  // u8x8 library initializes I2C hardware interface
  u8x8.setBusClock(400000L); // try to avoid display locking up
  u8x8.begin();
  // u8x8.setPowerSave(0);
 
  Wire.begin();
  // try setting a higher I2C clock speed
  Wire.setClock(400000L);
  Serial.println("\nI2C DAC MCP4725 and read voltage on PA2");

  // DAC initialization
  dac.begin(0x60);
  // Output Vctl to DAC, but do not write to DAC EEPROM 
  dac.setVoltage(adjusted_DAC_output, false); // min=0 max=4096 so 2048 should be 1/2 Vdd = approx. 1.65V    
    
  analogReadResolution(12); // make sure we read 12 bit values

  if (! aht.begin()) {
    Serial.println("Could not find AHT10? Check wiring");
    while (1) delay(10);
  }
  Serial.println("AHT10 found");

  // generate a test 2kHz square wave on PB9 PWM pin
  analogWriteFrequency(2000); // default PWM frequency is 1kHz, change it to 2kHz
  analogWrite(PB9, 127); // 127 means 50% duty cycle so a square wave

  // Finally: initialize BMP280
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }

  // Default settings from datasheet.
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // nothing required for internal sensors

  // SSD1306 I2C OLED display setup
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  // u8x8.setFont(u8x8_font_pressstart2p_f); // for some reason using this font locks up the display
  u8x8.drawString(0,0,"GPSDO - V.001m");
  u8x8.drawString(0,1,"F:            Hz");
  u8x8.drawString(0,2,"Uptime     days");
  u8x8.drawString(2,3,"00:00:00");
  u8x8.drawString(0,4,"GPS Lock Sats XX");
  u8x8.drawString(0,5,"Date");
  u8x8.drawString(0,6,"GMT time");
  u8x8.drawString(0,7,"T     C  ctl V");

  // Setup and start Timer 2 which measures OCXO frequency
  
  // setup pin used as ETR (10MHz external clock from OCXO)
  pinMode(PA15, INPUT_PULLUP);    // setup PA15 as input pin
  pinModeAF(PA15, GPIO_AF1_TIM2); // setup PA15 as TIM2 channel 1 / ETR
  
  // setup Timer 2 in input capture mode, active input channel 3
  // to latch counter value on rising edge

  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
  MyTim = new HardwareTimer(TIM2);
  
  // Configure rising edge detection to measure frequency
  MyTim->setMode(3, TIMER_INPUT_CAPTURE_RISING, PB10);

  // Configure 32-bit auto-reload register (ARR) with maximum possible value
  TIM2->ARR = 0xffffffff; // count to 2^32, then wraparound (approximately every 429 seconds)

  // select external clock source mode 2 by writing ECE=1 in the TIM2_SMCR register
  TIM2->SMCR |= TIM_SMCR_ECE; // 0x4000
  
  // start the timer
  MyTim->resume();
  
  delay(100); // not sure this delay is needed
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

static int32_t readVref()
{
#ifdef __LL_ADC_CALC_VREFANALOG_VOLTAGE
  return (__LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION));
#else
  return (VREFINT * ADC_RANGE / analogRead(AVREF)); // ADC sample to mV
#endif
}

static int32_t readTempSensor(int32_t VRef)
{
#ifdef __LL_ADC_CALC_TEMPERATURE
  return (__LL_ADC_CALC_TEMPERATURE(VRef, analogRead(ATEMP), LL_ADC_RESOLUTION));
#elif defined(__LL_ADC_CALC_TEMPERATURE_TYP_PARAMS)
  return (__LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(AVG_SLOPE, V25, CALX_TEMP, VRef, analogRead(ATEMP), LL_ADC_RESOLUTION));
#else
  return 0;
#endif
}

static int32_t readVoltage(int32_t VRef, uint32_t apin)
{
  return (__LL_ADC_CALC_DATA_TO_VOLTAGE(VRef, analogRead(apin), LL_ADC_RESOLUTION));
}

void displayInfo()
{
  Serial.print(F("Location:")); 
  if (gps.location.isValid())
  {
    Serial.print(F(" ALT=")); Serial.print(gps.altitude.meters()); 
    Serial.print(F(" LAT=")); Serial.print(gps.location.lat(),6); 
    Serial.print(F(" LON=")); Serial.print(gps.location.lng(),6); 
    sats = gps.satellites.value();
    char sats_str[3];
    strcpy(sats_str, u8x8_u8toa(sats, 2));
    u8x8.drawString(14,4,sats_str);
    Serial.print(F(" SATS=")); Serial.println(sats);
    gpsLock = true;
  }
  else
  {
    Serial.print(F(" INVALID"));
    gpsLock = false;
  }
  if (gpsLock) {
    u8x8.setCursor(4, 4); u8x8.print("Lock");
  } else {
    u8x8.setCursor(4, 4); u8x8.print("N/A ");
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());

    // note the use of u8x8.print() function
    u8x8.setCursor(6, 5);
    if (gps.date.day() < 10) u8x8.print("0");
    u8x8.print(gps.date.day());
    u8x8.print("/");
    if (gps.date.month() < 10) u8x8.print("0");
    u8x8.print(gps.date.month());
    u8x8.print("/");
    u8x8.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    GMTtime[0] = '0' + gps.time.hour() / 10;
    GMTtime[1] = '0' + gps.time.hour() % 10;
   
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    GMTtime[3] = '0' + gps.time.minute() / 10;
    GMTtime[4] = '0' + gps.time.minute() % 10;
   
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());

    // OCXO frequency
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
    // display OCXO frequency on OLED
    if (calcfreqint < 10000000) {
      u8x8.setCursor(3, 1); u8x8.print(" ");
    }
    else u8x8.setCursor(3, 1);
    u8x8.print(calcfreqint);
    u8x8.print("  ");

    
    
    // Serial.print(F("."));
    // if (gps.time.centisecond() < 10) Serial.print(F("0"));
    // Serial.print(gps.time.centisecond());
    GMTtime[6] = '0' + gps.time.second() / 10;
    GMTtime[7] = '0' + gps.time.second() % 10;
    u8x8.drawString(4,6,GMTtime);
  }
  else
  {
    Serial.print(F("INVALID"));
  }
    
  // send data from various sensors and change yellow led
  // status according to measured current
  int analogVal = analogRead(VctlInputPin);
  float Tbmp280 = bmp.readTemperature();

  float Vctl = (float(analogVal)/4096) * 3.3;
  Serial.print("Vctl: ");
  Serial.print(Vctl);
  Serial.print("  DAC: ");
  Serial.println(adjusted_DAC_output);
  u8x8.setCursor(9, 7);
  u8x8.print(Vctl,2);

  //get and print temperatures
    Serial.print(F("Temperature = "));
    Serial.print(Tbmp280);
    Serial.println(" *C");

    u8x8.setCursor(2, 7);
    u8x8.print(Tbmp280,1);

    Serial.print(F("Pressure = "));
    Serial.print((bmp.readPressure()+PressureOffset)/100);
    Serial.println(" hPa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude()); /* Adjusted to local forecast! */
    Serial.println(" m");

    Serial.println();

  // internal sensors
  Serial.print("VRef(mv)= ");
  int32_t VRef = readVref();
  Serial.print(VRef);
  Serial.print("\tTemp(°C)= ");
  Serial.print(readTempSensor(VRef));
  // on Black Pill, VBAT is connected to VDD - schottky diode drop and then 1/4 voltage divider inside STM32F411CEU6
  Serial.print("\tVbat(mv)= ");
  Serial.print(readVoltage(VRef, AVBAT));
  // A0 is connected to the user key on Black Pill
  // and a 1:2 voltage divider from Vcc
  Serial.print("\tVcc(V)= ");
  float Vcc;
  Vcc = 0.002 * readVoltage(VRef, A0);
  Serial.println(Vcc, 2);

  // update OLED display uptime
  uptime[0] = '0' + hours / 10;
  uptime[1] = '0' + hours % 10;
  uptime[3] = '0' + minutes / 10;
  uptime[4] = '0' + minutes % 10;
  uptime[6] = '0' + seconds / 10;
  uptime[7] = '0' + seconds % 10;
  u8x8.drawString(2,3,uptime); 
  // days
  u8x8.setCursor(9, 2);
  u8x8.print(days);

  // AHT10
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");

  /*
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");

  // INA219 measurements
  Serial.print("Bus voltage: ");
  Serial.print(imonitor.busVoltage());
  Serial.println("V");
  Serial.print("Shunt voltage: ");
  Serial.print(imonitor.shuntVoltage());
  Serial.println("mV");
  Serial.print("Shunt current: ");
  ocxo_current = imonitor.shuntCurrent();
  Serial.print(ocxo_current);
  Serial.println("mA");
  Serial.print("Bus power: ");
  Serial.print(imonitor.busPower());
  Serial.println("mW"); 
  Serial.println();

// update yellow_led_state according to current
// current less than 50mA -> led OFF
// current more than 300ma -> led blink
// otherwise led ON
  if (ocxo_current < 50) yellow_led_state = 0;
  else yellow_led_state = 1;
  if (ocxo_current > 300) yellow_led_state = 2;
  */
}

void calcavg() {
  // Calculate the OCXO frequency to 1, 2, 3 decimals only when the respective buffers are full
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

// Main loop
// ---------

void loop()
{   
  // the main loop displays information every time a new GPS sentence is correctly encoded.
  // (around 6 times per second)
  while (Serial1.available() > 0)
    if (gps.encode(Serial1.read())) {
      calcavg();
      displayInfo();
      Serial2.print("Hello World");
    }
    
  // every 400s of uptime, adjust DAC output up or down 1 bit (or leave it untouched)
  // depending on the 100s frequency average
  
  if (must_adjust_DAC) { // adjust DAC once every 400s
    if (avgfhun >= 10000000.01) {
      // decrease DAC by one bit
      adjusted_DAC_output--;
      dac.setVoltage(adjusted_DAC_output, false); // min=0 max=4096
      must_adjust_DAC = false;
    } else if (avgfhun <= 9999999.99) {
      // increase DAC by one bit
      adjusted_DAC_output++;
      dac.setVoltage(adjusted_DAC_output, false); // min=0 max=4096
      must_adjust_DAC = false;
    }
  } 
}
