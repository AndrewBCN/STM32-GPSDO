/*******************************************************************************************************
  STM32 GPSDO v0.04h by André Balsa, June 2021
  GPLV3 license
  Reuses small bits of the excellent GPS checker code Arduino sketch by Stuart Robinson - 05/04/20
  From version 0.03 includes a command parser, so the GPSDO can receive commands from the USB serial or
  Bluetooth serial interfaces and execute various callback functions.
  From version 0.04 includes an auto-calibration function, enabled by default at power on. The
  calibration process can also be launched at any time by sending the "C" command.
  The very first calibration after power on includes an OCXO warmup delay, usually 300 seconds.
  Version 0.04f implements a GPS receiver "tunnel mode" where the MCU simply relays the information
  when the "T" command is received.
  This should make it possible to connect the STM32 GPSDO to a laptop/PC running u-center.
  Note that tunnel mode is exited automatically after a configurable timeout. There is no other way to
  exit tunnel mode.

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

// GPSDO with optional I2C SSD1306 or SPI ST7735 display, STM32 MCU, DFLL in software

/*******************************************************************************************************
  This Arduino with STM32 Core package sketch implements a GPSDO with display option. It uses an SSD1306 
  128x64 I2C OLED display. It reads the GPS for 1 or 5 seconds and copies the half-dozen or so default
  NMEA sentences from the GPS to either the USB serial or Bluetooth serial ports (but not both) if
  verbose mode is enabled. That is followed by various sensors data and the FLL and OCXO data.
  This is an example printout from a working GPSDO running firmware version v0.04e:
   
    Wait for GPS fix max. 1 second

    $GNGSA,A,3,27,10,23,26,18,15,,,,,,,2.30,1.91,1.28*13
    $GNGSA,A9,76,38,268,,77,23,327,,84,05,085,,85,57,052,23*61
    $GLGSV,3,3,09,86,53,308,*5F
    $GNGLL,4833.66284,N,00746.88237,E,134626.00,A,A*72
    $GNRMC,134627.00,A,4833.66358,N,00746.88039,E,2.527,,050621,,,A*64
    $GNGGA,134627.00,4833.66358,N,00746.88039,E,1,07,1.91,137.7,M,47.3,M,,*46
    
    Fix time 889mS
    Uptime: 000d 00:28:44
    New GPS Fix: 
    Lat: 48.561058 Lon: 7.781340 Alt: 137.7m
    Sats: 7 HDOP: 1.91
    UTC Time: 13:46:27 Date: 5/6/2021
    
    Voltages: 
    Vctl: 1.97  DAC: 2404
    VctlPWM: 1.81  PWM: 35751
    Vcc: 5.02
    Vdd: 3.29
    
    Frequency measurements using 64-bit counter:
    64-bit Counter: 17215439735
    Frequency: 10000000 Hz
    10s Frequency Avg: 10000000.0 Hz
    100s Frequency Avg: 9999999.99 Hz
    1,000s Frequency Avg: 9999999.997 Hz
    10,000s Frequency Avg: 0.0000 Hz
    
    BMP280 Temperature = 26.6 *C
    Pressure = 1020.0 hPa
    Approx altitude = 57.3 m
    AHT10 Temperature: 23.57 *C
    Humidity: 76.48% rH

  When the program detects that the GPS has a fix, it prints the information above to the USB serial
  xor the Bluetooth serial (if the Bluetooth serial port is defined in the preprocessor directives).
  If the I2C OLED display is attached that is updated as well.

  The USB serial port is set at 115200 baud, the Bluetooth serial port at 57600 baud, and the GPS
  serial port is set initially at 9600 baud then reconfigured at 38400 baud.
*******************************************************************************************************/
/* Libraries required to compile:
    - TinyGPS++
    - U8g2/u8x8 graphics library
    - Adafruit AHTX0
    - Adafruit BMP280
    - Adafruit MCP4725 12-bit DAC library
    - movingAvg library, on STM32 architecture needs a simple patch to avoid warning during compilation
	- Adafruit_ST7735+GFX

    For commands parsing, uses SerialCommands library found here:
    https://github.com/ppedro74/Arduino-SerialCommands

   And also requires the installation of support for the STM32 MCUs by installing the STM32duino
   package (STM32 core version 2.0.0 or later).
*******************************************************************************************************/
/* Commands implemented:
    - V : returns program name, version and author
    - F : flush ring buffers
    - d/u p/d 1/10 : adjust Vctl down/up PWM/DAC fine/coarse, example dp1 means decrease PWM by 1.
    
/* Commands to be implemented:
    - L0 to L9 : select log levels
    - L0 : silence mode
    - L1 : fix only mode
    - L7 : fix and full status mode, no NMEA (default)
    - L8 : NMEA stream from GPS module only mode
    - L9 : NMEA + full status

/*******************************************************************************************************
  Program Operation -  This program is a GPSDO with optional OLED display. It uses a small SSD1306
  128x64 I2C OLED display. At startup the program starts checking the data coming from the GPS for a
  valid fix. It reads the GPS NMEA stream for 1/5 seconds and if there is no fix, prints a message on the
  Arduino IDE serial monitor and updates the seconds without a fix on the display. During this time the
  NMEA stream coming from the GPS is copied to the serial monitor also. The DFLL is active as soon as
  the GPS starts providing a 1PPS pulse. The 10MHz OCXO is controlled by a voltage generated by either
  the 16-bit PWM or the MCP4725 I2C DAC; this voltage (Vctl) is adjusted once every 429 seconds.
*******************************************************************************************************/

// Enabling the INA219 sensor using the LapINA219 library causes the firmware to lock up after a few minutes
// I have not identified the cause, it could be the library, or a hardware issue, or I have a bad sensor, etc.
// Requires further testing with another INA219 sensor, or another library.
// Note that leaving the INA219 sensor on the I2C bus without otherwise reading from / writing to it, does
// not cause any lock up.

// TODO when I have the time:
// 1. Solve the INA219 puzzle.
// 2. Refactor the setup and main loop functions to make them as simple as possible.

#define Program_Name "GPSDO"
#define Program_Version "v0.05i"
#define Author_Name "André Balsa"

// Define hardware options
// -----------------------
//#define GPSDO_OLED            // SSD1306 128x64 I2C OLED display
#define GPSDO_LCD             // ST7735 160x128 SPI LCD display
//#define GPSDO_MCP4725         // MCP4725 I2C 12-bit DAC
#define GPSDO_PWM_DAC         // STM32 16-bit PWM DAC, requires two rc filters (2xr=20k, 2xc=10uF)
//#define GPSDO_AHT10           // I2C temperature and humidity sensor
#define GPSDO_GEN_2kHz        // generate 2kHz square wave test signal on pin PB9 using Timer 4
#define GPSDO_BMP280_SPI      // SPI atmospheric pressure, temperature and altitude sensor
//#define GPSDO_INA219          // INA219 I2C current and Kvoltage sensor
//#define GPSDO_BLUETOOTH       // Bluetooth serial (HC-06 module)
#define GPSDO_VCC             // Vcc (nominal 5V) ; reading Vcc requires 1:2 voltage divider to PA0
#define GPSDO_VDD             // Vdd (nominal 3.3V) reads VREF internal ADC channel
#define GPSDO_CALIBRATION     // auto-calibration is enabled
#define GPSDO_UBX_CONFIG      // optimize u-blox GPS receiver configuration
//#define GPSDO_VERBOSE_NMEA    // GPS module NMEA stream echoed to USB serial xor Bluetooth serial
#define FastDebugMode        // reduce waiting times for checking SW, for SW release this shall be commented

// Includes
// --------
#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x02000000)
#error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x02000000"
#endif

// Increase HardwareSerial (UART) TX and RX buffer sizes from default 64 characters to 256.
// The main worry here is that we could miss some characters from the u-blox GPS module if
// the processor is busy doing something else (e.g. updating the display, reading a sensor, etc)
// specially since we increase the GPS baud rate from 9600 to 38400.

#define SERIAL_TX_BUFFER_SIZE 256       // Warning: > 256 could cause problems, see comments in STM32 HardwareSerial library
#define SERIAL_RX_BUFFER_SIZE 256

const uint16_t waitFixTime = 1;         // Maximum time in seconds waiting for a fix before reporting no fix / yes fix
                                        // Tested values 1 second and 5 seconds, 1s recommended


#include <movingAvg.h>                  // https://github.com/JChristensen/movingAvg , needs simple patch
                                        // to avoid warning message during compilation

#ifdef GPSDO_BLUETOOTH
//              UART    RX   TX
HardwareSerial Serial2(PA3, PA2);                  // Serial connection to HC-06 Bluetooth module
#endif // BLUETOOTH
#define BT_BAUD 57600                              // Bluetooth baud rate

#include <SerialCommands.h>                        // Commands parser library
char serial_command_buffer_[32];                   // buffer for commands library
// The following line determines which serial port we'll listen to
// "\n" means only newline needed to accept command
#ifdef GPSDO_BLUETOOTH
SerialCommands serial_commands_(&Serial2, serial_command_buffer_, sizeof(serial_command_buffer_), "\n", " ");
#else
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\n", " ");
#endif // BLUETOOTH

#include <TinyGPS++.h>                             // get library here > http://arduiniana.org/libraries/tinygpsplus/
TinyGPSPlus gps;                                   // create the TinyGPS++ object

#include <Wire.h>                                  // Hardware I2C library on STM32
                                                   // Uses PB6 (SCL1) and PB7 (SDA1) on Black Pill
#ifdef GPSDO_AHT10
#include <Adafruit_AHTX0.h>                        // Adafruit AHTX0 library
Adafruit_AHTX0 aht;                                // create object aht
#endif // AHT10

#ifdef GPSDO_INA219
#include <LapINA219.h>                             // LapINA219 library library
LapINA219 ina219(0x40);                            // create object ina219 with I2C address 0x40
float ina219volt=0.0, ina219curr=0.0;
TwoWire Wire3(PB4,PA8);                            // Second TwoWire instance for INA219 on SDA3/SCL3 (should be put somewhere more fitting but must stay global)
#endif // INA219

// OLED 0.96 SSD1306 128x64
// LCD 1.8" ST7735 160x128
#ifdef GPSDO_OLED
#include <U8x8lib.h>                                      // get library here >  https://github.com/olikraus/u8g2 
U8X8_SSD1306_128X64_NONAME_HW_I2C disp(U8X8_PIN_NONE);    // use this line for standard 0.96" SSD1306
#endif // OLED

#ifdef GPSDO_LCD
#include <Adafruit_GFX.h>       // need this adapted for STM32F4xx/F411C: https://github.com/fpistm/Adafruit-GFX-Library/tree/Fix_pin_type
#include <Adafruit_ST7735.h>
//#include <Fonts/FreeSansBold18pt7b.h>
#include <SPI.h>
#define TFT_DC  PA1
#define TFT_CS  PA2
#define TFT_RST PA3
// For 1.44" and 1.8" TFT with ST7735 use:
Adafruit_ST7735 disp = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
#endif // LCD

#ifdef GPSDO_MCP4725
#include <Adafruit_MCP4725.h>             // MCP4725 12-bit DAC Adafruit library
Adafruit_MCP4725 dac;
const uint16_t default_DAC_output = 2400; // 12-bit value, varies from OCXO to OCXO, and with aging and temperature
                                          // Some values I have been using, determined empirically:
                                          // 2603 for an ISOTEMP 143-141
                                          // 2549 for a CTI OSC5A2B02
                                          // 2400 for an NDK ENE3311B
                                          // 2180 for a second NDK ENE3311B
uint16_t adjusted_DAC_output;             // we adjust this value to "close the loop" of the DFLL when using the DAC
#endif // MCP4725

const uint16_t default_PWM_output = 35585; // "ideal" 16-bit PWM value, varies with OCXO, RC network, and time and temperature
                                           // 35585 for a second NDK ENE3311B
uint16_t adjusted_PWM_output;              // we adjust this value to "close the loop" of the DFLL when using the PWM
volatile bool must_adjust_DAC = false;     // true when there is enough data to adjust Vctl

#define VctlInputPin PB0              // ADC pin to read Vctl from DAC

#define VctlPWMOutputPin PB9          // digital output pin used to output a PWM value, TIM4 ch4
                                      // Two cascaded RC filters transform the PWM into an analog DC value
#define VctlPWMInputPin PB1           // ADC pin to read Vctl from filtered PWM

volatile int dacVctl = 0;             // DAC Vctl read by ADC pin PB0
volatile int pwmVctl = 0;             // PWM Vctl read by ADC pin PB1

#ifdef GPSDO_VCC
#define VccDiv2InputPin PA0           // Vcc/2 using resistor divider connects to PA0
int adcVcc = 0;
#endif // VCC

#ifdef GPSDO_VDD
int adcVdd = 0;                      // Vdd is read internally as Vref
#endif // VDD

// movingAvg objects for the voltages measured by MCU ADC
// all averages over 10 samples (10 seconds in principle)
#ifdef GPSDO_VDD
movingAvg avg_adcVdd(10);
int16_t avgVdd = 0;
#endif // VDD
#ifdef GPSDO_VCC
movingAvg avg_adcVcc(10);
int16_t avgVcc = 0;
#endif // VCC
movingAvg avg_dacVctl(10);
int16_t avgdacVctl = 0;
movingAvg avg_pwmVctl(10);
int16_t avgpwmVctl = 0;

#ifdef GPSDO_BMP280_SPI
// BMP280 - SPI
#include <SPI.h>
#include <Adafruit_BMP280.h>
#define BMP280_CS   (PA4)              // SPI1 uses PA4, PA5, PA6, PA7
Adafruit_BMP280 bmp(BMP280_CS);        // hardware SPI, use PA4 as Chip Select
const uint16_t PressureOffset = 1860;  // that offset must be calculated for your sensor and location
float bmp280temp=0.0, bmp280pres=0.0, bmp280alti=0.0; // read sensor, save here
#endif // BMP280_SPI

// LEDs
// Blue onboard LED blinks to indicate ISR is working
#define blueledpin  PC13    // Blue onboard LED is on PC13 on STM32F411CEU6 Black Pill
// Yellow extra LED is off, on or blinking to indicate some GPSDO status
#define yellowledpin PB8   // Yellow LED on PB8
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
char uptimestr[9] = "00:00:00";    // uptime string
char updaysstr[5] = "000d";        // updays string

char trendstr[5] = " ___";    // PWM trend string


// OCXO frequency measurement
volatile uint32_t lsfcount=0, previousfcount=0, calcfreqint=10000000;
/* Moving average frequency variables
   Basically we store the counter captures for 10 and 100 seconds.
   When the buffers are full, the average frequency is quite simply
   the difference between the oldest and newest data divided by the size
   of the buffer.
   Each second, when the buffers are full, we overwrite the oldest data
   with the newest data and calculate each average frequency.
 */
volatile uint64_t fcount64=0, prevfcount64=0, calcfreq64=10000000; 
// ATTENTION! must declare 64-bit, not 32-bit variable, because of shift
volatile uint64_t tim2overflowcounter = 0;  // testing, counts the number of times TIM2 overflows
volatile bool overflowflag = false;         // flag set by the overflow ISR, reset by the 2Hz ISR
volatile bool captureflag = false;          // flag set by the capture ISR, reset by the 2Hz ISR
volatile bool overflowErrorFlag = false;    // flag set if there was an overflow processing error

volatile uint64_t circbuf_ten64[11]; // 10+1 seconds circular buffer
volatile uint64_t circbuf_hun64[101]; // 100+1 seconds circular buffer
volatile uint64_t circbuf_tho64[1001]; // 1,000+1 seconds circular buffer
volatile uint64_t circbuf_tth64[10001]; // 10,000 + 1 seconds circular buffer

volatile uint32_t cbiten_newest=0; // index to oldest, newest data
volatile uint32_t cbihun_newest=0;
volatile uint32_t cbitho_newest=0;
volatile uint32_t cbitth_newest=0;

volatile bool cbTen_full=false, cbHun_full=false, cbTho_full=false, cbTth_full=false;  // flag when buffer full
volatile double avgften=0, avgfhun=0, avgftho=0, avgftth=0; // average frequency calculated once the buffer is full
volatile bool flush_ring_buffers_flag = true;  // indicates ring buffers should be flushed
volatile bool force_calibration_flag = true;   // indicates GPSDO should start calibration sequence
volatile bool ocxo_needs_warming = true;       // indicates OCXO needs to warm up a few minutes after power on
volatile bool tunnel_mode_flag = false;        // the GPSDO relays the information directly to and from the GPS module to the USB serial
#ifdef FastDebugMode
  const uint16_t tunnelSecs = 15;              // tunnel mode timeout in seconds; 15s for testing
  const uint16_t ocxo_warmup_time = 15;        // ocxo warmup time in seconds; 15s for testing
#else             
  const uint16_t tunnelSecs = 300;             // tunnel mode timeout in seconds; 300s or 600s normal use
  const uint16_t ocxo_warmup_time = 300;       // ocxo warmup time in seconds;  300s or 600s normal use
#endif  / FastDebugMode

// SerialCommands callback functions
// This is the default handler, and gets called when no other command matches. 
void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
  sender->GetSerial()->print("Unrecognized command [");
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println("]");
}

// called for V (version) command
void cmd_version(SerialCommands* sender)
{
  sender->GetSerial()->print(Program_Name);
  sender->GetSerial()->print(" - ");
  sender->GetSerial()->print(Program_Version);
  sender->GetSerial()->print(" by ");
  sender->GetSerial()->println(Author_Name);
}

// called for F (flush ring buffers) command
void cmd_flush(SerialCommands* sender)
{
  flush_ring_buffers_flag = true;  // ring buffers will be flushed inside interrupt routine
  sender->GetSerial()->println("Ring buffers flushed");
}

// called for C (calibration) command
void cmd_calibrate(SerialCommands* sender)
{
  force_calibration_flag = true;  // starts auto-calibration sequence
  sender->GetSerial()->println("Auto-calibration sequence started");
}

// called for T (tunnel) command
void cmd_tunnel(SerialCommands* sender)
{
  tunnel_mode_flag = true;  // switches GPSDO operation to tunnel mode
  sender->GetSerial()->println("Switching to USB Serial <-> GPS tunnel mode");
}

// called for up10 (increase PWM 10 bits) command
void cmd_up10(SerialCommands* sender)
{
  adjusted_PWM_output = adjusted_PWM_output + 10;
  analogWrite(VctlPWMOutputPin, adjusted_PWM_output);
  sender->GetSerial()->println("increased PWM 10 bits");
}

#ifdef GPSDO_MCP4725
// called for ud10 (increase DAC 10 bits) command
void cmd_ud10(SerialCommands* sender)
{
  adjusted_DAC_output = adjusted_DAC_output + 10;
  dac.setVoltage(adjusted_DAC_output, false);
  sender->GetSerial()->println("increased DAC 10 bits");
}

// called for dd10 (decrease DAC 10 bits) command
void cmd_dd10(SerialCommands* sender)
{
  adjusted_DAC_output = adjusted_DAC_output - 10;
  dac.setVoltage(adjusted_DAC_output, false);
  sender->GetSerial()->println("decreased DAC 10 bits");
}

// called for ud1 (increase DAC 1 bit) command
void cmd_ud1(SerialCommands* sender)
{
  adjusted_DAC_output = adjusted_DAC_output + 1;
  dac.setVoltage(adjusted_DAC_output, false);
  sender->GetSerial()->println("increased DAC 1 bit");
}

// called for dd1 (decrease DAC 1 bit) command
void cmd_dd1(SerialCommands* sender)
{
  adjusted_DAC_output = adjusted_DAC_output - 1;
  dac.setVoltage(adjusted_DAC_output, false);
  sender->GetSerial()->println("decreased DAC 1 bit");
}

#endif

// called for dp10 (decrease PWM 10 bits) command
void cmd_dp10(SerialCommands* sender)
{
  adjusted_PWM_output = adjusted_PWM_output - 10;
  analogWrite(VctlPWMOutputPin, adjusted_PWM_output);
  sender->GetSerial()->println("decreased PWM 10 bits");
}

// called for up1 (increase PWM 1 bit) command
void cmd_up1(SerialCommands* sender)
{
  adjusted_PWM_output = adjusted_PWM_output + 1;
  analogWrite(VctlPWMOutputPin, adjusted_PWM_output);
  sender->GetSerial()->println("increased PWM 1 bit");
}

// called for dp1 (decrease PWM 1 bit) command
void cmd_dp1(SerialCommands* sender)
{
  adjusted_PWM_output = adjusted_PWM_output - 1;
  analogWrite(VctlPWMOutputPin, adjusted_PWM_output);
  sender->GetSerial()->println("decreased PWM 1 bit");
}

//Note: Commands are case sensitive
SerialCommand cmd_version_("V", cmd_version);
SerialCommand cmd_flush_("F", cmd_flush);
SerialCommand cmd_calibrate_("C", cmd_calibrate);
SerialCommand cmd_tunnel_("T", cmd_tunnel);

// coarse adjust
SerialCommand cmd_up10_("up10", cmd_up10);
SerialCommand cmd_dp10_("dp10", cmd_dp10);

#ifdef GPSDO_MCP4725
SerialCommand cmd_ud10_("ud10", cmd_ud10);
SerialCommand cmd_dd10_("dd10", cmd_dd10);

SerialCommand cmd_ud1_("ud1", cmd_ud1);
SerialCommand cmd_dd1_("dd1", cmd_dd1);
#endif

// fine adjust
SerialCommand cmd_up1_("up1", cmd_up1);
SerialCommand cmd_dp1_("dp1", cmd_dp1);

// loglevel
uint8_t loglevel = 7;   // see commands comments for log level definitions, default is 7

// Interrupt service routines

// Interrupt Service Routine for TIM2 counter overflow / wraparound
void Timer2_Overflow_ISR(void)
{
  overflowflag = true;
}

// Interrupt Service Routine for TIM2 counter capture
void Timer2_Capture_ISR(void)
{
  captureflag = true;
}

// Interrupt Service Routine for the 2Hz timer
void Timer_ISR_2Hz(void) // WARNING! Do not attempt I2C communication inside the ISR

{ // Toggle pin. 2hz toogle --> 1Hz pulse, perfect 50% duty cycle
  digitalWrite(blueledpin, !digitalRead(blueledpin));

  halfsecond = !halfsecond; // true @ 1Hz

  // read TIM2->CCR3 once per second (when captureflag is set) and if it has changed, calculate OCXO frequency

  if (captureflag) {
    lsfcount = TIM2->CCR3; // read TIM2->CCR3
    captureflag = false;   // clear capture flag
    if (flush_ring_buffers_flag) 
    {
      flushringbuffers();   // flush ring buffers after a sat fix loss
    }
    else // check if the frequency counter has been updated and process accordingly
    { 
      // there are two possible cases
      // 1. lsfcount is the same as last time -> there is nothing to do, or
      // 2. lsfcount is NOT the same as last time -> process
      if (lsfcount != previousfcount)
      {
        // again we must consider two cases
        // 1. lsfcount < previousfcount -> a wraparound has occurred, process
        // 2. lsfcount > previous fcount -> no wraparound processing required
        if (lsfcount < previousfcount)
        {
          must_adjust_DAC = true; // set flag, once every wraparound / every 429s
          // check wraparound flag, it should be set, if so clear it, otherwise raise error flag
          tim2overflowcounter++;
          if (overflowflag) overflowflag=false; else overflowErrorFlag = true;
        }
        fcount64 = (tim2overflowcounter << 32) + lsfcount; // hehe now we have a 64-bit counter
        if (fcount64 > prevfcount64) {  // if we have a new count - that happens once per second
         if (((fcount64 - prevfcount64) > 9999500) && ((fcount64 - prevfcount64) < 10000500)) { // if we have a valid fcount, otherwise it's discarded
          logfcount64();  // save fcount in the 64-bit ring buffers
          calcfreq64 = fcount64 - prevfcount64; // the difference is exactly the OCXO frequency in Hz
         }
        prevfcount64 = fcount64;
        }
      }
      previousfcount = lsfcount; // this happens whether it has changed or not
    }                         
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

void logfcount64() // called once per second from ISR to update all the ring buffers
{
  // 10 seconds buffer
  circbuf_ten64[cbiten_newest]=fcount64;
  cbiten_newest++;
  if (cbiten_newest > 10) {
     cbTen_full=true; // this only needs to happen once, when the buffer fills up for the first time
     cbiten_newest = 0;   // (wrap around)
  }
  // 100 seconds buffer
  circbuf_hun64[cbihun_newest]=fcount64;
  cbihun_newest++;
  if (cbihun_newest > 100) {
     cbHun_full=true; // this only needs to happen once, when the buffer fills up for the first time
     cbihun_newest = 0;   // (wrap around)
  }
  // 1000 seconds buffer
  circbuf_tho64[cbitho_newest]=fcount64;
  cbitho_newest++;
  if (cbitho_newest > 1000) {
     cbTho_full=true; // this only needs to happen once, when the buffer fills up for the first time
     cbitho_newest = 0;   // (wrap around)
  }
  // 10000 seconds buffer (2 hr 46 min 40 sec)
  circbuf_tth64[cbitth_newest]=fcount64;
  cbitth_newest++;
  if (cbitth_newest > 10000) {
     cbTth_full=true; // this only needs to happen once, when the buffer fills up for the first time
     cbitth_newest = 0;   // (wrap around)
  }

  calcavg(); // always recalculate averages after logging fcount (if the respective buffers are full)
}

void calcavg() {
  // Calculate the OCXO frequency to 1, 2, 3 or 4 decimal places only when the respective buffers are full
  // Try to understand the algorithm for the 10s ring buffer first, the others work exactly the same
  
  uint64_t latfcount64, oldfcount64; // latest fcount, oldest fcount stored in ring buffer
  
  if (cbTen_full) { // we want (latest fcount - oldest fcount) / 10
    // latest fcount is always circbuf_ten64[cbiten_newest-1]
    // except when cbiten_newest is zero
    // oldest fcount is always circbuf_ten64[cbiten_newest] when buffer is full
    if (cbiten_newest == 0) latfcount64 = circbuf_ten64[10];
    else latfcount64 = circbuf_ten64[cbiten_newest-1];
    oldfcount64 = circbuf_ten64[cbiten_newest];
    // now that we have latfcount64 and oldfcount64 we can calculate the average frequency
    avgften = double(latfcount64 - oldfcount64)/10.0;   
  }  
  if (cbHun_full) { // we want (latest fcount - oldest fcount) / 100
    
    // latest fcount is always circbuf_hun[cbihun_newest-1]
    // except when cbihun_newest is zero
    // oldest fcount is always circbuf_hun[cbihun_newest] when buffer is full

    if (cbihun_newest == 0) latfcount64 = circbuf_hun64[100];
    else latfcount64 = circbuf_hun64[cbihun_newest-1];
    oldfcount64 = circbuf_hun64[cbihun_newest];
    
    avgfhun = double(latfcount64 - oldfcount64)/100.0;
  }
  if (cbTho_full) { // we want (latest fcount - oldest fcount) / 1000
    
    // latest fcount is always circbuf_tho[cbitho_newest-1]
    // except when cbitho_newest is zero
    // oldest fcount is always circbuf_tho[cbitho_newest] when buffer is full

    if (cbitho_newest == 0) latfcount64 = circbuf_tho64[1000];
    else latfcount64 = circbuf_tho64[cbitho_newest-1];
    oldfcount64 = circbuf_tho64[cbitho_newest];
    
    avgftho = double(latfcount64 - oldfcount64)/1000.0;
    // oldest fcount is always circbuf_ten[cbiten_newest-2]
    // except when cbiten_newest is <2 (zero or 1)
  } 
  if (cbTth_full) { // we want (latest fcount - oldest fcount) / 10000
    
    // latest fcount is always circbuf_tth[cbitth_newest-1]
    // except when cbitth_newest is zero
    // oldest fcount is always circbuf_tth[cbitth_newest] when buffer is full

    if (cbitth_newest == 0) latfcount64 = circbuf_tth64[10000];
    else latfcount64 = circbuf_tth64[cbitth_newest-1];
    oldfcount64 = circbuf_tth64[cbitth_newest];
    
    avgftth = double(latfcount64 - oldfcount64)/10000.0;
    // oldest fcount is always circbuf_ten[cbiten_newest-2]
    // except when cbiten_newest is <2 (zero or 1)
  } 
}

void flushringbuffers(void) {
  cbTen_full = false;
  cbHun_full = false;
  cbTho_full = false;
  cbTth_full = false;
  cbiten_newest = 0;
  cbihun_newest = 0;
  cbitho_newest = 0;
  cbitth_newest = 0;
  avgften = 0;
  avgfhun = 0;
  avgftho = 0;
  avgftth = 0;
  prevfcount64 = 0;
  previousfcount = 0;
  flush_ring_buffers_flag = false; // clear flag
}

void setup()
{
  // setup display at first place to please user
  // Initialize I2C 
  Wire.begin();
  // try setting a higher I2C clock speed
  Wire.setClock(400000L); 

  //#if defined GPSDO_OLED || defined GPSDO_LCD
  #ifdef GPSDO_OLED
  // Setup OLED I2C display
  // Note that u8x8 library initializes I2C hardware interface
  disp.setBusClock(400000L); // try to avoid display locking up
  disp.begin();
  disp.setFont(u8x8_font_chroma48medium8_r);
  disp.clear();                
  // splash screen
  disp.setCursor(3, 5);
  disp.print(F(Program_Name));
  disp.print(F(" - "));
  disp.print(F(Program_Version));
  #endif // OLED 
  
  #ifdef GPSDO_LCD
  // Setup LCD SPI display
  disp.initR(INITR_BLACKTAB); // 1.8" LDC
  //disp.setSPISpeed(50000000);    //default = 400...=23MHz; 5000...=46MHz
  delay(500);
  disp.fillScreen(ST7735_BLACK);
  disp.setTextColor(ST7735_YELLOW, ST7735_BLACK);  //
  disp.setRotation(1);   // 0..3 max, here we use 90° = landscape
  disp.setFont();
  disp.setTextSize(3);
  // splash screen
  disp.setCursor(40, 30);
  disp.print(F(Program_Name));
  disp.setTextSize(1);
  disp.setCursor(60, 65);
  //disp.print(F(" - "));
  disp.setTextColor(ST7735_WHITE, ST7735_BLACK);
  disp.print(F(Program_Version));
  
  
  disp.setTextColor(ST7735_WHITE, ST7735_BLACK);
  disp.setTextSize(1);
  //disp.setFont();
  #endif // OLED || LCD

  // Wait 1 second for things to stabilize
  delay(1000);
   
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
  Serial.begin(115200); // USB serial
  Serial1.begin(9600);  // Hardware serial 1 to GPS module
  #ifdef GPSDO_BLUETOOTH
  // HC-06 module baud rate factory setting is 9600, 
  // use separate program to set baud rate to 115200
  Serial2.begin(BT_BAUD); // Hardware serial 2 to Bluetooth module
  #endif // BLUETOOTH

  // setup commands parser
  serial_commands_.SetDefaultHandler(cmd_unrecognized);
  serial_commands_.AddCommand(&cmd_version_);
  serial_commands_.AddCommand(&cmd_flush_);
  serial_commands_.AddCommand(&cmd_calibrate_);
  serial_commands_.AddCommand(&cmd_tunnel_);

  serial_commands_.AddCommand(&cmd_up1_);
  serial_commands_.AddCommand(&cmd_dp1_);
  serial_commands_.AddCommand(&cmd_up10_);
  serial_commands_.AddCommand(&cmd_dp10_);

  #ifdef GPSDO_MCP4725  
    serial_commands_.AddCommand(&cmd_ud1_);
    serial_commands_.AddCommand(&cmd_dd1_);
    serial_commands_.AddCommand(&cmd_ud10_);
    serial_commands_.AddCommand(&cmd_dd10_);  
  #endif
  
  Serial.println();
  Serial.println(F(Program_Name));
  Serial.println(F(Program_Version));
  Serial.println();

  #ifdef GPSDO_UBX_CONFIG
  // Reconfigure the GPS receiver
  // first send the $PUBX configuration commands
  delay(3000); // give everything a moment to stabilize
  Serial.println("GPS checker program started");
  Serial.println("Sending $PUBX commands to GPS");  
  // first send the $PUBG configuration commands
  Serial1.print("$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n"); // disable all VTG messages (useless since we are stationary)
  Serial1.print("$PUBX,41,1,0003,0003,38400,0*24\r\n"); // set GPS baud rate to 38400 in/out protocols NMEA+UBX
  Serial1.flush();                              // empty the buffer
  delay(100);                                   // give it a moment
  Serial1.end();                                // close serial port
  Serial1.begin(38400);                         // re-open at new rate
  delay(3000);
  // second, send the proprietary UBX configuration commands
  Serial.println("Now sending UBX commands to GPS");
  ubxconfig();
  #endif // UBX_CONFIG
  
  #ifdef GPSDO_INA219
  ina219.begin(&Wire3);                           // calibrates ina219 sensor Edit: start the sensor on the third I2C controller
  Wire.setClock(400000L); 
  #endif // INA219 

  #ifdef GPSDO_MCP4725
  // Setup I2C DAC, read voltage on PB0
  adjusted_DAC_output = default_DAC_output; // initial DAC value
  dac.begin(0x60);
  // Output Vctl to DAC, but do not write to DAC EEPROM 
  dac.setVoltage(adjusted_DAC_output, false); // min=0 max=4096 so 2048 should be 1/2 Vdd = approx. 1.65V
  #endif
  
  // Make sure ADC resolution is 12-bit
  analogReadResolution(12);

  #ifdef GPSDO_AHT10
  if (! aht.begin()) {
    Serial.println("Could not find AHT10? Check wiring");
    while (1) delay(10);
  }
  Serial.println("AHT10 found");
  Wire.setClock(400000L); 
  #endif // AHT10

  // generate a 2kHz square wave on PB9 PWM pin, using Timer 4 channel 4
  // PB9 is Timer 4 Channel 4 from Arduino_Core_STM32/variants/STM32F4xx/F411C(C-E)(U-Y)/PeripheralPins_BLACKPILL_F411CE.c
  analogWrite(VctlPWMOutputPin, 127);      // configures PB9 as PWM output pin at default frequency and resolution
  analogWriteFrequency(2000); // default PWM frequency is 1kHz, change it to 2kHz
  analogWriteResolution(16);  // set PWM resolution to 16 bits (the maximum for the STM32F411CEU6)
  adjusted_PWM_output = default_PWM_output; // initial PWM value
  analogWrite(VctlPWMOutputPin, adjusted_PWM_output);    // 32767 for 16 bits -> 50% duty cycle so a square wave

  #ifdef GPSDO_BMP280_SPI
  // Initialize BMP280
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
  #endif // BMP280_SPI
  
  Serial.println(F("GPSDO Starting"));
  Serial.println();

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

  // Configure the ISR for the timer overflow interrupt
  FreqMeasTim->attachInterrupt(Timer2_Overflow_ISR);
  
  // Configure the ISR for the 1PPS capture
  FreqMeasTim->attachInterrupt(3, Timer2_Capture_ISR);  

  // select external clock source mode 2 by writing ECE=1 in the TIM2_SMCR register
  TIM2->SMCR |= TIM_SMCR_ECE; // 0x4000
  
  // start the timer
  FreqMeasTim->resume();

  // Initialize movingAvg objects (note this allocates space on heap) and immediately read 1st value
  #ifdef GPSDO_VDD
  avg_adcVdd.begin();
  adcVdd = analogRead(AVREF);
  avgVdd = avg_adcVdd.reading(adcVdd);
  # endif // VDD
  
  #ifdef GPSDO_VCC
  avg_adcVcc.begin();
  adcVcc = analogRead(VccDiv2InputPin);
  avgVcc = avg_adcVcc.reading(adcVcc);
  # endif // VCC
  
  avg_dacVctl.begin();
  dacVctl = analogRead(VctlInputPin);
  avgdacVctl = avg_dacVctl.reading(dacVctl);
  
  avg_pwmVctl.begin();
  pwmVctl = analogRead(VctlPWMInputPin);
  avgpwmVctl = avg_pwmVctl.reading(pwmVctl);

  startGetFixmS = millis();

  // setup done
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

#ifdef GPSDO_UBX_CONFIG
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
#endif // UBX_CONFIG

void loop()
{
  serial_commands_.ReadSerial();  // process any command from either USB serial (usually 
                                  // the Arduino monitor) xor Bluetooth serial (e.g. a smartphone)
  if (force_calibration_flag && gpsWaitFix(waitFixTime)) docalibration(); else

  if (tunnel_mode_flag) tunnelgps(); else
  
  if (gpsWaitFix(waitFixTime))    // wait up to waitFixTime seconds for fix, returns true if we have a fix
  {
    #ifdef GPSDO_BLUETOOTH
    Serial2.println();
    Serial2.println();
    Serial2.print(F("Fix time "));
    Serial2.print(endFixmS - startGetFixmS);
    Serial2.println(F("mS"));
    #else
    Serial.println();
    Serial.println();
    Serial.print(F("Fix time "));
    Serial.print(endFixmS - startGetFixmS);
    Serial.println(F("mS"));
    #endif // BLUETOOTH

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


    if (must_adjust_DAC && cbHun_full) // in principle just once every 429 seconds, and only if we have valid data
    {
      // use different algorithms for 12-bit I2C DAC and STM32 16-bit PWM DAC
      #ifdef GPSDO_MCP4725
      adjustVctlDAC();  
      #endif // MCP4725
      #ifdef GPSDO_PWM_DAC
      adjustVctlPWM();
      #endif // PWM_DAC
    }

    dacVctl = analogRead(VctlInputPin);         // read the Vctl voltage output by the DAC
    avgdacVctl = avg_dacVctl.reading(dacVctl);  // average it
    
    pwmVctl = analogRead(VctlPWMInputPin);      // read the filtered Vctl voltage output by the PWM
    avgpwmVctl = avg_pwmVctl.reading(pwmVctl);  // average it

    #ifdef GPSDO_VCC
    adcVcc = analogRead(VccDiv2InputPin);       // read Vcc
    avgVcc = avg_adcVcc.reading(adcVcc);        // average it
    # endif // VCC

    #ifdef GPSDO_VDD
    adcVdd = analogRead(AVREF);                 // Vdd is read internally as Vref
    avgVdd = avg_adcVdd.reading(adcVdd);        // average it
    #endif // VDD    

    #ifdef GPSDO_BMP280_SPI
    bmp280temp = bmp.readTemperature();              // read bmp280 sensor, save values
    bmp280pres = bmp.readPressure();
    bmp280alti = bmp.readAltitude();
    #endif // BMP280_SPI    

    #ifdef GPSDO_INA219
    ina219volt = ina219.busVoltage();                // read ina219 sensor, save values
    ina219curr = ina219.shuntCurrent();
    #endif // INA219 
  
    uptimetostrings();           // get updaysstr and uptimestr

    yellow_led_state = 0;        // turn off yellow LED

    #ifdef GPSDO_BLUETOOTH 
    printGPSDOstats(Serial2);   // print stats to Bluetooth Serial
    #else                       // xor
    printGPSDOstats(Serial);    // print stats to USB Serial
    #endif // BLUETOOTH

	#ifdef GPSDO_OLED
    displayscreen_OLED();
    #endif // OLED
	
	#ifdef GPSDO_LCD
    displayscreen_LCD();
    #endif // LCD

    startGetFixmS = millis();    // have a fix, next thing that happens is checking for a fix, so restart timer
  }
  else // no GPS fix could be acquired for the last five seconds
  {
    yellow_led_state = 1;        // turn on yellow LED
    // display no fix message on Display
    //#ifdef GPSDO_OLED
    #if defined GPSDO_OLED || defined GPSDO_LCD
    #ifdef GPSDO_OLED
	disp.clear();                
	disp.setCursor(0, 0);
	#endif // OLED
    #ifdef GPSDO_LCD
	disp.fillScreen(ST7735_BLACK);
	disp.setCursor(0, 0);
	#endif // LCD
	disp.print(F(Program_Name));
    disp.print(F(" - "));
    disp.print(F(Program_Version));
    #ifdef GPSDO_OLED
    disp.setCursor(0, 1);
    #endif // OLED
    #ifdef GPSDO_LCD
	disp.setCursor(0, 8);
    #endif // LCD
    disp.print(F("Wait fix "));
    disp.print( (millis() - startGetFixmS) / 1000 );
    disp.print(F("s"));
    #endif // OLED
    
    #ifdef GPSDO_BLUETOOTH      // print no fix message to either
    Serial2.println();          // Bluetooth serial or USB serial
    Serial2.print(F("Waiting for GPS Fix "));
    Serial2.print( (millis() - startGetFixmS) / 1000 );
    Serial2.println(F("s"));
    #else
    Serial.println();
    Serial.print(F("Waiting for GPS Fix "));
    Serial.print( (millis() - startGetFixmS) / 1000 );
    Serial.println(F("s"));
    #endif // BLUETOOTH

    // no fix, raise flush_ring_buffers_flag
    flush_ring_buffers_flag = true;
  }
}
// end of loop

void tunnelgps()
// GPSDO tunnel mode operation
{
  #ifdef GPSDO_BLUETOOTH      // print calibrating started message to either
  Serial2.println();          // Bluetooth serial xor USB serial
  Serial2.print(F("Entering tunnel mode..."));
  Serial2.println();
  #else
  Serial.println();
  Serial.print(F("Entering tunnel mode..."));
  Serial.println();
  #endif // BLUETOOTH

  // tunnel mode operation goes here
  uint32_t endtunnelmS = millis() + (tunnelSecs * 1000);
  uint8_t GPSchar;
  uint8_t PCchar;
  while (millis() < endtunnelmS)
  {
    if (Serial1.available() > 0)
    {
      GPSchar = Serial1.read();
      Serial.write(GPSchar);  // echo NMEA stream to USB serial
    }
    if (Serial.available() > 0)
    {
      PCchar = Serial.read();
      Serial1.write(PCchar);  // echo PC stream to GPS serial
    }
  }
  // tunnel mode operation ends here
  
  #ifdef GPSDO_BLUETOOTH      // print calibrating started message to either
  Serial2.println();          // Bluetooth serial xor USB serial
  Serial2.print(F("Tunnel mode exited."));
  Serial2.println();
  #else
  Serial.println();
  Serial.print(F("Tunnel mode exited."));
  Serial.println();
  #endif // BLUETOOTH
  
  tunnel_mode_flag = false; // reset flag, exit tunnel mode
}

void docalibration()
// GPSDO calibration routine
{
  unsigned long startWarmup = millis(); // we need a rough timer
  if (ocxo_needs_warming) {
    // spend a few seconds/minutes here waiting for the OCXO to warm
    // show countdown timer on OLED display
    // and report on either USB serial or Bluetooth serial
    // Note: during calibration the GPSDO does not accept any commands
    uint16_t countdown = ocxo_warmup_time;
	
    // display warmup message on OLED
    //#ifdef GPSDO_OLED
    #if defined GPSDO_OLED || defined GPSDO_LCD              
    #ifdef GPSDO_OLED
    disp.clear();                
    #endif // OLED
    #ifdef GPSDO_LCD
    disp.fillScreen(ST7735_BLACK);
    #endif // LCD
    disp.setCursor(0, 0);
    disp.print(F(Program_Name));
    disp.print(F(" - "));
    disp.print(F(Program_Version));
    #ifdef GPSDO_OLED
    disp.setCursor(0, 2);
    #endif // OLED
    #ifdef GPSDO_LCD
    disp.setCursor(0, 16);
    #endif // LCD
    disp.print(F("OCXO warming up"));
    #ifdef GPSDO_OLED
    disp.setCursor(0, 3);
    #endif // OLED
    #ifdef GPSDO_LCD
    disp.setCursor(0, 24);
    #endif // LCD
    disp.print(F("Please wait"));

    while (countdown) {
          yellow_led_state = 2;        // blink yellow LED
		  
	      // display warmup message on display
		  #ifdef GPSDO_OLED
          disp.setCursor(0, 4);
	      disp.clearLine(4);                
          #endif // OLED
          #ifdef GPSDO_LCD
          disp.setCursor(0, 32);
	      #endif // LCD
          disp.print(countdown);
          disp.print(F("s    "));
          #endif // OLED || LCD
          
          #ifdef GPSDO_BLUETOOTH      // print warming up message to either
          Serial2.println();          // Bluetooth serial xor USB serial
          Serial2.print(F("Warming up "));
          Serial2.print(countdown);
          Serial2.println(F("s"));
          #else
          Serial.println();
          Serial.print(F("Warming up "));
          Serial.print(countdown);
          Serial.println(F("s"));
          #endif // BLUETOOTH

          // do nothing for 1s
          delay(1000);
          countdown--;
    }
    ocxo_needs_warming = false; // reset flag, next "hot" calibration skips ocxo warmup 
  }
  // proceed with calibration
  #ifdef GPSDO_BLUETOOTH      // print calibrating started message to either
  Serial2.println();          // Bluetooth serial xor USB serial
  Serial2.print(F("Calibrating..."));
  Serial2.println();
  #else
  Serial.println();
  Serial.print(F("Calibrating..."));
  Serial.println();
  #endif // BLUETOOTH

   // display calibrating message on OLED
  //#ifdef GPSDO_OLED
  #if defined GPSDO_OLED || defined GPSDO_LCD
  #ifdef GPSDO_OLED
  disp.clear();                
  #endif // OLED
  #ifdef GPSDO_LCD
  disp.fillScreen(ST7735_BLACK);
  #endif // LCD
  disp.setCursor(0, 0);
  disp.print(F(Program_Name));
  disp.print(F(" - "));
  disp.print(F(Program_Version));
  #ifdef GPSDO_OLED
  disp.setCursor(0, 2);
  #endif // OLED
  #ifdef GPSDO_LCD
  disp.setCursor(0, 16);
  #endif // LCD
  disp.print(F("Calibrating..."));
  #ifdef GPSDO_OLED
  disp.setCursor(0, 3);
  #endif // OLED
  #ifdef GPSDO_LCD
  disp.setCursor(0, 24);
  #endif // LCD
  disp.print(F("Please wait"));
  #endif // OLED | LCD

  /*  The calibration algorithm
   *  The objective of the calibration is to find the approximate Vctl to obtain
   *  10MHz +/- 0.1Hz.
   *  
   *  we can use either a PID algorithm or a simple linear interpolation algorithm
   *  
   *  The following describes a simple linear interpolation algorithm
   *  
   *  We first output 1.5V for the DAC or PWM, wait 30 seconds and note the 10s frequency average.
   *  Next we output 2.5V for the DAC or PWM, wait 30 seconds and note the 10s frequency average.
   *  Now we calculate the Vctl for 10MHz +/- 0.1Hz using linear interpolation between the two points.
   */
  // for 12-bit DAC
  // 1.5V for DAC = 4096 x (1.5 / 3.3) = 1862 results in frequency f1 = 10MHz + e1
  // 2.5V for DAC = 4096 x (2.5 / 3.3) = 3103 results in frequency f2 = 10MHz + e2
  // for 16-bit PWM
  // 1.5V for PWM = 65536 x (1.5 / 3.2) = 30720 results in frequency f1 = 10MHz + e1
  // 2.5V for PWM = 65536 x (2.5 / 3.2) = 51200 results in frequency f2 = 10MHz + e2
  // where f2 > f1 (most OCXOs have positive slope).
  double f1, f2, e1, e2;
  // make sure we have a fix and data
  while (!cbTen_full) delay(1000);
  // measure frequency for Vctl=1.5V
  
  //#ifdef GPSDO_OLED  
  #if defined GPSDO_OLED || defined GPSDO_LCD 
  #ifdef GPSDO_OLED  
  disp.setCursor(0, 3);
  disp.clearLine(3);
  #endif // OLED
  #ifdef GPSDO_LCD
  disp.setCursor(0, 24);
  #endif // LCD
  disp.print(F("Vctl: 1.5V / 15s"));
  #endif // OLED | LCD
  
  Serial.println(F("Setting PWM to 1.5V, wait 15s"));
  analogWrite(VctlPWMOutputPin, 30720);
  delay(15000);
  Serial.print(F("f1 (average frequency for 1.5V Vctl): "));
  f1 = avgften;
  
  //#ifdef GPSDO_OLED  
  #if defined GPSDO_OLED || defined GPSDO_LCD
  #ifdef GPSDO_OLED  
  disp.setCursor(0, 4);
  disp.clearLine(4);
  #endif // OLED
  #ifdef GPSDO_LCD
  disp.setCursor(0, 32);
  #endif // LCD  
  disp.print(F("F1:"));
  disp.print(f1,1);
  disp.print(F(" Hz"));
  #endif // OLED | LCD
  
  Serial.print(f1,1);
  Serial.println(F(" Hz"));
  // make sure we have a fix and data again
  while (!cbTen_full) delay(1000);
  // measure frequency for Vctl=2.5V
  
  //#ifdef GPSDO_OLED  
  #if defined GPSDO_OLED || defined GPSDO_LCD
  #ifdef GPSDO_OLED  
  disp.setCursor(0, 5);
  disp.clearLine(5);
  #endif // OLED
  #ifdef GPSDO_LCD
  disp.setCursor(0, 40);
  #endif // LCD  
  disp.print(F("Vctl: 2.5V / 15s"));
  #endif // OLED | LCD
  
  Serial.println(F("Setting PWM to 2.5V, wait 15s"));
  analogWrite(VctlPWMOutputPin, 51200);
  delay(15000);
  Serial.print(F("f2 (average frequency for 2.5V Vctl): "));
  f2 = avgften;
  
  //#ifdef GPSDO_OLED 
  #if defined GPSDO_OLED || defined GPSDO_LCD  
  #ifdef GPSDO_OLED 
  disp.setCursor(0, 6);
  disp.clearLine(6);
  #endif // OLED
  #ifdef GPSDO_LCD
  disp.setCursor(0, 48);
  #endif // LCD  
  disp.print(F("F2:"));
  disp.print(f2,1);
  disp.print(F(" Hz"));
  #endif // OLED | LCD
  
  Serial.print(f2,1);
  Serial.println(F(" Hz"));
  // slope s is (f2-f1) / (51200-30720) for PWM
  // So F=10MHz +/- 0.1Hz for PWM = 30720 - (e1 / s)
  // set Vctl
  // adjusted_PWM_output = formula
  adjusted_PWM_output = 30720 - ((f1 - 10000000.0) / ((f2 - f1) / 20480));
  Serial.print(F("Calculated PWM: "));
  Serial.println(adjusted_PWM_output);

  //#ifdef GPSDO_OLED
  #if defined GPSDO_OLED || defined GPSDO_LCD
  #ifdef GPSDO_OLED 
  disp.clearLine(7);
  disp.setCursor(0, 7);
  #endif // OLED
  #ifdef GPSDO_LCD
  disp.setCursor(0, 56);
  #endif // LCD  
  disp.print(F("Calc. PWM:"));
  disp.print(adjusted_PWM_output,1);
  disp.print(F(" "));
  //float Vctlp = (float(avgpwmVctl)/4096) * 3.3;  // this des not make sense here
  //disp.print(Vctlp);
  //disp.print(F("V "));
  delay(5000);                          // Wait for 5 second (so You can read what's written on OLED display
  #ifdef GPSDO_OLED
  disp.clear();                
  #endif // OLED
  #ifdef GPSDO_LCD
  disp.fillScreen(ST7735_BLACK);
  #endif // LCD
  disp.setCursor(0, 0);
  disp.print(F(Program_Name));
  disp.print(F(" - "));
  disp.print(F(Program_Version));                         
  #endif // OLED | LCD

  analogWrite(VctlPWMOutputPin, adjusted_PWM_output); 
  // calibration done
  
  #ifdef GPSDO_BLUETOOTH      // print calibration finished message to either
  Serial2.println();          // Bluetooth serial xor USB serial
  Serial2.print(F("Calibration done."));
  Serial2.println();
  #else
  Serial.println();
  Serial.print(F("Calibration done."));
  Serial.println();
  #endif // BLUETOOTH
  
  force_calibration_flag = false; // reset flag, calibration done
}

#ifdef GPSDO_MCP4725
void adjustVctlDAC()
// This should reach a stable DAC output value / a stable 10000000.00 frequency
// after an hour or so
{
  // decrease frequency
  if (avgfhun >= 10000000.01) {
    if (avgfhun >= 10000000.10) {
      // decrease DAC by ten bits = coarse
      adjusted_DAC_output = adjusted_DAC_output - 10;
	    strcpy(trendstr, " c- ");
    } else {
      // decrease DAC by one bit = fine
      adjusted_DAC_output--;
	    strcpy(trendstr, " f- ");
    }
    dac.setVoltage(adjusted_DAC_output, false); // min=0 max=4096
  } 
  // or increase frequency
  else if (avgfhun <= 9999999.99) {
    if (avgfhun <= 9999999.90) {
      // increase DAC by ten bits = coarse
      adjusted_DAC_output = adjusted_DAC_output + 10;      
	    strcpy(trendstr, " c+ ");
    } else {
      // increase DAC by one bit = fine
      adjusted_DAC_output++;
	    strcpy(trendstr, " f+ ");
    }
    dac.setVoltage(adjusted_DAC_output, false); // min=0 max=4096
  }
  // or do nothing because avgfrequency over last 100s is 10000000.00Hz
  must_adjust_DAC = false; // clear flag and we are done
}    // end adjustVctlDAC
#endif

#ifdef GPSDO_PWM_DAC
void adjustVctlPWM()
// This should reach a stable PWM output value / a stable 10000000.00 frequency
// after an hour or so, and 10000000.000 after eight hours or so
{
  // check first if we have the data, then do ultrafine and veryfine frequency
  // adjustment, when we are very close
  // ultimately the objective is 10000000.000 over the last 1000s (16min40s)
  if ((cbTho_full) && (avgftho >= 9999999.990) && (avgftho <= 10000000.010)) { 
    
    // decrease frequency; 1000s based
    if (avgftho >= 10000000.001) {
      if (avgftho >= 10000000.005) {
        // decrease PWM by 5 bits = very fine
        adjusted_PWM_output = adjusted_PWM_output - 5;
	    strcpy(trendstr, " vf-");
        } 
		else {
        // decrease PWM by one bit = ultrafine
        adjusted_PWM_output = adjusted_PWM_output - 1;
	    strcpy(trendstr, " uf-");
        }
    } 
    // or increase frequency; 1000s based
    else if (avgftho <= 9999999.999) {
      if (avgftho <= 9999999.995) {
       // increase PWM by 5 bits = very fine
        adjusted_PWM_output = adjusted_PWM_output + 5;      
	    strcpy(trendstr, " vf+");
        } 
		else {
        // increase PWM by one bit = ultrafine
        adjusted_PWM_output = adjusted_PWM_output + 1;
	    strcpy(trendstr, " uf+");
      }
    }
  }
	///// next check the 100s values in second place because we are too far off
	// decrease frequency; 100s based
  else if (avgfhun >= 10000000.01) {
    if (avgfhun >= 10000000.10) {
      // decrease PWM by 100 bits = coarse
      adjusted_PWM_output = adjusted_PWM_output - 100;
	  strcpy(trendstr, " c- ");
      } 
	  else {
      // decrease PWM by ten bits = fine
      adjusted_PWM_output = adjusted_PWM_output - 10;
 	  strcpy(trendstr, " f- ");
      }
  } 
  // or increase frequency; 100s based
  else if (avgfhun <= 9999999.99) {
    if (avgfhun <= 9999999.90) {
     // increase PWM by 100 bits = coarse
      adjusted_PWM_output = adjusted_PWM_output + 100;      
	  strcpy(trendstr, " c+ ");
    } 
	else {
    // increase PWM by ten bits = fine
      adjusted_PWM_output = adjusted_PWM_output + 10;
      strcpy(trendstr, " f+ "); 
	  }
  }
  else {    // here we keep setting, because it is exact 10000000.000MHz
	  strcpy(trendstr, " hit");
  } 
  // write the computed value to PWM
  analogWrite(VctlPWMOutputPin, adjusted_PWM_output);
  must_adjust_DAC = false; // clear flag and we are done  
  }      // end adjustVctlPWM
#endif // GPSDO_PWM_DAC


bool gpsWaitFix(uint16_t waitSecs)
{
  // waits a specified number of seconds for a fix,
  // returns true as soon as fix available or false on timeout

  uint32_t endwaitmS;
  uint8_t GPSchar;

  #ifdef GPSDO_BLUETOOTH
  Serial2.println();
  //Serial2.print(F("Wait for GPS fix max. "));
  //Serial2.print(waitSecs);
  //if (waitSecs > 1) Serial2.println(F(" seconds")); else Serial2.println(F(" second"));
  #else
  Serial.println();
  //Serial.print(F("Wait for GPS fix max. "));
  //Serial.print(waitSecs);
  //if (waitSecs > 1) Serial.println(F(" seconds")); else Serial.println(F(" second"));
  #endif // Bluetooth

  endwaitmS = millis() + (waitSecs * 1000);

  while (millis() < endwaitmS)
  {
    if (Serial1.available() > 0)
    {
      GPSchar = Serial1.read();
      gps.encode(GPSchar);
      #ifdef GPSDO_VERBOSE_NMEA
      #ifdef GPSDO_BLUETOOTH
      Serial2.write(GPSchar); // echo NMEA stream to Bluetooth serial
      #else
      Serial.write(GPSchar);  // echo NMEA stream to USB serial
      #endif // Bluetooth
      #endif // VERBOSE_NMEA
    }

    if (gps.location.isUpdated() && gps.altitude.isUpdated() && gps.date.isUpdated())
    {
      endFixmS = millis();                                //record the time when we got a GPS fix
      return true;
    }
  }
  return false;
}


void printGPSDOstats(Stream &Serialx) 
{
  float tempfloat;
  
  Serialx.print(F("Uptime: "));
  Serialx.print(updaysstr);
  Serialx.print(F(" "));
  Serialx.println(uptimestr);
  
  Serialx.println(F("New GPS Fix: "));

  tempfloat = ( (float) GPSHdop / 100);

  Serialx.print(F("Lat: "));
  Serialx.print(GPSLat, 6);
  Serialx.print(F(" Lon: "));
  Serialx.print(GPSLon, 6);
  Serialx.print(F(" Alt: "));
  Serialx.print(GPSAlt, 1);
  Serialx.println(F("m"));
  Serialx.print(F("Sats: "));
  Serialx.print(GPSSats);
  Serialx.print(F(" HDOP: "));
  Serialx.println(tempfloat, 2);
  Serialx.print(F("UTC Time: "));

  if (hours < 10)
  {
    Serialx.print(F("0"));
  }

  Serialx.print(hours);
  Serialx.print(F(":"));

  if (mins < 10)
  {
    Serialx.print(F("0"));
  }

  Serialx.print(mins);
  Serialx.print(F(":"));

  if (secs < 10)
  {
    Serialx.print(F("0"));
  }

  Serialx.print(secs);
  Serialx.print(F(" Date: "));

  Serialx.print(day);
  Serialx.print(F("/"));
  Serialx.print(month);
  Serialx.print(F("/"));
  Serialx.println(year);

  Serialx.println();
  Serialx.println(F("Voltages: "));

  #ifdef GPSDO_MCP4725
  float Vctl = (float(avgdacVctl)/4096) * 3.3;
  Serialx.print("VctlDAC: ");
  Serialx.print(Vctl);
  Serialx.print("  DAC: ");
  Serialx.println(adjusted_DAC_output);
  #endif

  float Vctlp = (float(avgpwmVctl)/4096) * 3.3;
  Serialx.print("VctlPWM: ");
  Serialx.print(Vctlp);
  Serialx.print("  PWM: ");
  Serialx.println(adjusted_PWM_output);

  #ifdef GPSDO_VCC
  // Vcc/2 is provided on pin PA0
  float Vcc = (float(avgVcc)/4096) * 3.3 * 2.0;
  Serialx.print("Vcc: ");
  Serialx.println(Vcc);
  #endif // VCC

  #ifdef GPSDO_VDD
  // internal sensor Vref
  float Vdd = (1.21 * 4096) / float(avgVdd); // from STM32F411CEU6 datasheet
  Serialx.print("Vdd: ");                    // Vdd = Vref on Black Pill
  Serialx.println(Vdd);
  #endif // VDD

  #ifdef GPSDO_INA219
  // current sensor for the OCXO
  Serialx.print(F("OCXO voltage: "));
  Serialx.print(ina219volt, 2);
  Serialx.println(F("V"));
  Serialx.print(F("OCXO current: "));
  Serialx.print(ina219curr, 0);
  Serialx.println(F("mA"));
  #endif // INA219 
      
  // OCXO frequency measurements
  Serialx.println();
  Serialx.println(F("Frequency measurements using 64-bit counter:"));
  // temporarily added to check proper 16-bit PWM operation
  // Serialx.print(F("TIM4 ARR: ")); // should in principle be 48000-1, because 96MHz / 2kHz = 48000
  // Serialx.println(TIM4->ARR); // and yes, verified
  // Serialx.print(F("TIM4 ch4 CCR: ")); // in principle, the PWM value x 48000/65536
  // Serialx.println(TIM4->CCR4); // and also yes, verified
  // end of temp code
  // if (overflowErrorFlag) Serialx.println(F("ERROR: overflow "));
  // Serialx.print(F("Most Significant 32 bits (OverflowCounter): "));
  // Serialx.println(tim2overflowcounter);
  // Serialx.print(F("Least Significant 32 bits (TIM2->CCR3): "));
  // Serialx.println(lsfcount);
  Serialx.print(F("64-bit Counter: "));
  Serialx.println(fcount64);
  Serialx.print(F("Frequency: "));
  Serialx.print(calcfreq64);
  Serialx.println(F(" Hz"));
  Serialx.print("10s Frequency Avg: ");
  Serialx.print(avgften,1);
  Serialx.println(F(" Hz"));
  Serialx.print("100s Frequency Avg: ");
  Serialx.print(avgfhun,2);
  Serialx.println(F(" Hz"));
  Serialx.print("1,000s Frequency Avg: ");
  Serialx.print(avgftho,3);
  Serialx.println(F(" Hz"));
  Serialx.print("10,000s Frequency Avg: ");
  Serialx.print(avgftth,4);
  Serialx.println(F(" Hz"));  

  #ifdef GPSDO_BMP280_SPI
  // BMP280 measurements
  Serialx.println();
  Serialx.print(F("BMP280 Temperature = "));
  Serialx.print(bmp280temp, 1);
  Serialx.println(" *C");
  Serialx.print(F("Pressure = "));
  Serialx.print((bmp280pres+PressureOffset)/100, 1);
  Serialx.println(" hPa");
  Serialx.print(F("Approx altitude = "));
  Serialx.print(bmp280alti, 1); /* Adjusted to local forecast! */
  Serialx.println(" m");
  #endif // BMP280_SPI

  #ifdef GPSDO_AHT10
  // AHT10 measurements
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  Serialx.print("AHT10 Temperature: ");
  Serialx.print(temp.temperature);
  Serialx.println(" *C");
  Serialx.print("Humidity: ");
  Serialx.print(humidity.relative_humidity);
  Serialx.println("% rH");
  #endif // AHT10
  
  Serialx.println();
}

void uptimetostrings() {
  // translate uptime variables to strings
  uptimestr[0] = '0' + uphours / 10;
  uptimestr[1] = '0' + uphours % 10;
  uptimestr[3] = '0' + upminutes / 10;
  uptimestr[4] = '0' + upminutes % 10;
  uptimestr[6] = '0' + upseconds / 10;
  uptimestr[7] = '0' + upseconds % 10;
 
  if (updays > 99) { // 100 days or more
    updaysstr[0] = '0' + updays / 100;
    updaysstr[1] = '0' + (updays % 100) / 10;
    updaysstr[2] = '0' + (updays % 100) % 10;
  }
  else { // less than 100 days
    updaysstr[0] = '0';
    updaysstr[1] = '0' + updays / 10;
    updaysstr[2] = '0' + updays % 10;
  }
}

#ifdef GPSDO_OLED
void displayscreen_OLED()
{
  //show GPSDO data on OLED display
  float tempfloat;

  // OCXO frequency
  disp.setCursor(0, 1);
  disp.print(F("F:"));
  // display 1s, 10s or 100s value depending on whether data is available
  if (cbTen_full) {
    if (cbHun_full) { // if we have data over 100 seconds
      if (avgfhun < 10000000) {
        disp.setCursor(2, 1); disp.print(" ");
      }
      else disp.setCursor(2, 1);
      disp.print(avgfhun, 2); // to 2 decimal places
      disp.print("Hz ");
    }
    else { // nope, only 10 seconds
      if (avgften < 10000000) {
        disp.setCursor(2, 1); disp.print(" ");
      }
      else disp.setCursor(2, 1);
      disp.print(avgften, 1); // to 1 decimal place
      disp.print("Hz  ");
    }
  }
  else { // we don't have any averages
    calcfreqint = calcfreq64; // convert to 32-bit integer
    if (calcfreqint < 10000000) {
      disp.setCursor(2, 1); disp.print(" ");
    }
    else disp.setCursor(2, 1);
    disp.print(calcfreqint); // integer
    disp.print("Hz    ");
  }

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
  disp.print(F("m  "));
  disp.setCursor(9, 4);
  disp.print(F("Sats "));
  disp.print(GPSSats);
  if (GPSSats < 10) disp.print(F(" ")); // clear possible digit when sats >= 10
  // HDOP
  //disp.clearLine(5);
  disp.setCursor(0, 5);
  // choose HDOP or uptime
  //disp.print(F("HDOP "));
  //tempfloat = ((float) GPSHdop / 100);
  //disp.print(tempfloat);
  disp.print(F("Up "));  
  disp.print(updaysstr);
  disp.print(F(" "));
  disp.print(uptimestr);

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

  #ifdef GPSDO_BMP280_SPI
  // BMP280 temperature
  disp.setCursor(10, 6);
  disp.print(bmp280temp, 1);
  disp.print(F("C"));
  #endif // BMP280_SPI

  #ifdef GPSDO_VCC
  disp.setCursor(11, 2);
  // Vcc/2 is provided on pin PA0
  float Vcc = (float(avgVcc)/4096) * 3.3 * 2.0;
  disp.print(Vcc);
  disp.print(F("V"));
  #endif // VCC

  #ifdef GPSDO_VDD
  // internal sensor Vref
  disp.setCursor(11, 3);
  float Vdd = (1.21 * 4096) / float(avgVdd); // from STM32F411CEU6 datasheet                 
  disp.print(Vdd);                           // Vdd = Vref on Black Pill
  disp.print(F("V"));
  #endif // VDD

  disp.setCursor(11, 7); // display PWM/DAC value
  #ifdef GPSDO_PWM_DAC
  disp.print(adjusted_PWM_output);
  #else
  disp.print(adjusted_DAC_output);
  #endif // PWM_DAC
}
#endif // OLED

#ifdef GPSDO_LCD
// we use font1 8x6 pix and font2 16x12 pix
void displayscreen_LCD()
{
  //show GPSDO data on OLED display
  float tempfloat;

  // Latitude
  disp.setCursor(0, 40);
  disp.print(F("Lat: "));
  disp.print(GPSLat, 6);
  // Longitude
  disp.setCursor(0, 48);
  disp.print(F("Lon: "));
  disp.print(GPSLon, 6);
  // Altitude 
  disp.setCursor(0, 56);
  disp.print(F("Alt: "));
  disp.print(GPSAlt);
  disp.print(F("m  "));
  //Satellites
  disp.setCursor(90, 40);
  disp.print(F("Sats: "));
  disp.print(GPSSats);
  if (GPSSats < 10) disp.print(F(" ")); // clear possible digit when sats >= 10
  // HDOP
  disp.setCursor(0, 64);
  // choose HDOP or uptime
  //disp.print(F("HDOP "));
  //tempfloat = ((float) GPSHdop / 100);
  //disp.print(tempfloat);
  disp.print(F("UpT: "));  
  disp.print(updaysstr);
  //disp.print(F(" "));
  disp.setCursor(30, 72);
  disp.print(uptimestr);

  #ifdef GPSDO_BMP280_SPI
  // BMP280 temperature
  disp.setCursor(90, 64);
  disp.print(F(" "));
  disp.print((char)247);
  //disp.print((char)9);
  disp.print(F("C: "));
  disp.print(bmp280temp, 1);
  // BMP280 pressure
  disp.setCursor(90, 72);
  disp.print(F("hPa: "));
  disp.print(((bmp280pres+PressureOffset)/100), 1);
  #endif // BMP280_SPI

  #ifdef GPSDO_VCC
  disp.setCursor(90, 48);
  // Vcc/2 is provided on pin PA0
  float Vcc = (float(avgVcc)/4096) * 3.3 * 2.0;
  disp.print(F("5V0: "));
  disp.print(Vcc);
  disp.print(F("V"));
  #endif // VCC

  #ifdef GPSDO_VDD
  // internal sensor Vref
  disp.setCursor(90, 56);
  float Vdd = (1.21 * 4096) / float(avgVdd); // from STM32F411CEU6 datasheet                 
  disp.print(F("3V3: "));
  disp.print(Vdd);                           // Vdd = Vref on Black Pill
  disp.print(F("V"));
  #endif // VDD

  disp.setCursor(0, 80);  // display PWM/DAC value
  #ifdef GPSDO_PWM_DAC
    disp.print(F("PWM: "));
    disp.print(adjusted_PWM_output);
    disp.print(F(trendstr));
  #else
  disp.print(F("DAC: "));
    disp.print(adjusted_DAC_output);
    disp.print(F(trendstr));
  #endif // PWM_DAC
  
  // display vref value
  disp.setCursor(90, 80);  // display vref value
  float Vctlp = (float(avgpwmVctl)/4096) * 3.3;
  disp.print(F("Vctl: "));
  disp.print(Vctlp);
  disp.print(F("V"));
  
  // Display Headline and Version
  disp.setTextColor(ST7735_YELLOW, ST7735_BLACK);
  disp.setTextSize(2);
  disp.setCursor(0, 0);
  disp.print(F("   "));
  disp.print(F(Program_Name));
  //
  disp.setTextSize(1);
  disp.setCursor(115, 5);
  disp.setTextColor(ST7735_BLUE, ST7735_BLACK);
  disp.print(F(Program_Version));
  disp.setTextSize(2);
  
  // OCXO frequency
  disp.setCursor(0, 19);
  disp.setTextColor(ST7735_YELLOW, ST7735_BLACK);

  // display 1s, 10s or 100s value depending on whether data is available
  if (cbTen_full) {
    if (cbTho_full) { // if we have data over 1000 seconds
      if (avgftho < 10000000) {
		disp.print(" ");
        }
        disp.print(avgftho, 3); // to 3 decimal places
      }
      else if (cbHun_full) {
        if (avgfhun < 10000000) {
		  disp.print(" ");
          }
          disp.print(avgfhun, 2); // to 2 decimal places
	      disp.print(" ");
    }

	else { // nope, only 10 seconds
      if (avgften < 10000000) {
		disp.print(" ");
        }
       disp.print(avgften, 1); // to 1 decimal place
	   disp.print("  ");
    }
  }
  else { // we don't have any averages and print integer value
    calcfreqint = calcfreq64; // convert to 32-bit integer
    if (calcfreqint < 10000000) {
	    disp.print(" ");
        }
    disp.print(calcfreqint); // integer
	disp.print("    ");       // these are used for more exact dispaly
    }
  // due to limited space small character unit
  disp.setTextSize(1);
  disp.setCursor(144, 19);
  // due to some unknown issue in printing the freq digits/unit we clear the section of the display first
  disp.fillRect(146, 19, 13, 17, ST7735_BLACK);
  disp.print("Hz");

  // clock and date
  disp.setTextSize(2);
  disp.setTextColor(ST7735_RED, ST7735_BLACK);
  // Date
  disp.setCursor(2, 94);
  //disp.print(F("Date: "));  
  disp.print(day);
  disp.print(F("."));
  disp.print(month);
  disp.print(F("."));
  disp.print(year);
  
  // Time
  disp.setTextColor(ST7735_GREEN, ST7735_BLACK);
  disp.setCursor(2, 113);

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
  disp.setTextColor(ST7735_WHITE, ST7735_BLACK);
  disp.print(F(" UTC"));
  
  // reset all font stuff for normal display
  disp.setTextSize(1);
  disp.setTextColor(ST7735_WHITE, ST7735_BLACK);

}
#endif // LCD
