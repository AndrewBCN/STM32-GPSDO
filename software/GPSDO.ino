/**********************************************************************************************************
  STM32 GPSDO v0.05j by André Balsa, March 2022
  GPLV3 license
  GitHub collaborators: iannezsp (Angelo Iannello)
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
  - Initial ST7735 SPI LCD display support code contributed by Badwater-Frank.

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
**********************************************************************************************************/

// GPSDO with STM32 MCU, optional OLED/LCD display, various sensors, DFLL in software, optional Bluetooth

/**********************************************************************************************************
  This Arduino with STM32 Core package sketch implements a GPSDO with display options. It uses an SSD1306 
  128x64 I2C OLED display or SPI LCD. It reads the GPS for 1 or 5 seconds and copies the half-dozen or so
  default NMEA sentences from the GPS to either the USB serial or Bluetooth serial ports (but not both) 
  if verbose mode is enabled. That is followed by various sensors data and the FLL and OCXO data.
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
**********************************************************************************************************/
/* Libraries required to compile, depending on configured options:
    - TinyGPS++
    - U8g2/u8x8 graphics library, see https://github.com/olikraus/u8g2
    - Adafruit AHTX0
    - Adafruit BMP280
    - movingAvg library, on STM32 architecture needs a simple patch to avoid warning during compilation
    - Color LCD support requires the installation of the Adafruit ST7735 and ST7789 LCD library
      and the Adafruit GFX library.
    - TM1637 LED clock module requires the TM1637 library, see https://github.com/avishorp/TM1637

   For commands parsing, uses SerialCommands library found here:
   https://github.com/ppedro74/Arduino-SerialCommands

   And also requires the installation of support for the STM32 MCUs by installing the STM32duino
   package (STM32 Core version 2.2.0 or later).
**********************************************************************************************************/
/* Commands implemented:
    - V : returns program name, version and author
    - F : flush ring buffers
    - C : calibrate OCXO
    - dp/up 1/10 : adjust Vctl down/up PWM fine/coarse, example dp1 means decrease PWM by 1.
    - SP <number> : set PWM to value between 1 and 65535
    - RD/RH : toggle between tab Delimited and Human readable GPSDO status reporting
    
/* Commands to be implemented:
    - L0 to L9 : select log levels
    - L0 : silence mode
    - L1 : fix only mode
    - L7 : fix and full status mode, no NMEA (default)
    - L8 : NMEA stream from GPS module only mode
    - L9 : NMEA + full status

/**********************************************************************************************************
  Program Operation -  This program is a GPSDO with optional OLED display. It uses a small SSD1306
  128x64 I2C OLED display. At startup the program starts checking the data coming from the GPS for a
  valid fix. It reads the GPS NMEA stream for 1/5 seconds and if there is no fix, prints a message on the
  Arduino IDE serial monitor and updates the seconds without a fix on the display. During this time the
  NMEA stream coming from the GPS is copied to the serial monitor also. The DFLL is active as soon as
  the GPS starts providing a 1PPS pulse. The 10MHz OCXO is controlled by a voltage generated by either
  the 16-bit PWM ; this voltage (Vctl) is adjusted once every 429 seconds.
**********************************************************************************************************/

// Version v0.04i and later: Erik Kaashoek has suggested a 10s sampling rate for the 64-bit counter, to save RAM.
// This is work in progress, see the changes in Timer2_Capture_ISR.

// Version v0.05j and later: reporting on USB serial / Bluetooth serial can be toggled between human readable and tabulated data.

// 1. picDIV synchronization control.
// 2. Refactor the entire program to make it easier to understand and maintain.
// 3. Implement support for the STM32F401CCU6 Black Pill.
// 4. Frequency / period meter implementation.
// 5. Improve layout of ST7789 display.

#define Program_Name "GPSDO"
#define Program_Version "v0.05j"
#define Author_Name "André Balsa"

// Debug options
// -------------
#define FastBootMode          // reduce various delays during boot
#define TunnelModeTesting     // reduce tunnel mode timeout

// Hardware options
// ----------------
// #define GPSDO_STM32F401       // use an STM32F401 Black Pill instead of STM32F411 (reduced RAM)
                                 // IMPORTANT! Don't forget to select the correct board in the Tools->Board menu in the arduino IDE
#define GPSDO_OLED            // SSD1306 128x64 I2C OLED display
// #define GPSDO_LCD_ST7735      // ST7735 160x128 SPI LCD display
#define GPSDO_LCD_ST7789      // ST7789 240x240 SPI LCD display (testing)
#define GPSDO_PWM_DAC         // STM32 16-bit PWM DAC, requires two rc filters (2xr=20k, 2xc=10uF) - note this will become the default
#define GPSDO_AHT10           // AHT10 or AHT20 (recommended) I2C temperature and humidity sensor
#define GPSDO_GEN_2kHz_PB5    // generate 2kHz square wave test signal on pin PB5 using Timer 3
// #define GPSDO_BMP280_SPI      // SPI atmospheric pressure, temperature and altitude sensor
#define GPSDO_BMP280_I2C      // I2C atmospheric pressure, temperature and altitude sensor
#define GPSDO_INA219          // INA219 I2C current and voltage sensor
// #define GPSDO_BLUETOOTH       // Bluetooth serial (HC-06 module)
#define GPSDO_VCC             // Vcc (nominal 5V) ; reading Vcc requires 1:2 voltage divider to PA0
#define GPSDO_VDD             // Vdd (nominal 3.3V) reads VREF internal ADC channel
#define GPSDO_CALIBRATION     // auto-calibration is enabled
#define GPSDO_UBX_CONFIG      // optimize u-blox GPS receiver configuration
#define GPSDO_VERBOSE_NMEA    // GPS module NMEA stream echoed to USB serial xor Bluetooth serial
// #define GPSDO_PICDIV          // generate a 1.2s synchronization pulse for the picDIV
#define GPSDO_TM1637          // TM1637 4-digit LED module
// #define GPSDO_TIC            // read TIC 12-bit value on PA1 (ADC channel 1), then discharge capacitor using PB2
#define GPSDO_EEPROM            // enable STM32 buffered EEPROM emulation library

// Includes
// --------
#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x02020000)
  #error "Due to API changes, this sketch is compatible with STM32_CORE_VERSION >= 0x02020000 (2.2.0 or later)"
#endif

// Increase HardwareSerial (UART) TX and RX buffer sizes from default 64 characters to 256.
// The main worry here is that we could miss some characters from the u-blox GPS module if
// the processor is busy doing something else (e.g. updating the display, reading a sensor, etc)
// specially since we increase the GPS baud rate from 9600 to 38400.

#define SERIAL_TX_BUFFER_SIZE 256       // Warning: > 256 could cause problems, see comments in STM32 HardwareSerial library
#define SERIAL_RX_BUFFER_SIZE 256

bool report_tab_delimited = false;      // true for tab delimited reporting, false for human-readable reporting
uint64_t report_line_no = 0;            // line number for tab delimited reporting, 0 if no GPS fix

const uint16_t waitFixTime = 1;         // Maximum time in seconds waiting for a fix before reporting no fix / yes fix
                                        // Tested values 1 second and 5 seconds, 1s recommended

#include <movingAvg.h>                  // https://github.com/JChristensen/movingAvg , needs simple patch
                                        // to avoid warning message during compilation

#ifdef GPSDO_PICDIV
  #define picDIVsyncPin PB3               // digital output pin used to generate a 1.2s synchronization pulse for the picDIV
#endif // PICDIV

#ifdef GPSDO_GEN_2kHz_PB5
  #define Test2kHzOutputPin PB5           // digital output pin used to output a test 2kHz square wave
#endif // GEN_2kHz_PB5

// HC-06 Bluetooth module
#ifdef GPSDO_BLUETOOTH
  //              UART    RX   TX
  HardwareSerial Serial2(PA3, PA2);                // Serial connection to HC-06 Bluetooth module
  #define BT_BAUD 57600                            // Bluetooth baud rate
#endif // BLUETOOTH

// EEPROM emulation in flash
#ifdef GPSDO_EEPROM
  #include <EEPROM.h>                              // Buffered EEPROM emulation library
#endif // EEPROM

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

// AHT10 / AHT20 temperature humidity sensor       // Uses PB6 (SCL1) and PB7 (SDA1) on Black Pill for I2C1
#ifdef GPSDO_AHT10
  #include <Adafruit_AHTX0.h>                      // Adafruit AHTX0 library
  Adafruit_AHTX0 aht;                              // create object aht
#endif // AHT10

// INA219 current voltage sensor
#ifdef GPSDO_INA219
  #include <Adafruit_INA219.h>
  Adafruit_INA219 ina219;
  float ina219volt=0.0, ina219curr=0.0;
#endif // INA219

// TM1637 4-digit LED module
#ifdef GPSDO_TM1637
  #include <TM1637Display.h>                      // get library here > https://github.com/avishorp/TM1637
  // Module connection pins (Digital Pins)
  #define CLK PA8                                 // interface to TM1637 requires two GPIO pins
  #define DIO PB4
  TM1637Display tm1637(CLK, DIO);                 // create tm1637 object
  const uint8_t mid_dashes[] = {
    SEG_G,   // -
    SEG_G,   // -
    SEG_G,   // -
    SEG_G    // -
  };
  const uint8_t low_oooo_s[] = {
    SEG_C | SEG_D | SEG_E | SEG_G,   // o
    SEG_C | SEG_D | SEG_E | SEG_G,   // o
    SEG_C | SEG_D | SEG_E | SEG_G,   // o
    SEG_C | SEG_D | SEG_E | SEG_G    // o
  };
#endif // TM1637

// OLED 0.96 SSD1306 128x64
#ifdef GPSDO_OLED
  #include <U8x8lib.h>                                      // get library here >  https://github.com/olikraus/u8g2 
  U8X8_SSD1306_128X64_NONAME_HW_I2C disp(U8X8_PIN_NONE);    // use this line for standard 0.96" SSD1306
#endif // OLED

// LCD 1.8" ST7735 160x128 (tested by Badwater-Frank)
#ifdef GPSDO_LCD_ST7735
  #include <Adafruit_GFX.h>       // need this adapted for STM32F4xx/F411C: https://github.com/fpistm/Adafruit-GFX-Library/tree/Fix_pin_type
  #include <Adafruit_ST7735.h>
  //#include <Fonts/FreeSansBold18pt7b.h>
  #include <SPI.h>
  #define TFT_DC  PA1             // note this pin assigment conflicts with the original schematic
  #define TFT_CS  PA2
  #define TFT_RST PA3
  // For 1.44" and 1.8" TFT with ST7735 use:
  Adafruit_ST7735 disp = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
#endif // LCD_ST7735

// LCD 1.3" ST7789 240x240 (Testing)
#ifdef GPSDO_LCD_ST7789
  #include <Adafruit_GFX.h>       // need this adapted for STM32F4xx/F411C: https://github.com/fpistm/Adafruit-GFX-Library/tree/Fix_pin_type
  #include <Adafruit_ST7789.h>
  #include <SPI.h>
  #define TFT_DC  PB12            // note pin assigment that does not conflict with other interfaces
  #define TFT_CS  PB13            // in reality, CS not connected, CS not used on 1.3" TFT ST7789 display
  #define TFT_RST PB15            // also uses pins PA5, PA6, PA7 for MOSI MISO and SCLK
  // For 1.3" LCD with ST7789
  Adafruit_ST7789 disp_st7789 = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
  bool must_clear_disp_st7789 = false;  // flag is set when display has to be cleared 
  
  // include the following GFX library fonts
  #include <Fonts/FreeMono9pt7b.h>      // tiny, but readable in white
  #include <Fonts/FreeMonoBold12pt7b.h> // medium size, readable
#endif // LCD_ST7789

// PWM 16-bit DAC
const uint16_t default_PWM_output = 35585; // "ideal" 16-bit PWM value, varies with OCXO, RC network, and time and temperature
                                           // 35585 for a second NDK ENE3311B
uint16_t adjusted_PWM_output;              // we adjust this value to "close the loop" of the DFLL when using the PWM
volatile bool must_adjust_DAC = false;     // true when there is enough data to adjust Vctl
char trendstr[5] = " ___";                 // PWM trend string, set in the adjustVctlPWM() function

#define VctlPWMOutputPin PB9          // digital output pin used to output a PWM value, TIM4 ch4
                                      // Two cascaded RC filters transform the PWM into an analog DC value
#define VctlPWMInputPin PB1           // ADC pin to read Vctl from filtered PWM
volatile int pwmVctl = 0;             // variable used to store PWM Vctl read by ADC pin PB1

// VCC - 5V
#ifdef GPSDO_VCC
  #define VccDiv2InputPin PA0           // Vcc/2 using resistor divider connects to PA0
  int adcVcc = 0;
#endif // VCC

// VDD - 3.3V
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

movingAvg avg_pwmVctl(10);
int16_t avgpwmVctl = 0;

// BMP280 atmospheric pressure and temperature sensor
#ifdef GPSDO_BMP280_SPI
  // BMP280 - SPI
  #include <SPI.h>
  #include <Adafruit_BMP280.h>
  #define BMP280_CS   (PA4)              // SPI1 uses PA4, PA5, PA6, PA7
  Adafruit_BMP280 bmp(BMP280_CS);        // hardware SPI, use PA4 as Chip Select
#endif // BMP280_SPI

#ifdef GPSDO_BMP280_I2C
  // BMP280 - I2C
  #include <Adafruit_BMP280.h>
  Adafruit_BMP280 bmp;                  // hardware I2C
#endif // BMP280_I2C

#if (defined (GPSDO_BMP280_SPI) || defined (GPSDO_BMP280_I2C))
  const uint16_t PressureOffset = 1860;  // that offset must be calculated for your sensor and location
  float bmp280temp=0.0, bmp280pres=0.0, bmp280alti=0.0; // read sensor, save here
#endif // BMP280

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

// OCXO frequency measurement

// special 10s sampling rate data structures (work in progress)
volatile uint16_t esamplingfactor = 10; // sample 64-bit counter every 10 seconds
volatile uint16_t esamplingcounter = 0; // counter from 0 to esamplingfactor
volatile bool esamplingflag = false;

volatile uint64_t circbuf_esten64[11];    // 10+1 x10 seconds circular buffer, so 100 seconds
volatile uint32_t cbihes_newest = 0;      // newest/oldest index
volatile bool cbHes_full = false;         // flag set when buffer has filled up
volatile double avgesample = 0;           // 100 seconds average with 10s sampling rate

// other OCXO frequency measurement data structures
const uint32_t basefreq=10000000;        // OCXO nominal frequency in Hz
volatile uint32_t lsfcount=0, previousfcount=0, calcfreqint=basefreq;

// Frequency check boundaries 
const uint32_t lowerfcount = 9999500;
const uint32_t upperfcount = 10000500;

/* Moving average frequency variables
   Basically we store the counter captures for 10 and 100 seconds.
   When the buffers are full, the average frequency is quite simply
   the difference between the oldest and newest data divided by the size
   of the buffer.
   Each second, when the buffers are full, we overwrite the oldest data
   with the newest data and calculate each average frequency.
 */
volatile uint64_t fcount64=0, prevfcount64=0, calcfreq64=basefreq; 
// ATTENTION! must declare 64-bit, not 32-bit variable, because of shift
volatile uint64_t tim2overflowcounter = 0;  // testing, counts the number of times TIM2 overflows
volatile bool overflowflag = false;         // flag set by the overflow ISR, reset by the 2Hz ISR
volatile bool captureflag = false;          // flag set by the capture ISR, reset by the 2Hz ISR
volatile bool overflowErrorFlag = false;    // flag set if there was an overflow processing error

volatile uint64_t circbuf_ten64[11]; // 10+1 seconds circular buffer
volatile uint64_t circbuf_hun64[101]; // 100+1 seconds circular buffer
volatile uint64_t circbuf_tho64[1001]; // 1,000+1 seconds circular buffer
#ifndef GPSDO_STM32F401
volatile uint64_t circbuf_tth64[10001]; // 10,000 + 1 seconds circular buffer
#else // STM32F401 has less RAM
volatile uint64_t circbuf_fth64[5001];  // 5,000 + 1 seconds circular buffer
#endif // GPSDO_STM32F401

volatile uint32_t cbiten_newest=0; // index to oldest, newest data
volatile uint32_t cbihun_newest=0;
volatile uint32_t cbitho_newest=0;
volatile uint32_t cbitth_newest=0;

volatile bool cbTen_full=false, cbHun_full=false, cbTho_full=false, cbTth_full=false;  // flag when buffer full
volatile double avgften=0, avgfhun=0, avgftho=0, avgftth=0; // average frequency calculated once the buffer is full
volatile bool flush_ring_buffers_flag = true;  // indicates ring buffers should be flushed

// Miscellaneous data structures

// picDIV support
#ifdef GPSDO_PICDIV
  #define VphaseInputPin PB1                    // ADC pin to read Vphase from 1ns-resolution TIC
  volatile bool force_armpicDIV_flag = true;    // indicates picDIV must be armed waiting to sync on next PPS from GPS module
#endif // PICDIV

volatile bool force_calibration_flag = true;   // indicates GPSDO should start calibration sequence

volatile bool ocxo_needs_warming = true;       // indicates OCXO needs to warm up a few minutes after power on

#ifdef FastBootMode
  const uint16_t ocxo_warmup_time = 15;        // ocxo warmup time in seconds; 15s for testing
  const uint16_t ocxo_calib_time = 15;         // 15s fast calibration countdown time (for each calibration step)
#else             
  const uint16_t ocxo_warmup_time = 300;       // ocxo warmup time in seconds;  300s or 600s normal use
  const uint16_t ocxo_calib_time = 60;         // 60s normal calibration countdown time (for each calibration step)
#endif  // FastBootMode

volatile bool tunnel_mode_flag = false;        // the GPSDO relays the information directly to and from the GPS module to the USB serial
#ifdef TunnelModeTesting
  const uint16_t tunnelSecs = 15;                // tunnel mode timeout in seconds; 15s for testing, 300s or 600s normal use
#else
  const uint16_t tunnelSecs = 300;               // tunnel mode timeout in seconds; 15s for testing, 300s or 600s normal use
#endif  // TunnelModeTesting

// Miscellaneous functions

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

// called for RD (Report tab Delimited format) command
void cmd_repdel(SerialCommands* sender)
{
  report_tab_delimited = true;  // set switch
  sender->GetSerial()->println("Switching to reporting in Tab Delimited Format");
}

// called for RH (Report Human readable format) command
void cmd_rephum(SerialCommands* sender)
{
  report_tab_delimited = false;  // reset switch
  report_line_no = 0;            // reset line counter
  sender->GetSerial()->println("Switching to reporting in Human Readable Format");
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

// called for SP (set PWM) command
void cmd_setPWM(SerialCommands* sender)
{
  int32_t pwm;
  char* pwm_str = sender->Next();
  if (pwm_str == NULL) // check if a value was specified
  {
    sender->GetSerial()->println("No PWM value specified, using default");
    pwm = default_PWM_output;
    adjusted_PWM_output = pwm;
    analogWrite(VctlPWMOutputPin, adjusted_PWM_output);
  }
  else // check the value that was specified
  {
    pwm = atoi(pwm_str); // note atoi() returns zero if it cannot convert the string to a valid integer
    if ((pwm >= 1) && (pwm <= 65535)) // check if the value specified is positive 16-bit integer
    {
      sender->GetSerial()->print("Setting PWM value "); // if yes, set the value
      sender->GetSerial()->println(pwm);
      adjusted_PWM_output = pwm;
      analogWrite(VctlPWMOutputPin, adjusted_PWM_output);
    }
    else // incorrect value specified, print error message
    {
      sender->GetSerial()->println("PWM value must be positive integer between 1 and 65535, leaving unchanged");  
    }  
  }
}

// PWM direct control commands (up/down)
// -------------------------------------
// called for up1 (increase PWM 1 bit) command
void cmd_up1(SerialCommands* sender)
{
  adjusted_PWM_output = adjusted_PWM_output + 1;
  analogWrite(VctlPWMOutputPin, adjusted_PWM_output);
  sender->GetSerial()->println("increased PWM 1 bit");
}
// called for up10 (increase PWM 10 bits) command
void cmd_up10(SerialCommands* sender)
{
  adjusted_PWM_output = adjusted_PWM_output + 10;
  analogWrite(VctlPWMOutputPin, adjusted_PWM_output);
  sender->GetSerial()->println("increased PWM 10 bits");
}
// called for dp1 (decrease PWM 1 bit) command
void cmd_dp1(SerialCommands* sender)
{
  adjusted_PWM_output = adjusted_PWM_output - 1;
  analogWrite(VctlPWMOutputPin, adjusted_PWM_output);
  sender->GetSerial()->println("decreased PWM 1 bit");
}
// called for dp10 (decrease PWM 10 bits) command
void cmd_dp10(SerialCommands* sender)
{
  adjusted_PWM_output = adjusted_PWM_output - 10;
  analogWrite(VctlPWMOutputPin, adjusted_PWM_output);
  sender->GetSerial()->println("decreased PWM 10 bits");
}

#ifdef GPSDO_EEPROM
  // add callback functions for PS and PR commands to Store PWM and Recall PWM value in Flash (emulating EEPROM)
  // warning! limited number of writes allowed
#endif // EEPROM

// SerialCommand commands
// Note: Commands are case sensitive
SerialCommand cmd_version_("V", cmd_version);     // print program name and version
SerialCommand cmd_flush_("F", cmd_flush);         // flush ring buffers
SerialCommand cmd_calibrate_("C", cmd_calibrate); // force calibration
SerialCommand cmd_tunnel_("T", cmd_tunnel);       // activate tunnel mode
SerialCommand cmd_rephum_("RH", cmd_rephum);      // activate humand readable reporting
SerialCommand cmd_repdel_("RD", cmd_repdel);      // activate tab delimited reporting
SerialCommand cmd_setPWM_("SP", cmd_setPWM);      // note this command takes a 16-bit PWM value (1 to 65535) as an argument
// 16-bit PWM commands
SerialCommand cmd_up1_("up1", cmd_up1);
SerialCommand cmd_up10_("up10", cmd_up10);
SerialCommand cmd_dp1_("dp1", cmd_dp1);
SerialCommand cmd_dp10_("dp10", cmd_dp10);

#ifdef GPSDO_EEPROM
  // add PS and PR commands to Store PWM and Recall PWM value in Flash (emulating EEPROM)
  // warning! limited number of writes allowed
#endif // EEPROM

// loglevel
uint8_t loglevel = 7;   // see commands comments for log level definitions, default is 7
                        // note log levels are not implemented yet

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
         if (((fcount64 - prevfcount64) > lowerfcount) && ((fcount64 - prevfcount64) < upperfcount)) { // if we have a valid fcount, otherwise it's discarded
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

// ---------------------------------------------------------------------------------------------
//    GPSDO tunnel mode (GPS serial is relayed to Bluetooth serial or USB serial)
// --------------------------------------------------------------------------------------------- 
void tunnelgps()
// GPSDO tunnel mode operation
{
  #ifdef GPSDO_BLUETOOTH      // print entering tunnel mode message to either
  Serial2.println();          // Bluetooth serial xor USB serial
  Serial2.print(F("Entering tunnel mode..."));
  Serial2.println();
  #else
  Serial.println();
  Serial.print(F("Entering tunnel mode..."));
  Serial.println();
  #endif // BLUETOOTH

  // tunnel mode operation starts here
  uint32_t endtunnelmS = millis() + (tunnelSecs * 1000);
  uint8_t GPSchar;
  uint8_t PCchar;
  while (millis() < endtunnelmS)
  {
    if (Serial1.available() > 0)
    {
      GPSchar = Serial1.read();
      #ifdef GPSDO_BLUETOOTH
      Serial2.write(GPSchar); // echo GPS NMEA serial stream to Bluetooth serial
      #else      
      Serial.write(GPSchar);  // echo GPS NMEA serial stream to USB serial
      #endif // BLUETOOTH
    }
    #ifdef GPSDO_BLUETOOTH
    if (Serial2.available() > 0)
    #else
    if (Serial.available() > 0)
    #endif // BLUETOOTH
    {
      #ifdef GPSDO_BLUETOOTH
      PCchar = Serial2.read();
      #else
      PCchar = Serial.read();
      #endif // BLUETOOTH
      Serial1.write(PCchar);  // echo USB serial stream to GPS serial
    }
  }
  // tunnel mode operation ends here
  
  #ifdef GPSDO_BLUETOOTH      // print exiting tunnel mode message to either
  Serial2.println();          // Bluetooth serial xor USB serial
  Serial2.print(F("Tunnel mode exited."));
  Serial2.println();
  #else
  Serial.println();
  Serial.print(F("Tunnel mode exited."));
  Serial.println();
  #endif // BLUETOOTH
  
  tunnel_mode_flag = false; // reset flag, exit tunnel mode
} // end of tunnel mode routine

// ---------------------------------------------------------------------------------------------
//    OCXO warmup delay routine (only needed during a GPSDO "cold start")
// ---------------------------------------------------------------------------------------------
void doocxowarmup()
{
  // Spend a few seconds/minutes here just waiting for the OCXO to warmup
  // show countdown timer on OLED or LCD display
  // and report on either USB serial or Bluetooth serial
  // Note: during OCXO warmup the GPSDO does not accept any commands
  uint16_t countdown = ocxo_warmup_time;
  while (countdown) {
              
          #ifdef GPSDO_OLED
          disp.clear();                // display warmup message on OLED
          disp.setCursor(0, 0);
          disp.print(F(Program_Name));
          disp.print(F(" - "));
          disp.print(F(Program_Version));
          disp.setCursor(0, 2);
          disp.print(F("OCXO warming up"));
          disp.setCursor(0, 3);
          disp.print(F("Please wait"));
          disp.setCursor(5, 4);
          disp.print(countdown);
          disp.print(F("s"));
          #endif // OLED
          
          #ifdef GPSDO_LCD_ST7735
          disp.fillScreen(ST7735_BLACK);  // display warmup message on LCD ST7735
          disp.setCursor(0, 0);
          disp.print(F(Program_Name));
          disp.print(F(" - "));
          disp.print(F(Program_Version));
          disp.setCursor(0, 16);
          disp.print(F("OCXO warming up"));
          disp.setCursor(0, 24);
          disp.print(F("Please wait"));
          disp.setCursor(0, 32);
          disp.print(countdown);
          disp.print(F("s"));
          #endif // LCD_ST7735
          
          #ifdef GPSDO_LCD_ST7789
            // display OCXO warmup message on ST7789 LCD
            disp_st7789.fillScreen(ST77XX_BLACK); // clear display
            // Display program name and version
            disp_st7789.setTextSize(1);
            disp_st7789.setFont(&FreeMonoBold12pt7b);
            disp_st7789.setTextColor(ST77XX_YELLOW);
            disp_st7789.setCursor(0, 16);
            disp_st7789.print(F("STM32 "));
            disp_st7789.print(F(Program_Name));
            disp_st7789.setTextSize(1);
            disp_st7789.setFont(&FreeMono9pt7b);
            disp_st7789.setTextColor(ST77XX_CYAN);
            disp_st7789.setCursor(168, 11);
            disp_st7789.print(F(Program_Version));
            // display OCXO warming up and countdown
            disp_st7789.setCursor(0, 36);
            disp_st7789.setTextColor(ST77XX_WHITE);
            disp_st7789.print(F("OCXO warming up"));
            disp_st7789.setCursor(0, 50);
            disp_st7789.print(F("Please wait"));
            disp_st7789.setCursor(0, 64);
            disp_st7789.print(countdown);
            disp_st7789.print(F("s"));
            
            must_clear_disp_st7789 = true; 

          #endif // LCD_ST7789
          
          #ifdef GPSDO_BLUETOOTH      // print warmup countdown message to either
            Serial2.println();          // Bluetooth serial xor USB serial
            Serial2.print(F("OCXO Warming up, "));
            Serial2.print(countdown);
            Serial2.println(F("s remaining"));
          #else
            Serial.println();
            Serial.print(F("OCXO Warming up, "));
            Serial.print(countdown);
            Serial.println(F("s remaining"));
          #endif // BLUETOOTH

          // do nothing for 1s
          delay(1000);
          countdown--;
  }
  ocxo_needs_warming = false; // reset flag, next "hot" calibration skips ocxo warmup   
} // end of OCXO warmup routine

// ---------------------------------------------------------------------------------------------
//    GPSDO calibration routine
// --------------------------------------------------------------------------------------------- 
void docalibration()
// OCXO Vctl calibration: find an approximate value for Vctl
{
  yellow_led_state = 2;        // blink yellow LED (handled by 2Hz ISR)
  
  if (ocxo_needs_warming) doocxowarmup();
  
  // Note: during calibration the GPSDO does not accept any commands
  #ifdef GPSDO_BLUETOOTH      // print calibration started message to either
  Serial2.println();          // Bluetooth serial xor USB serial
  Serial2.print(F("Calibrating..."));
  Serial2.println();
  #else
  Serial.println();
  Serial.print(F("Calibrating..."));
  Serial.println();
  #endif // BLUETOOTH

  #ifdef GPSDO_OLED
  disp.clear();                // display calibrating message on OLED
  disp.setCursor(0, 0);
  disp.print(F(Program_Name));
  disp.print(F(" - "));
  disp.print(F(Program_Version));
  disp.setCursor(0, 2);
  disp.print(F("Calibrating..."));
  disp.setCursor(0, 3);
  disp.print(F("Please wait"));
  #endif // OLED

  #ifdef GPSDO_LCD_ST7735
  disp.fillScreen(ST7735_BLACK);  // display calibrating message on LCD ST7735
  disp.setCursor(0, 0);
  disp.print(F(Program_Name));
  disp.print(F(" - "));
  disp.print(F(Program_Version));
  disp.setCursor(0, 16);
  disp.print(F("Calibrating..."));
  disp.setCursor(0, 24);
  disp.print(F("Please wait"));
  #endif // LCD_ST7735

  #ifdef GPSDO_LCD_ST7789
    // display calibrating message on LCD ST7789
    
    disp_st7789.fillScreen(ST77XX_BLACK); // clear display
    
    // display program name and version
    disp_st7789.setTextSize(1);
    disp_st7789.setFont(&FreeMonoBold12pt7b);
    disp_st7789.setTextColor(ST77XX_YELLOW);
    disp_st7789.setCursor(0, 16);
    disp_st7789.print(F("STM32 "));
    disp_st7789.print(F(Program_Name));
    disp_st7789.setTextSize(1);
    disp_st7789.setFont(&FreeMono9pt7b);
    disp_st7789.setTextColor(ST77XX_CYAN);
    disp_st7789.setCursor(168, 11);
    disp_st7789.print(F(Program_Version));
    
    // display calibrating message
    disp_st7789.setCursor(0, 36);
    disp_st7789.setTextColor(ST77XX_WHITE);
    disp_st7789.print(F("Calibrating..."));
    disp_st7789.setCursor(0, 50);
    disp_st7789.print(F("Please wait"));
     
    must_clear_disp_st7789 = true;
  
  #endif // LCD_ST7789

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
  while (!cbTen_full) delay(1000); // note there is a small chance that we lose PPS during calibration
                                   // resulting in completely wrong calibration value
  
  // measure frequency for Vctl=1.5V
  Serial.println(F("Measure frequency for Vctl=1.5V"));
  Serial.print(F("Set PWM Vctl to 1.5V, wait ")); Serial.print(ocxo_calib_time); Serial.println(F("s"));
  analogWrite(VctlPWMOutputPin, 30720);
  
  uint16_t calib_countdown = ocxo_calib_time; // note there are two possible values depending on FastBootMode setting
  while (calib_countdown > 0)
    {
      calib_countdown--; 
      Serial.print(calib_countdown); Serial.print(F("s "));
      delay(1000);    
    }  
  
  Serial.println();
  Serial.print(F("f1 (average frequency for Vctl=1.5V): "));
  f1 = avgften;
  Serial.print(f1,1);
  Serial.println(F(" Hz"));
  Serial.println();
  
  // make sure we have a fix and data again
  while (!cbTen_full) delay(1000);
  
  // measure frequency for Vctl=2.5V
  Serial.println(F("Measure frequency for Vctl=2.5V"));
  Serial.println(F("Set PWM Vctl to 2.5V, wait ")); Serial.print(ocxo_calib_time); Serial.println(F("s"));
  analogWrite(VctlPWMOutputPin, 51200);
  
  calib_countdown = ocxo_calib_time;  // no need to declare variable again
  while (calib_countdown > 0)
    {
      calib_countdown--; 
      Serial.print(calib_countdown); Serial.print(F("s "));
      delay(1000);    
    }  
  
  Serial.println();
  Serial.print(F("f2 (average frequency for 2.5V Vctl): "));
  f2 = avgften;
  Serial.print(f2,1);
  Serial.println(F(" Hz"));
  Serial.println();
  
  // slope s is (f2-f1) / (51200-30720) for PWM
  // So F=10MHz +/- 0.1Hz for PWM = 30720 - (e1 / s)
  // set Vctl
  // adjusted_PWM_output = formula
  adjusted_PWM_output = 30720 - ((f1 - 10000000.0) / ((f2 - f1) / 20480));
  Serial.print(F("Calculated PWM: "));
  Serial.println(adjusted_PWM_output);
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

  #ifdef GPSDO_OLED
    disp.clear();             // clear display and show program name and version again
    disp.setCursor(0, 0);
    disp.print(F(Program_Name));
    disp.print(F(" - "));
    disp.print(F(Program_Version)); 
  #endif // OLED
   
  #ifdef GPSDO_LCD_ST7735
    disp.fillScreen(ST7735_BLACK);  
  #endif // LCD_ST7735 
  
  #ifdef GPSDO_LCD_ST7789
    disp_st7789.fillScreen(ST77XX_BLACK);  
  #endif // LCD_ST7789 
  
  yellow_led_state = 0;           // turn off yellow LED (handled by 2Hz ISR)
  force_calibration_flag = false; // reset flag, calibration done
} // end of GPSDO calibration routine

// ---------------------------------------------------------------------------------------------
//    Adjust Vctl PWM routine
// ---------------------------------------------------------------------------------------------
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

  if (!report_tab_delimited) {
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
  }

  endwaitmS = millis() + (waitSecs * 1000);

  while (millis() < endwaitmS)
  {
    if (Serial1.available() > 0)
    {
      GPSchar = Serial1.read();
      gps.encode(GPSchar);
      if (!report_tab_delimited) {
        #ifdef GPSDO_VERBOSE_NMEA
          #ifdef GPSDO_BLUETOOTH
            Serial2.write(GPSchar); // echo NMEA stream to Bluetooth serial
          #else
            Serial.write(GPSchar);  // echo NMEA stream to USB serial
          #endif // Bluetooth
        #endif // VERBOSE_NMEA
      }
    }

    if (gps.location.isUpdated() && gps.altitude.isUpdated() && gps.date.isUpdated())
    {
      endFixmS = millis();                                //record the time when we got a GPS fix
      return true;
    }
  }
  return false;
}

void printGPSDOtab(Stream &Serialx) {       // tab delimited fields suitable for spreadsheet import
  
/*  
 *   STM32 GPSDO reporting tab delimited fields
 *   if any field is not available (e.g. sensor not configured), print "0"

 *   Line no. (0 if no position fix, increments by one each second if position fix)
 *   timestamp (UTC)
 *   uptime (days hours mins secs)
 *   64-bit counter 
 *   frequency (Hz)
 *   10s freq. avg. (one decimal) (Hz)
 *   100s freq. avg. (two decimals) (Hz)
 *   1,000s freq. avg. (three decimals) (Hz)
 *   10,000s freq. avg. (four decimals) (Hz)
 *   no. of sats
 *   HDOP (meters)
 *   PWM (16-bit, 1-65535)
 *   PWM adc mov. avg. (V)
 *   Vcc adc mov. avg. (5.0V nominal)
 *   Vdd adc mov. avg. (3.3V nominal)
 *   BMP280 Temp. (C)
 *   BMP280 Atm. Pressure (hPa)
 *   AHT20 Temp. (C)
 *   AHT20 Humidity (%)
 *   INA219 OCXO Voltage (5.05V nominal)
 *   INA219 OCXO Current (mA, 2A maximum)
 *   TIC (10-bit, 1024ns max)

 *   When a value is not available, field contains "0".
 */

  report_line_no++;              // increment line number, first line is 1
  
  Serialx.print(report_line_no); // line number
  Serialx.print("\t");           // tab
  
  Serialx.print(day);            // date dd/mm/yyyy
  Serialx.print(F("/"));
  Serialx.print(month);
  Serialx.print(F("/"));
  Serialx.print(year);
  Serialx.print(F(" "));         // <space>

  if (hours < 10) {              // time hh:mm:ss
    Serialx.print(F("0"));
  }
  Serialx.print(hours);
  Serialx.print(F(":"));
  if (mins < 10) {
    Serialx.print(F("0"));
  }
  Serialx.print(mins);
  Serialx.print(F(":"));
  if (secs < 10) {
    Serialx.print(F("0"));
  }
  Serialx.print(secs);
  Serialx.print("\t");           // tab

  Serialx.print(updaysstr);      // uptime 000d hh:mm:ss
  Serialx.print(F(" "));         // <space>
  Serialx.print(uptimestr);
  Serialx.print("\t");           // tab

  Serialx.print(fcount64);       // 64-bit counter
  Serialx.print("\t");           // tab

  Serialx.print(calcfreq64);     // frequency
  Serialx.print("\t");           // tab
  Serialx.print(avgften,1);      // avg. 10s
  Serialx.print("\t");           // tab
  Serialx.print(avgfhun,2);      // avg. 100s
  Serialx.print("\t");           // tab
  Serialx.print(avgftho,3);      // avg. 1,000s
  Serialx.print("\t");           // tab 
  Serialx.print(avgftth,4);      // avg. 10,000s
  Serialx.print("\t");           // tab

  Serialx.print(GPSSats);        // sats
  Serialx.print("\t");           // tab
  
  float tempfloat;               // HDOP 
  tempfloat = ( (float) GPSHdop / 100);
  Serialx.print(tempfloat, 2);
  Serialx.print("\t");           // tab

  Serialx.print(adjusted_PWM_output); // PWM
  Serialx.print("\t");           // tab

  float Vctlp = (float(avgpwmVctl)/4096) * 3.3; // PWM Vctl
  Serialx.print(Vctlp);
  Serialx.print("\t");           // tab

  #ifdef GPSDO_VCC
    // Vcc/2 is provided on pin PA0
    float Vcc = (float(avgVcc)/4096) * 3.3 * 2.0;
    Serialx.print(Vcc);
  #else
    Serialx.print("0");
  #endif // VCC
  Serialx.print("\t");           // tab

  #ifdef GPSDO_VDD
    // internal sensor Vref
    float Vdd = (1.21 * 4096) / float(avgVdd); // from STM32F411CEU6 datasheet
                                               // Vdd = Vref on Black Pill
    Serialx.print(Vdd);
  #else
    Serialx.print("0");
  #endif // VDD
  Serialx.print("\t");           // tab

  #if (defined (GPSDO_BMP280_SPI) || defined (GPSDO_BMP280_I2C))
  // BMP280 measurements
    Serialx.print(bmp280temp, 1);
    Serialx.print("\t");
    Serialx.print((bmp280pres+PressureOffset)/100, 1);
  #else
    Serialx.print("0");
    Serialx.print("\t");
    Serialx.print("0");
  #endif // BMP280_SPI
  Serialx.print("\t");           // tab

  #ifdef GPSDO_AHT10
  // AHT10/AHT20 measurements
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
    Serialx.print(temp.temperature);
    Serialx.print("\t");
    Serialx.print(humidity.relative_humidity);
  #else
    Serialx.print("0");
    Serialx.print("\t");
    Serialx.print("0");  
  #endif // AHT10
  Serialx.print("\t");           // tab

  #ifdef GPSDO_INA219
  // current sensor for the OCXO
    Serialx.print(ina219volt, 2);
    Serialx.print("\t");
    Serialx.print(ina219curr, 0);
  #else
    Serialx.print("0");
    Serialx.print("\t");
    Serialx.print("0");
  #endif // INA219  
  Serialx.print("\t");           // tab

  #ifdef GPSDO_TIC
  // Phase difference in ns, WIP
  #else  
    Serialx.print("0"); 
  #endif // INA219
  
  Serialx.println();           // end of line
  
} // end of printGPSDOtab - tab delimited data output

void printGPSDOstats(Stream &Serialx) {     // human readable output
  
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

  if (hours < 10) {
    Serialx.print(F("0"));
  }
  Serialx.print(hours);
  Serialx.print(F(":"));
  if (mins < 10) {
    Serialx.print(F("0"));
  }
  Serialx.print(mins);
  Serialx.print(F(":"));
  if (secs < 10) {
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

  #if (defined (GPSDO_BMP280_SPI) || defined (GPSDO_BMP280_I2C))
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

#ifdef GPSDO_OLED
void displayscreen_OLED() // show GPSDO data on OLED display
{
  float tempfloat;

  // OCXO frequency
  disp.setCursor(0, 1);
  disp.print(F("F "));
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

  #if (defined (GPSDO_BMP280_SPI) || defined (GPSDO_BMP280_I2C))
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

#ifdef GPSDO_LCD_ST7735
void displayscreen_LCD_ST7735() // show GPSDO data on LCD ST7735 display
                                // we use font1 8x6 pix and font2 16x12 pix
{
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

  #if (defined (GPSDO_BMP280_SPI) || defined (GPSDO_BMP280_I2C))
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
  disp.fillRect(144, 19, 15, 20, ST7735_BLACK);
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
#endif // LCD_ST7735

#ifdef GPSDO_LCD_ST7789
void displayscreen_LCD_ST7789() { // show GPSDO data on LCD ST7789 display
                                  // we use Mono fonts and built-in fonts
  float tempfloat;

  // check if we have to clear the display
  if (must_clear_disp_st7789) {
    disp_st7789.fillScreen(ST77XX_BLACK); // clear display
    must_clear_disp_st7789 = false;       // reset flag
  }
  disp_st7789.fillScreen(ST77XX_BLACK); // clear display
  // Display program name and version
  disp_st7789.setTextSize(1);
  disp_st7789.setFont(&FreeMonoBold12pt7b);
  disp_st7789.setTextColor(ST77XX_YELLOW);
  disp_st7789.setCursor(0, 16);
  disp_st7789.print(F("STM32 "));
  disp_st7789.print(F(Program_Name));
  disp_st7789.setFont(&FreeMono9pt7b);
  disp_st7789.setTextColor(ST77XX_CYAN);
  disp_st7789.setCursor(168, 11);
  disp_st7789.print(F(Program_Version));
    
  // Latitude
  disp_st7789.setTextColor(ST77XX_WHITE);
  disp_st7789.setCursor(2, 60);
  disp_st7789.print(F("Lat: "));
  // disp_st7789.print(GPSLat, 6);
  disp_st7789.print("12.345678 "); // test to see if there is a problem with printing strings vs floats
  // Longitude
  disp_st7789.setCursor(2, 74);
  disp_st7789.print(F("Lon: "));
  disp_st7789.print(GPSLon, 6);
  // Altitude 
  disp_st7789.setCursor(2, 88);
  disp_st7789.print(F("Alt: "));
  disp_st7789.print(GPSAlt, 1);
  disp_st7789.print(F("m  "));
  //Satellites
  disp_st7789.setCursor(140, 88);
  disp_st7789.print(F("Sats: "));
  disp_st7789.print(GPSSats);
  if (GPSSats < 10) disp_st7789.print(F(" ")); // clear possible digit when sats >= 10
  // HDOP
  disp_st7789.setCursor(2, 102);
  disp_st7789.print(F("HDOP: "));
  tempfloat = ((float) GPSHdop / 100);
  disp_st7789.print(tempfloat, 1);
  disp_st7789.setCursor(2, 116);
  disp_st7789.print(F("UpT: "));  
  disp_st7789.print(updaysstr);
  disp_st7789.setCursor(140, 116);
  disp_st7789.print(uptimestr);

  #if (defined (GPSDO_BMP280_SPI) || defined (GPSDO_BMP280_I2C))
    // BMP280 temperature
    disp_st7789.setCursor(2, 130);
    disp_st7789.print(F(" "));
    //disp_st7789.print((char)247);
    //disp_st7789.print((char)9);
    disp_st7789.print(F("*C: "));
    disp_st7789.print(bmp280temp, 1);
    // BMP280 pressure
    disp_st7789.setCursor(2, 144);
    disp_st7789.print(F("hPa: "));
    disp_st7789.print(((bmp280pres+PressureOffset)/100), 1);
  #endif // BMP280_SPI

  #ifdef GPSDO_VCC
    disp_st7789.setCursor(2, 160);
    // Vcc/2 is provided on pin PA0
    float Vcc = (float(avgVcc)/4096) * 3.3 * 2.0;
    disp_st7789.print(F("5V0: "));
    disp_st7789.print(Vcc, 2);
    disp_st7789.print(F("V"));
  #endif // VCC

  #ifdef GPSDO_VDD
    // internal sensor Vref
    disp_st7789.setCursor(2, 174);
    float Vdd = (1.21 * 4096) / float(avgVdd); // from STM32F411CEU6 datasheet                 
    disp_st7789.print(F("3V3: "));
    disp_st7789.print(Vdd, 2);                    // Vdd = Vref on Black Pill
    disp_st7789.print(F("V"));
  #endif // VDD
/*
  disp_st7789.setCursor(0, 80);  // display PWM/DAC value
  #ifdef GPSDO_PWM_DAC
    disp_st7789.print(F("PWM: "));
    disp_st7789.print(adjusted_PWM_output);
    disp_st7789.print(F(trendstr));
  #endif // PWM_DAC
  
  // display Vctl value
  disp_st7789.setCursor(90, 80);  // display Vctl value
  float Vctlp = (float(avgpwmVctl)/4096) * 3.3;
  disp_st7789.print(F("Vctl: "));
  disp_st7789.print(Vctlp);
  disp_st7789.print(F("V"));
*/  
  // OCXO frequency
  disp_st7789.setFont();          // reset font to built-in
  disp_st7789.setTextSize(2);
  disp_st7789.setCursor(10, 24);
  disp_st7789.setTextColor(ST77XX_RED, ST77XX_BLACK);

  // display 1s, 10s or 100s value depending on whether data is available
  if (cbTen_full) {
    if (cbTho_full) { // if we have data over 1000 seconds
      if (avgftho < 10000000) {
        disp_st7789.print(" ");
      }
      disp_st7789.print(avgftho, 3); // to 3 decimal places
    }
      else if (cbHun_full) {
        if (avgfhun < 10000000) {
      disp_st7789.print(" ");
          }
          disp_st7789.print(avgfhun, 2); // to 2 decimal places
        disp_st7789.print(" ");
    }

  else { // nope, only 10 seconds
    if (avgften < 10000000) {
      disp_st7789.print(" ");
    }
    disp_st7789.print(avgften, 1); // to 1 decimal place
    disp_st7789.print("  ");
    }
  }
  else { // we don't have any averages and print integer value
    calcfreqint = calcfreq64;       // convert to 32-bit integer
    if (calcfreqint < 10000000) {
      disp_st7789.print(" ");
    }
    disp_st7789.print(calcfreqint); // integer
  disp_st7789.print("    ");        // these are used for more exact display
  }
  // due to limited space small character unit
  // disp_st7789.setTextSize(1);
  // disp_st7789.setCursor(168, 20);
  // due to some unknown issue in printing the freq digits/unit we clear the section of the display first
  // disp_st7789.fillRect(144, 19, 15, 20, ST77XX_BLACK);
  disp_st7789.print(" Hz");

  // clock and date
  disp_st7789.setTextSize(2);
  disp_st7789.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
  // Date
  disp_st7789.setCursor(2, 188);
  //disp_st7789.print(F("Date: "));  
  disp_st7789.print(day);
  disp_st7789.print(F("."));
  disp_st7789.print(month);
  disp_st7789.print(F("."));
  disp_st7789.print(year);
  
  // Time
  disp_st7789.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  disp_st7789.setCursor(2, 208);

  if (hours < 10) {
    disp_st7789.print(F("0"));
  }
  disp_st7789.print(hours);
  disp_st7789.print(F(":"));

  if (mins < 10) {  
    disp_st7789.print(F("0"));
  }
  disp_st7789.print(mins);
  disp_st7789.print(F(":"));

  if (secs < 10) {  
    disp_st7789.print(F("0"));
  }
  disp_st7789.print(secs);
  
  disp_st7789.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  disp_st7789.print(F(" UTC"));
  
  // reset all font stuff for normal display
  // disp_st7789.setFont();
  // disp_st7789.setTextSize(1);
  // disp_st7789.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
}
#endif // LCD_ST7789

#ifdef GPSDO_TM1637
  void displaytime_TM1637() {
    int LEDtime = (hours * 100) + mins;
    // Show UTC or local time on TM1637 4-digit LED display and blink colon at 1Hz
    if ((secs%2) == 0) tm1637.showNumberDecEx(LEDtime, 0b11100000, true);
    else tm1637.showNumberDec(LEDtime, true);
  }
#endif // TM1637

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
} // end of uptimetostrings()


// ---------------------------------------------------------------------------------------------
//    setup routine, prepares the hardware for normal operation
// --------------------------------------------------------------------------------------------- 
void setup()
{
  // Wait 1 second for things to stabilize
  delay(1000);

  // setup 2kHz test signal on PB5, can be tested with an oscilloscope
  #ifdef GPSDO_GEN_2kHz_PB5                 // note this uses Timer 3 Channel 2
    analogWrite(Test2kHzOutputPin, 127);    // configures PB5 as PWM output pin at default frequency and resolution
    analogWriteFrequency(2000);             // default PWM frequency is 1kHz, change it to 2kHz
    analogWriteResolution(16);              // default PWM resolution is 8 bits, change it to 16 bits
    analogWrite(Test2kHzOutputPin, 32767);  // 32767 for 16 bits -> 50% duty cycle so a square wave
  #endif // GEN_2kHz_PB5
   
  // configure blueledpin in output mode
  pinMode(blueledpin, OUTPUT);    // blinking blue LED indicates interrupts are working

  // configure yellow_led_pin in output mode 
  pinMode(yellowledpin, OUTPUT);  // yellow LED is used to indicate status of the GPSDO  
  
  // Setup 2Hz Timer and its interrupt service routine
  HardwareTimer *tim2Hz = new HardwareTimer(TIM9);
  tim2Hz->setOverflow(2, HERTZ_FORMAT); // 2 Hz
  tim2Hz->attachInterrupt(Timer_ISR_2Hz);
  tim2Hz->resume();

  // Setup UART (serial) interfaces
  Serial.begin(115200);      // USB serial
  Serial1.begin(9600);       // Hardware serial 1 to GPS module
  #ifdef GPSDO_BLUETOOTH
    // HC-06 module baud rate factory setting is 9600, 
    // IMPORTANT! Use separate program to set baud rate to 57600
    Serial2.begin(BT_BAUD); // Hardware serial 2 to Bluetooth module
  #endif // BLUETOOTH

  Serial.println();
  Serial.println(F(Program_Name));
  Serial.println(F(Program_Version));
  Serial.println();
  Serial.println(F("2Hz interrupt configured"));    
  Serial.println(F("Serial interfaces configured"));    

  // setup commands parser
  serial_commands_.SetDefaultHandler(cmd_unrecognized);
  serial_commands_.AddCommand(&cmd_version_);
  serial_commands_.AddCommand(&cmd_flush_);
  serial_commands_.AddCommand(&cmd_calibrate_);
  serial_commands_.AddCommand(&cmd_tunnel_);
  serial_commands_.AddCommand(&cmd_setPWM_);
  serial_commands_.AddCommand(&cmd_rephum_);
  serial_commands_.AddCommand(&cmd_repdel_);
  
  serial_commands_.AddCommand(&cmd_up1_);
  serial_commands_.AddCommand(&cmd_up10_);
  serial_commands_.AddCommand(&cmd_dp1_);
  serial_commands_.AddCommand(&cmd_dp10_);

  #ifdef GPSDO_EEPROM
    // register the commands to Store PWM and Recall PWM
  #endif // EEPROM 

  Serial.println(F("Commands parser configured"));    
    
  #ifdef GPSDO_LCD_ST7735
    // Setup LCD SPI ST7735 display
    disp.initR(INITR_BLACKTAB); // 1.8" LCD
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

    Serial.println(F("ST7735 LCD display configured"));
  #endif // LCD_ST7735

  // Setup LCD SPI ST7789 display
  #ifdef GPSDO_LCD_ST7789
    disp_st7789.init(240, 240, SPI_MODE3); // 1.3" 240x240 TFT LCD
    delay(100);
    disp_st7789.fillScreen(ST77XX_BLACK);
    disp_st7789.setRotation(2);   // 0..3 max, 1 = 90° = landscape
    // Display program name and version
    disp_st7789.setTextSize(1);
    disp_st7789.setFont(&FreeMonoBold12pt7b);
    disp_st7789.setTextColor(ST77XX_YELLOW);
    disp_st7789.setCursor(0, 16);
    disp_st7789.print(F("STM32 "));
    disp_st7789.print(F(Program_Name)); // program name
    disp_st7789.setFont(&FreeMono9pt7b);
    disp_st7789.setTextColor(ST77XX_CYAN);
    disp_st7789.setCursor(168, 11);
    disp_st7789.print(F(Program_Version)); // program version

    Serial.println(F("ST7789 LCD display configured"));
  #endif // LCD_ST7789

  // Setup TM1637 4-digit LED module
  #ifdef GPSDO_TM1637
    // Set the display brightness (0-7):
    tm1637.setBrightness(5);
    // Clear the display:
    tm1637.clear();
    tm1637.setSegments(mid_dashes);

    Serial.println(F("TM1637 4-digit LED clock display configured"));
  #endif // TM1637

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

    Serial.println(F("u-blox GPS module configured"));  
  #endif // UBX_CONFIG

  // Setup AHT10 / AHT20 temperature and humidity sensor module
  #ifdef GPSDO_AHT10 // same code for AHT20
    Serial.println(F("Testing for presence of AHT10 or AHT20 Sensor on I2C bus"));
    if (!aht.begin()) {
      Serial.println(F("Could not find AHT10 or AHT20 sensor, check wiring"));
      while (1) delay(10);
    }
    else Serial.println(F("AHTX0 sensor configured"));
  #endif // AHT10 - Note this seems to initialize the I2C bus interface

  // Setup OLED I2C display
  #ifdef GPSDO_OLED
    // Note that u8x8 library initializes I2C hardware interface
    // disp.setBusClock(400000L); // try to avoid display locking up
    disp.begin();
    disp.setFont(u8x8_font_chroma48medium8_r);
    disp.clear();
    disp.setCursor(0, 0);
    disp.print(F(Program_Name));
    disp.print(F(" - "));
    disp.print(F(Program_Version));

    Serial.println(F("OLED display configured"));
  #endif // OLED 
  
  // Setup INA219 current sensor module
  #ifdef GPSDO_INA219
    // By default the initialization will use the largest range (32V, 2A).  However
    // you can call a setCalibration function to change this range (see comments).
    if (! ina219.begin()) {
      Serial.println(F("Could not find INA219 sensor, check wiring"));
      while (1) { delay(10); }
    }
    else Serial.println(F("INA219 sensor configured"));
    // To use a slightly lower 32V, 1A range (higher precision on amps):
    //ina219.setCalibration_32V_1A();
    // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
    //ina219.setCalibration_16V_400mA();
    ina219.setCalibration_32V_1A();
  #endif // INA219 
   
  analogReadResolution(12); // make sure we read 12 bit values when we read from any ADC analog channel 

  // generate a 2kHz square wave on PB9 PWM pin, using Timer 4 channel 4
  // PB9 is Timer 4 Channel 4 from Arduino_Core_STM32/variants/STM32F4xx/F411C(C-E)(U-Y)/PeripheralPins_BLACKPILL_F411CE.c
  analogWrite(VctlPWMOutputPin, 127);                   // configures PB9 as PWM output pin at default frequency and resolution
  analogWriteFrequency(2000);                           // default PWM frequency is 1kHz, change it to 2kHz
  analogWriteResolution(16);                            // set PWM resolution to 16 bits (the maximum for the STM32F411CEU6)
  adjusted_PWM_output = default_PWM_output;             // initial PWM value
  analogWrite(VctlPWMOutputPin, adjusted_PWM_output);   // 32767 for 16 bits -> 50% duty cycle so a square wave
  Serial.println(F("16-bit PWM DAC configured"));

  #if (defined (GPSDO_BMP280_SPI) || defined (GPSDO_BMP280_I2C))
  // Initialize BMP280
  Serial.println(F("Testing for presence of BMP280 Sensor on I2C bus"));
  #ifdef GPSDO_BMP280_I2C // BMP280 on I2C bus requires specifying address
    if (!bmp.begin(0x76,0x58)) {
  #else
    if (!bmp.begin()) {
  #endif
      Serial.println(F("Could not find BMP280 sensor, check wiring"));
      while (1) delay(10);
    }
    else Serial.println(F("BMP280 sensor configured"));
  
    // Default settings from datasheet
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode 
                  Adafruit_BMP280::SAMPLING_X2,        // Temp. oversampling
                  Adafruit_BMP280::SAMPLING_X16,       // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,         // Filtering
                  Adafruit_BMP280::STANDBY_MS_500);    // Standby time

  #endif // BMP280_SPI or BMP280_I2C
  
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
  
  avg_pwmVctl.begin();
  pwmVctl = analogRead(VctlPWMInputPin);
  avgpwmVctl = avg_pwmVctl.reading(pwmVctl);

  startGetFixmS = millis();
  
  Serial.println();
  Serial.println(F("GPSDO Starting"));
  Serial.println();

  #ifdef GPSDO_EEPROM
    // detect signature and if available, retrieve 16-bit PWM value
  #endif // EEPROM   

} // end of setup()


// ---------------------------------------------------------------------------------------------
//    loop routine: this is the main loop
// --------------------------------------------------------------------------------------------- 
void loop()
{
  serial_commands_.ReadSerial();  // process any command from either USB serial (usually 
                                  // the Arduino monitor) xor Bluetooth serial (e.g. a smartphone)
  if (force_calibration_flag) docalibration(); else

  if (tunnel_mode_flag) tunnelgps(); else
  
  if (gpsWaitFix(waitFixTime))    // wait up to waitFixTime seconds for fix, returns true if we have a fix
  { 
    // if we have a GPS fix (implies we have a stable 1PPS pulse from the GPS)
    
    if (!report_tab_delimited) { // only report fix time when in human readable output format mode
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
    } // !report_tab_delimited

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
      #ifdef GPSDO_PWM_DAC
        adjustVctlPWM();
      #endif // PWM_DAC
    }

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

    #if (defined (GPSDO_BMP280_SPI) || defined (GPSDO_BMP280_I2C))
      bmp280temp = bmp.readTemperature();              // read bmp280 sensor, save values
      bmp280pres = bmp.readPressure();
      bmp280alti = bmp.readAltitude();
    #endif // BMP280_SPI or BMP280_SPI   

    #ifdef GPSDO_INA219
      ina219volt = ina219.getBusVoltage_V();           // read ina219 sensor, save values
      ina219curr = ina219.getCurrent_mA();
    #endif // INA219 
  
    uptimetostrings();            // get updaysstr and uptimestr

    yellow_led_state = 0;         // turn off yellow LED

    if (report_tab_delimited) {   // check what to print and where
      #ifdef GPSDO_BLUETOOTH
        printGPSDOtab(Serial2);   // print tabulated data to Bluetooth Serial
      #else                       // xor
        printGPSDOtab(Serial);    // print tabulated data to USB Serial
      #endif // BLUETOOTH
    }
    else {
      #ifdef GPSDO_BLUETOOTH
        printGPSDOstats(Serial2);   // print stats to Bluetooth Serial
      #else                         // xor
        printGPSDOstats(Serial);    // print stats to USB Serial
      #endif // BLUETOOTH
    } // end if (report_tab_delimited)

    #ifdef GPSDO_OLED             // show stats on various displays
      displayscreen_OLED();
    #endif // OLED

    #ifdef GPSDO_LCD_ST7735
      displayscreen_LCD_ST7735();
    #endif // LCD_ST7735  
    
    #ifdef GPSDO_LCD_ST7789
      displayscreen_LCD_ST7789();
    #endif // LCD_ST7789  
    
    #ifdef GPSDO_TM1637
      displaytime_TM1637();
    #endif // TM1637  
    
    startGetFixmS = millis();    // have a fix, next thing that happens is checking for a fix, so restart timer
  }
  else // no GPS fix could be acquired for the last 1/2/5 seconds (see settings)
  {
    // we don't have a GPS fix, so we consider that we don't have a stable 1PPS
    yellow_led_state = 1;        // turn on yellow LED to show something is not right
    
    #ifdef GPSDO_OLED
      disp.clear();                // display no fix message on OLED
      disp.setCursor(0, 0);
      disp.print(F(Program_Name));
      disp.print(F(" - "));
      disp.print(F(Program_Version));
      disp.setCursor(0, 1);
      disp.print(F("Wait fix "));
      disp.print( (millis() - startGetFixmS) / 1000 );
      disp.print(F("s"));
    #endif // OLED

    #ifdef GPSDO_LCD_ST7735
      disp.fillScreen(ST7735_BLACK);  // display no fix message on ST7735 LCD
      disp.setCursor(0, 0);
      disp.print(F(Program_Name));
      disp.print(F(" - "));
      disp.print(F(Program_Version));
      disp.setCursor(0, 8);
      disp.print(F("Wait fix "));
      disp.print( (millis() - startGetFixmS) / 1000 );
      disp.print(F("s"));
    #endif // LCD_ST7735

    #ifdef GPSDO_LCD_ST7789
      // display no fix message on ST7789 LCD
      disp_st7789.fillScreen(ST77XX_BLACK); // clear display
      // Display program name and version
      disp_st7789.setTextSize(1);
      disp_st7789.setFont(&FreeMonoBold12pt7b);
      disp_st7789.setTextColor(ST77XX_YELLOW);
      disp_st7789.setCursor(0, 16);
      disp_st7789.print(F("STM32 "));
      disp_st7789.print(F(Program_Name));
      disp_st7789.setFont(&FreeMono9pt7b);
      disp_st7789.setTextColor(ST77XX_CYAN);
      disp_st7789.setCursor(168, 11);
      disp_st7789.print(F(Program_Version));
      // display wait fix message and seconds count
      disp_st7789.setCursor(0, 36);
      disp_st7789.setTextColor(ST77XX_WHITE);
      disp_st7789.print(F("    Wait fix "));
      disp_st7789.print( (millis() - startGetFixmS) / 1000 );
      disp_st7789.print(F("s"));

      must_clear_disp_st7789 = true;      
    #endif // LCD_ST7789

    #ifdef GPSDO_TM1637
      tm1637.setSegments(low_oooo_s);
    #endif // TM1637    

    if (report_tab_delimited) {     // check what to print and where
      #ifdef GPSDO_BLUETOOTH        // print 0 message to either
        Serial2.println(F("0"));
      #else
        Serial.println(F("0"));
      #endif // BLUETOOTH
      report_line_no = 0;           // we restart linecount every time we lose GPS position fix (PPS)
    }
    else {
      #ifdef GPSDO_BLUETOOTH        // print no fix message to either
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
    }
    
    // no fix or fix lost, we must flush the ring buffers,
    // so raise flush_ring_buffers_flag
    flush_ring_buffers_flag = true;
  }
  
} // end of loop()
