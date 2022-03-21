// lcd_test_1a
// by André Derrick Balsa (AndrewBCN)
// March 2022
// GPLV3
// Testing the LCD display with the STM32F401CCU6
// Also test SPI bus and I2C bus and various sensors

#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x02020000)
  #error "Due to API changes, this sketch is compatible with STM32_CORE_VERSION >= 0x02020000 (2.2.0 or later)"
#endif

// Increase HardwareSerial (UART) TX and RX buffer sizes from default 64 characters to 256.
// The main worry here is that we could miss some characters from the u-blox GPS module if
// the processor is busy doing something else (e.g. updating the display, reading a sensor, etc)
// specially since we increase the GPS baud rate from 9600 to 38400.

#define SERIAL_TX_BUFFER_SIZE 256       // Warning: > 256 could cause problems, see comments in STM32 HardwareSerial library
#define SERIAL_RX_BUFFER_SIZE 256

#define GPSDO_GEN_2kHz_PB5       // generate 2kHz square wave test signal on pin PB5 using Timer 3
#define GPSDO_INA219             // INA 219 current sensor

#ifdef GPSDO_GEN_2kHz_PB5
  #define Test2kHzOutputPin PB5    // digital output pin used to output a test 2kHz square wave
#endif // GEN_2kHz_PB5

#ifdef GPSDO_INA219
  #include <Adafruit_INA219.h>
  Adafruit_INA219 ina219;
#endif // INA219

#define VctlPWMOutputPin PB9     // digital output pin used to output a PWM value, TIM4 ch4
                                 // Two cascaded RC filters transform the PWM into an analog DC value

#include <Wire.h>                // Hardware I2C library on STM32
                                 // Uses PB6 (SCL1) and PB7 (SDA1) on Black Pill for I2C1
#include <SPI.h>                 // Hardware SPI library on STM32
                                 // Uses PA5, PA6, PA7 on Black Pill for SPI1
// AHT20 - I2C
#include <Adafruit_AHTX0.h>      // Adafruit AHTX0 library for AHT20 I2C temperature and humidity sensor
Adafruit_AHTX0 aht20;            // create object aht20

// BMP280 - I2C
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp280;          // hardware I2C
Adafruit_Sensor *bmp_temp = bmp280.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp280.getPressureSensor();

// TFT LCD ST7789 - SPI
#include <Adafruit_GFX.h>       // need this adapted for STM32F4xx/F411C: https://github.com/fpistm/Adafruit-GFX-Library/tree/Fix_pin_type
#include <Adafruit_ST7789.h>
//#include <Fonts/FreeSansBold18pt7b.h>

#define TFT_DC  PB12            // note this pin assigment conflicts with the original GPSDO schematic
#define TFT_CS  PB13            // in reality, not connected, CS not used on 1.3" TFT ST7789 display
#define TFT_RST PB15            // also uses pins PA5, PA6, PA7 for MOSI MISO and SCLK

Adafruit_ST7789 disp = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// LEDs
// Blue onboard LED blinks to indicate ISR is working
#define blueledpin  PC13    // Blue onboard LED is on PC13 on STM32F411CEU6 Black Pill
// Yellow extra LED is off, on or blinking to indicate some GPSDO status
#define yellowledpin PB8   // Yellow LED on PB8
volatile int yellow_led_state = 2;  // global variable 0=off 1=on 2=1Hz blink

// Uptime data
volatile uint8_t  uphours = 0;
volatile uint8_t  upminutes = 0;
volatile uint8_t  upseconds = 0;
volatile uint16_t updays = 0;
volatile bool halfsecond = false;
char uptimestr[9] = "00:00:00";    // uptime string
char updaysstr[5] = "000d";        // updays string


// Interrupt Service Routine for the 2Hz timer
void Timer_ISR_2Hz(void) {  // WARNING! Do not attempt I2C communication inside the ISR

  // Toggle pin. 2hz toogle --> 1Hz pulse, perfect 50% duty cycle
  digitalWrite(blueledpin, !digitalRead(blueledpin));

  halfsecond = !halfsecond; // true @ 1Hz
  
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
} // end of 2Hz ISR

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

void setup() {
  // Wait 1 second for things to stabilize
  delay(1000);

  // setup USB serial
  Serial.begin(9600);
  Serial.println(F("LCD display test"));    

  // configure blueledpin in output mode
  pinMode(blueledpin, OUTPUT);

  // configure yellow_led_pin in output mode
  pinMode(yellowledpin, OUTPUT);      
  
  // setup 2kHz test signal on PB5 if configured, uses Timer 3
  #ifdef GPSDO_GEN_2kHz_PB5                 // note this uses Timer 3 Channel 2
  analogWrite(Test2kHzOutputPin, 127);      // configures PB5 as PWM output pin at default frequency and resolution
  analogWriteFrequency(2000);               // default PWM frequency is 1kHz, change it to 2kHz
  analogWriteResolution(16);                // default PWM resolution is 8 bits, change it to 16 bits
  analogWrite(Test2kHzOutputPin, 32767);    // 32767 for 16 bits -> 50% duty cycle so a square wave
  #endif // GEN_2kHz_PB5

  // setup Vctl PWM DAC, output approximately 1.65V for testing purposes
  // we generate a 2kHz square wave on PB9 PWM pin, using Timer 4 channel 4
  // PB9 is Timer 4 Channel 4 from Arduino_Core_STM32/variants/STM32F4xx/F411C(C-E)(U-Y)/PeripheralPins_BLACKPILL_F411CE.c
  analogWrite(VctlPWMOutputPin, 127);      // configures PB9 as PWM output pin at default frequency and resolution
  analogWriteFrequency(2000);              // default PWM frequency is 1kHz, change it to 2kHz
  analogWriteResolution(16);               // set PWM resolution to 16 bits (the maximum for the STM32F411CEU6)
  analogWrite(VctlPWMOutputPin, 32767);    // 32767 for 16 bits -> 50% duty cycle so a square wave
    
  // setup 2Hz timer and interrupt, uses Timer 9
  HardwareTimer *tim2Hz = new HardwareTimer(TIM9);
  tim2Hz->setOverflow(2, HERTZ_FORMAT); // 2 Hz
  tim2Hz->attachInterrupt(Timer_ISR_2Hz);
  tim2Hz->resume();
   
  // setup sensors and LCD display
  // AHT20, BMP280, INA219, ST7789 240x240 TFT LCD
  
  Serial.println(F("Testing for presence of AHT20 Sensor on I2C bus"));
  if (!aht20.begin()) {
    Serial.println(F("Could not find AHT20 sensor, check wiring"));
    while (1) delay(10);
  }
  else Serial.println(F("AHT20 sensor found!"));

  Serial.println(F("Testing for presence of BMP280 Sensor on I2C bus"));
  if (!bmp280.begin(0x76,0x58)) {
    Serial.println(F("Could not find BMP280 sensor, check wiring"));
    while (1) delay(10);
  }
  else Serial.println(F("BMP280 sensor found!"));
  
  // Default settings from datasheet
  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode 
                  Adafruit_BMP280::SAMPLING_X2,        // Temp. oversampling
                  Adafruit_BMP280::SAMPLING_X16,       // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,         // Filtering
                  Adafruit_BMP280::STANDBY_MS_500);    // Standby time

  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  if (! ina219.begin()) {
    Serial.println(F("Could not find INA219 sensor, check wiring"));
    while (1) { delay(10); }
  }
  else Serial.println(F("INA219 sensor found!"));
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();
  ina219.setCalibration_32V_1A();
  
  // Setup 240x240 LCD SPI ST7789 display
  disp.init(240, 240, SPI_MODE3); // 1.3" 240x240 TFT LCD
  delay(500);
  disp.fillScreen(ST77XX_BLACK);
  disp.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);  //
  disp.setRotation(2);   // 0..3 max, here we use 180° = landscape
  disp.setFont();
  disp.setTextSize(3);
  disp.setCursor(0, 30);
  disp.print(F("Testing..."));
  disp.setTextSize(2);
  disp.setCursor(0, 60);
  disp.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  disp.print(F(" Smaller text - "));
  disp.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  disp.print(F("123"));
  disp.setCursor(0, 80);
  disp.setTextColor(ST77XX_RED, ST77XX_BLACK);
  disp.print(F("Different colors."));
  disp.setTextSize(3);
  disp.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  disp.setCursor(0, 120);
  disp.print(F("STM32 GPSDO"));
  disp.setTextSize(2);
  disp.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
  disp.setCursor(0, 150);
  disp.print(F("  Version v0.99z"));
  disp.setTextColor(ST77XX_BLUE, ST77XX_BLACK);
  disp.setCursor(0, 180);
  disp.print(F("  ... not really!"));
} // setup done

void loop() {
  // print something once per second to USB serial (Arduino monitor)

  uptimetostrings();           // get updaysstr and uptimestr
  Serial.print(F("Uptime: "));
  Serial.print(updaysstr);
  Serial.print(F(" "));
  Serial.println(uptimestr);

  Serial.println(); 

  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  
  Serial.println(F("BMP280 Sensor Readings"));
  Serial.print(F("Temperature = "));
  Serial.print(temp_event.temperature);
  Serial.println(F(" *C"));

  Serial.print(F("Pressure = "));
  Serial.print(pressure_event.pressure);
  Serial.println(F(" hPa"));
  
  Serial.println();

  Serial.println(F("AHT20 Sensor Readings"));
  sensors_event_t humidity, temp;
  aht20.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data
  Serial.print(F("Temperature = "));
  Serial.print(temp.temperature);
  Serial.println(F(" *C"));
  
  Serial.print(F("Humidity = "));
  Serial.print(humidity.relative_humidity);
  Serial.println(F("% rH"));

  Serial.println(); 

  // Read INA 219 current voltage sensor
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  
  Serial.println("");

  delay(2000);
}
