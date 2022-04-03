// eeprom_emulation_test_1a
// by André Derrick Balsa (AndrewBCN)
// April 2022
// GPLV3
// Testing STM32duino EEPROM emulation library (emulates EEPROM in Flash)

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
#define GPSDO_EEPROM            // enable STM32 buffered EEPROM emulation library

#ifdef GPSDO_GEN_2kHz_PB5
  #define Test2kHzOutputPin PB5    // digital output pin used to output a test 2kHz square wave
#endif // GEN_2kHz_PB5

// EEPROM emulation in flash
#ifdef GPSDO_EEPROM
  #include <EEPROM.h>                              // Buffered EEPROM emulation library
  #define DATA_LENGTH E2END
  const uint32_t eepromsize = DATA_LENGTH;
#endif // EEPROM

char signature[6] = "STM32";
bool sigalreadywritten = true;

// LEDs
// Blue onboard LED blinks to indicate ISR is working
#define blueledpin  PC13    // Blue onboard LED is on PC13 on STM32F411CEU6 Black Pill

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
  Serial.println(F("EEPROM library test"));    

  // configure blueledpin in output mode
  pinMode(blueledpin, OUTPUT);
  
  // setup 2kHz test signal on PB5 if configured, uses Timer 3
  #ifdef GPSDO_GEN_2kHz_PB5                 // note this uses Timer 3 Channel 2
  analogWrite(Test2kHzOutputPin, 127);      // configures PB5 as PWM output pin at default frequency and resolution
  analogWriteFrequency(2000);               // default PWM frequency is 1kHz, change it to 2kHz
  analogWriteResolution(16);                // default PWM resolution is 8 bits, change it to 16 bits
  analogWrite(Test2kHzOutputPin, 32767);    // 32767 for 16 bits -> 50% duty cycle so a square wave
  #endif // GEN_2kHz_PB5

  // setup 2Hz timer and interrupt, uses Timer 9
  HardwareTimer *tim2Hz = new HardwareTimer(TIM9);
  tim2Hz->setOverflow(2, HERTZ_FORMAT); // 2 Hz
  tim2Hz->attachInterrupt(Timer_ISR_2Hz);
  tim2Hz->resume();

  delay(1000);  

} // setup done

void loop() {
  // print something once per second to USB serial (Arduino monitor)
  uint32_t i = 0;

  uptimetostrings();           // get updaysstr and uptimestr
  Serial.print(F("Uptime: "));
  Serial.print(updaysstr);
  Serial.print(F(" "));
  Serial.println(uptimestr);

  delay(1000);

  for (i = 0; i < 6; i++) {
    Serial.print(signature[i]);
  }  
  Serial.println();

  eeprom_buffer_fill(); // fill the buffer with contents of Flash emulating EEPROM

  Serial.print(F("EEPROM size: "));
  Serial.println(eepromsize);

  sigalreadywritten = true;
  for (i = 0; i < 6; i++) {
    if (eeprom_buffered_read_byte(i) != signature[i]) sigalreadywritten = false;
    Serial.println(eeprom_buffered_read_byte(i));
    delay(500);
  }

  uint32_t writetime = millis();

  if (sigalreadywritten) {
    Serial.println(F("Signature already written, skipping EEPROM buffer flush"));
  }
  else {
    for (i = 0; i < 6; i++) {
      eeprom_buffered_write_byte(i, signature[i]);  // write signature to buffer
    }  
    eeprom_buffer_flush();                          // and flush buffer
  }
 
  Serial.print(F("Write time: "));                  // printout how long it took to flush the buffer
  Serial.print(millis() - writetime);
  Serial.println(F("ms"));
}
