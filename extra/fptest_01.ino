/* Floating point math test on the STM32F411CEU6
 *  The following code tests for rounding errors / weird floating point math
 *  that could potentiallycause wrong results in the actual STM32 GPSDO code.
 *  The code below is very similar to the actual code used to calculate
 *  floating point frequency averages from the data in the ring buffers.
 * So, do I need to worry? 
 * Here are the results printed to serial monitor:
      Floating point on STM32, precision test
      One decimal test
      10000000.3
      Two decimals test
      10000000.23
      Three decimals test
      10000000.123
      Four decimals test
      10000000.1234
      ... (repeated every 5 s)
 *
 * Conclusion: no, I don't need to worry! :)
 */

// Some global variables
double dbten=0, dbhun=0, dbtho=0, dbtth=0;
 
void setup() {
  Serial.begin(115200); // results are printed on the serial monitor
}

void loop() {
  Serial.println(F("Floating point on STM32, precision test"));
  
  // One decimal
  uint64_t x1 = 200000123;
  uint64_t x2 = 100000120;
  dbten = double(x1 - x2) / 10.0;
  Serial.println(F("One decimal test"));
  Serial.println(dbten, 1);

  // Two decimals
  uint64_t y1 = 2000000123;
  uint64_t y2 = 1000000100;
  dbhun = double(y1 - y2) / 100.00;
  Serial.println(F("Two decimals test"));
  Serial.println(dbhun, 2);  

  // Three decimals
  uint64_t z1 = 20000000123;
  uint64_t z2 = 10000000000;
  dbtho = double(z1 - z2) / 1000.000;
  Serial.println(F("Three decimals test"));
  Serial.println(dbtho, 3);
 
  // Four decimals
  uint64_t w1 = 200000001234;
  uint64_t w2 = 100000000000;
  dbtth = double(w1 - w2) / 10000.0000;
  Serial.println(F("Four decimals test"));
  Serial.println(dbtth, 4);
 
  delay(5000);
}
