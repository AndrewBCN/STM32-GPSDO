// Very short program just used to configure HC-06 Bluetooth modules
// for name and baud rate, using AT commands
HardwareSerial Serial2(PA3, PA2);   // Serial to Bluetooth module


void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600);  // serial to GPS module
  Serial2.begin(115200);  // serial to Bluetooth module
  
  Serial.begin(115200); // USB serial

  Serial.print(F("Check HC-06 module version, set baud rate to 115200 and rename to GPSDO1"));
  Serial.println();
  // check module version
  Serial2.println("AT+VERSION"); // module should answer with HC06 version
  delay(1500);
  if (Serial2.available()) Serial.println("Response="+Serial2.readString());
  // change BT module name
  Serial2.println("AT+NAMEGPSDO1"); // change 1 to 2 or 3 or etc
  delay(1500);
  if (Serial2.available()) Serial.println("Response="+Serial2.readString());
  // Change BT module baud rate (default is 9600) 
//  Serial2.println("AT+BAUD8"); // 8=115200  
//  delay(1500);
//  if (Serial2.available()) Serial.println("Response="+Serial2.readString());  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Done setting HC06, turn off development board now");
  delay(5000);
}
