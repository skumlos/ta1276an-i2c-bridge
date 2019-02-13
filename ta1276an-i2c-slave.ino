#define I2C_TIMEOUT 1000
#define I2C_PULLUP 1
#define SDA_PORT PORTA
#define SDA_PIN 0 // = 22
#define SCL_PORT PORTA
#define SCL_PIN 2 // = 24
#include <SoftI2CMaster.h>
#include <Wire.h>
#define TA1276AN_ADDR (68)

void setup() {
  Serial.begin(9600);
  Serial.print("TA1276AN I2C Bridge");
  Wire.begin(TA1276AN_ADDR);
  Wire.onReceive(writeRequest);
  Wire.onRequest(readRequest);
  if (!i2c_init()) Serial.println("I2C init failed");
}

void loop() {
  delay(100);
}

void writeRequest(int howMany) {
  Serial.print("Write ");
  int bytes[howMany];
  int p = 0;
  while (Wire.available()) { // loop through all but the last
    int c = Wire.read(); // receive byte as a character
    bytes[p] = c;
    ++p;
    Serial.print(c,HEX); // print the character
    Serial.print(",");
  }
  Serial.print("\n");
  if (!i2c_start_wait((TA1276AN_ADDR<<1)|I2C_WRITE)) {
    Serial.println("I2C device busy");
    delay(1000);
    return;
  }
  for(int i = 0; i < p; ++i) {
    i2c_write(bytes[i]);
  }
  i2c_stop();
}

void readRequest() {
  Serial.print("Read ");
  while (Wire.available()) {
    int c = Wire.read();
    Serial.print(c,HEX);
    Serial.print(",");
  }
  Serial.print("\n");  
}
