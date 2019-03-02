/* TA1276AN I2C Bridge
 * (2019) Martin Hejnfelt (martin@hejnfelt.com)
 *
 * For use with monitors that use the TA1276AN jungle IC.
 * This changes the I2C communication from monitor MCUs
 * that doesn't treat RGB like it should, to something
 * that does. Basically it ensures that RGB Brightness
 * and RGB Contrast are in sync with the "other"
 * brightness and contrast values, plus kills the "HI BRT"
 * bit which otherwise causes a somewhat dull picture (or
 * close to black image when 75 Ohm terminations resistors
 * are used (which they should).
 * Connect the main I2C line of the monitors PCB to the 
 * hardware I2C (TWI) pins and then specify the desired
 * secondary I2C pins for the communication to the jungle.
 * A speed Arduino is recommended, for now only tested on
 * a 20MHz atmega2560 clone plus 16MHz Nano v3.0 clone.
 * 
 * Requires the SoftI2CMaster library
 * https://github.com/felias-fogg/SoftI2CMaster
 * 
 * Monitors tested:
 * JVC DT-V100CG (initial version)
 * JVC TM-H1375SU (version 1.1)
 */

// Version 1.1

#define I2C_TIMEOUT 1000
#define I2C_PULLUP 1

// These work for a Mega2560
/*
#define SDA_PORT PORTA
#define SDA_PIN 0 // = 22
#define SCL_PORT PORTA
#define SCL_PIN 2 // = 24
*/

// These work for Nano 3.0
#define SDA_PORT PORTD
#define SDA_PIN 4 // = PD4
#define SCL_PORT PORTD
#define SCL_PIN 5 // = PD5

#include <SoftI2CMaster.h>
#include <Wire.h>
// Address in the datasheet is said to be 0x88 for write
// and 0x89 for read. That is somewhat of a "mistake" as
// i2c uses 7 bit addressing and the least significant bit
// is read (1) or write (0). Thus the address is shifted once
// to the right to get the "real" address which is then 44h/68
#define TA1276AN_ADDR (68)
#define SER_DEBUG_WRITE (1)
//#define SER_DEBUG_READ (0)

#define REG_UNICOLOR (0x00)
#define REG_BRIGHTNESS (0x01)
#define REG_RGB_BRIGHTNESS (0x05)
#define REG_RGB_CONTRAST (0x06)

void setup() {
  bool iicinit = i2c_init();
  Serial.begin(115200);
  Serial.print("TA1276AN I2C Bridge\n");
  Wire.begin(TA1276AN_ADDR);
  Wire.onReceive(writeRequest);
  Wire.onRequest(readRequest);
  if(!iicinit) Serial.println("I2C init failed");
}

void loop() {
  delay(10);
}

void writeRegister(const uint8_t reg, const uint8_t val) {
  i2c_start((TA1276AN_ADDR<<1)|I2C_WRITE);
  i2c_write(reg);
  i2c_write(val);
  i2c_stop(); 
#ifdef SER_DEBUG_WRITE
  Serial.print("Write register: ");
  Serial.print(reg,HEX);
  Serial.print(" value: ");
  Serial.print(val,HEX);
  Serial.print("\n");
#endif
}

uint8_t brightness = 0x80;
uint8_t contrast = 0x80;

void writeRequest(int byteCount) {
  if(byteCount > 2) {
    uint8_t reg = Wire.read();
    uint8_t bc = 1;
    Serial.print("Bulk write: start reg: ");
    Serial.print(reg,HEX);
    Serial.print(" ");
    i2c_start((TA1276AN_ADDR<<1)|I2C_WRITE);
    i2c_write(reg);
    do {
      uint8_t val = Wire.read();
      Serial.print(val,HEX);
      switch(reg) {
        case REG_RGB_BRIGHTNESS:
          {
            uint8_t v = (brightness>>1)<<1; 
            i2c_write(v);
            Serial.print("(");
            Serial.print(v,HEX);
            Serial.print(")");
          }
        break;
        case REG_RGB_CONTRAST:
          {
            uint8_t v = contrast & 0x7F;
            i2c_write(v);
            Serial.print("(");
            Serial.print(v,HEX);
            Serial.print(")");
          }
        break;
        case REG_BRIGHTNESS:
          brightness = val;
          i2c_write(val);
          break;
        case REG_UNICOLOR:
          contrast = val;
          i2c_write(val);          
        break;
        default:
          i2c_write(val);
        break;
      }
      ++bc;
      ++reg;
    } while(bc < byteCount);
    i2c_stop();
    Serial.print("\n");
  } else {
    uint8_t reg = Wire.read();
    uint8_t val = Wire.read();
    switch(reg) {
      case REG_RGB_BRIGHTNESS:
      case REG_RGB_CONTRAST:
        // Do nothing
      break;
      case REG_BRIGHTNESS:
        writeRegister(reg,val);
        // Write the same value to RGB Brightness register
        writeRegister(REG_RGB_BRIGHTNESS,((val>>1)<<1));
      break;
      case REG_UNICOLOR:
        writeRegister(reg,val);
        // Write the same value to RGB Contrast register
        // But make sure to kill most significant bit
        // otherwise picture gets dark (at least for me)
        writeRegister(REG_RGB_CONTRAST,val & 0x7F);
      break;
      default:
        writeRegister(reg,val);
      break;
    }
  }
}

void readRequest() {
  // Read requests to TA1276AN returns two bytes
  // So we read those and replies. One would think
  // this could give timing errors, but at least
  // on atmega2560 @ 20MHz it doesn't
  uint8_t r[2];
  i2c_start((TA1276AN_ADDR<<1)|I2C_READ);
  r[0] = i2c_read(false); // read one byte
  r[1] = i2c_read(true); // read one byte and send NAK to terminate
  i2c_stop(); // send stop condition
  Wire.write(r,2);
#ifdef SER_DEBUG_READ
  Serial.print("Read ");
  Serial.print(r[0],HEX);
  Serial.print(",");
  Serial.print(r[1],HEX);
  Serial.print("\n");
#endif
}
