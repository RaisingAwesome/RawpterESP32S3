#pragma once
#include <wire.h>

const uint8_t IST8310_ADDR = 0x0E;
const uint8_t MAG_DRDY_PIN = 5;   // IO5
const uint8_t REG_DATA_X_L = 0x03; 
const uint8_t  IST8310_WHO_AM_I_REG = 0x00;
const uint8_t  IST8310_WHO_AM_I_VAL = 0x10;

struct IST8310
{ 
  int16_t x=0, y=0, z=0;
  bool hasData = false;

  // Calibration parameters
  float offX = 0, offY = 0, offZ = 0; // Hard Iron (Offsets)
  float scaleX = 1, scaleY = 1, scaleZ = 1; // Soft Iron (Scaling)

  void begin()
  {
    pinMode(MAG_DRDY_PIN, INPUT); // Set DRDY as input
    
    uint8_t chipID = readRegister(IST8310_ADDR, IST8310_WHO_AM_I_REG);

    if (chipID != IST8310_WHO_AM_I_VAL) {
      Serial.printf("IST8310 Fail! Expected 0x10, got 0x%02X\n", chipID);
      while (true) {delay(100);}
    }

    Serial.printf("IST8310 WHO_AM_I response: 0x%02X\n", chipID);
    delay(100);
    //should already be running Wire.begin(18, 8, 400000); 
    writeRegister(IST8310_ADDR, 0x04, 0x01);
    delay(50);

    // Average 16 times (0x08) or 8 times (0x04) for speed
    writeRegister(IST8310_ADDR, 0x0B, 0x08); 
    
    // Set to 10Hz Continuous Mode
    // Note: If 0x0D hangs, try 0x01 for single-shot inside the loop
    writeRegister(IST8310_ADDR, 0x0A, 0x0B); 
    Serial.println("IST8310 Initialized.");
    delay(1000);
  }

  bool update()
  { 
    // Only read if DRDY is HIGH
    if (REG_READ(GPIO_IN_REG) & (1 << MAG_DRDY_PIN)) { //fast way to check if IO5 is high
      readIST8310(x, y, z);
      return true;
    } 
    else
    {
      return false;
    } 
    // No delay needed here; digitalRead handles the timing
  }

  bool readIST8310(int16_t &x, int16_t &y, int16_t &z) {
    Wire.beginTransmission(IST8310_ADDR);
    Wire.write(REG_DATA_X_L);
    if (Wire.endTransmission() != 0) return false;

    Wire.requestFrom(IST8310_ADDR, (uint8_t)6);
    if (Wire.available() == 6) {
      uint8_t xl = Wire.read();
      uint8_t xh = Wire.read();
      uint8_t yl = Wire.read();
      uint8_t yh = Wire.read();
      uint8_t zl = Wire.read();
      uint8_t zh = Wire.read();

      x = -(int16_t)(xh << 8 | xl); // must be flipped with the minus sign to match the installed orientation
      y = (int16_t)(yh << 8 | yl);
      z = -(int16_t)(zh << 8 | zl); // must be flipped with the minus sign to match the installed orientation

      x = ((float)x - (-7.5)) * 0.96; // (x- offset) * scalex from running CALIBRATE_MAGNETOMETER = true
      y = ((float)y - (-3.0)) * 1.15;
      z = ((float)z - (-4.50)) * 0.92;

      return true;
    }
    return false;
  }
  void calibrate(uint32_t duration_ms = 45000) {
    int16_t minX = 32767, maxX = -32768;
    int16_t minY = 32767, maxY = -32768;
    int16_t minZ = 32767, maxZ = -32768;

    Serial.println("Starting Mag Calibration. Rotate device in all directions (Paint a Sphere's Inside with the nose)!");
    uint32_t startTime = millis();

    while (millis() - startTime < duration_ms) {
      if (update()) {
        if (x < minX) minX = x; if (x > maxX) maxX = x;
        if (y < minY) minY = y; if (y > maxY) maxY = y;
        if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
      }
      delay(10); 
    }

    // 1. Hard Iron Correction (Calculate offsets to center the sphere)
    offX = (maxX + minX) / 2.0f;
    offY = (maxY + minY) / 2.0f;
    offZ = (maxZ + minZ) / 2.0f;

    // 2. Soft Iron Correction (Scale axes so the sphere is round, not an oval)
    float avgDeltaX = (maxX - minX) / 2.0f;
    float avgDeltaY = (maxY - minY) / 2.0f;
    float avgDeltaZ = (maxZ - minZ) / 2.0f;
    float avgDelta = (avgDeltaX + avgDeltaY + avgDeltaZ) / 3.0f;

    scaleX = avgDelta / avgDeltaX;
    scaleY = avgDelta / avgDeltaY;
    scaleZ = avgDelta / avgDeltaZ;

    Serial.println("Calibration Complete.");
    Serial.printf("Offsets: X:%.2f Y:%.2f Z:%.2f\n", offX, offY, offZ);
    Serial.printf("Scales:  X:%.2f Y:%.2f Z:%.2f\n", scaleX, scaleY, scaleZ);
    while (true) {delay(100);}
  }
  float headingWhenFlat()
  {
    float heading = atan2((float)y, (float)x) * 180.0 / M_PI; //degrees without IMU input - assumes the drone is flat
    return heading;  
  }
  float getHeadingRadians() {
    // 1. Refresh the raw values from the sensor
    while (!update()) delay(100);

    // 2. Calculate heading in radians (-PI to PI)
    // Using atan2f for 32-bit float efficiency on ESP32-S3
    float heading = atan2f((float)y, (float)x);

    // 3. Optional: Add Magnetic Declination for Hopkinsville, KY (~ -3.8 degrees)
    // float declination = -3.8f * (M_PI / 180.0f);
    // heading += declination;

    // 4. Wrap to ensure the result stays within -PI to PI
    if (heading > M_PI) heading -= 2.0f * M_PI;
    if (heading < -M_PI) heading += 2.0f * M_PI;

    return heading;
  }

  void writeRegister(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  }
  
  // You'll need a readRegister helper if you don't have one:
  uint8_t readRegister(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
  }
};