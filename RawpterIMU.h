#pragma once
#include <SPI.h>                             // SPI Communication Support for IMU
// Yaw and magnetometer chat: https://gemini.google.com/share/d0c10d7e359e
struct RawpterIMU
{
  // Madgwick Parameters (the method that calculates angles fromt he IMU)
  SPIClass IMUSPI;   // declare only
  RawpterIMU() : IMUSPI(HSPI) {}   // construct here

  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
  float q0 = 1.0f; // Initialize quaternion for madgwick filter
  float q1 = 0.0f;
  float q2 = 0.0f;
  float q3 = 0.0f;
  unsigned long lastMadgwickUpdateMicros = 0;
  float madDeltaTime = 1 / IMU_FREQ_HZ;

  // IMU:
  float AccX, AccY, AccZ;
  float AccX_prev, AccY_prev, AccZ_prev;
  float GyroX, GyroY, GyroZ;
  float GyroX_prev, GyroY_prev, GyroZ_prev;
  float roll_IMU, pitch_IMU, yaw_IMU;
  float roll_IMU_prev, pitch_IMU_prev;

  // GPS Flight Controller
  
  float AccErrorX = -0.01;
  float AccErrorY = -0.01;
  float AccErrorZ = 0.00;
  float GyroErrorX = 0.54;
  float GyroErrorY = -0.73;
  float GyroErrorZ = 0.36;
  
  // No-GPS Flight Controller - big boy
/*
  float AccErrorX = 0.03;
  float AccErrorY = 0.01;
  float AccErrorZ = -0.01;
  float GyroErrorX = -1.21;
  float GyroErrorY = 0.42;
  float GyroErrorZ = 0.77;
  */
  
  void getIMUdata(ConfigData &configData)
  {
    // Fast GPIO read (ESP32 example)
    if (gpio_get_level((gpio_num_t)IMU_INT_PIN))
      return; // active LOW

    // Burst read: 6 accel + 6 gyro bytes in one transaction
    uint8_t buf[12];
    readBlock(ACCEL_DATA_X1, buf, 12);

    int16_t rawAx = (buf[0] << 8) | buf[1];
    int16_t rawAy = (buf[2] << 8) | buf[3];
    int16_t rawAz = (buf[4] << 8) | buf[5];

    int16_t rawGx = (buf[6] << 8) | buf[7];
    int16_t rawGy = (buf[8] << 8) | buf[9];
    int16_t rawGz = (buf[10] << 8) | buf[11];

    // Apply offsets + scaling
    float ax = (rawAx - AccErrorX) * G_PER_LSB;
    float ay = (rawAy - AccErrorY) * G_PER_LSB;
    float az = (rawAz - AccErrorZ) * G_PER_LSB;

    float gx = (rawGx - GyroErrorX) * DPS_PER_LSB;
    float gy = (rawGy - GyroErrorY) * DPS_PER_LSB;
    float gz = (rawGz - GyroErrorZ) * DPS_PER_LSB;

    // Filter (optimized form)
    AccX = AccX_prev + configData.Accel_filter * (ax - AccX_prev);
    AccY = AccY_prev + configData.Accel_filter * (ay - AccY_prev);
    AccZ = AccZ_prev + configData.Accel_filter * (az - AccZ_prev);

    GyroX = GyroX_prev + configData.Gyro_filter * (gx - GyroX_prev);
    GyroY = GyroY_prev + configData.Gyro_filter * (gy - GyroY_prev);
    GyroZ = GyroZ_prev + configData.Gyro_filter * (gz - GyroZ_prev);

    AccX_prev = AccX;
    AccY_prev = AccY;
    AccZ_prev = AccZ;
    GyroX_prev = GyroX;
    GyroY_prev = GyroY;
    GyroZ_prev = GyroZ;

    // Timing
    unsigned long currentMicros = micros();
    unsigned long tempMicros = (currentMicros - lastMadgwickUpdateMicros);
    //if (tempMicros>=0 && tempMicros<1000) madDeltaTime = tempMicros * USEC_TO_SEC; else madDeltaTime=0.0005f;
    madDeltaTime = tempMicros * USEC_TO_SEC;
    lastMadgwickUpdateMicros = currentMicros;

    // Madgwick update
    // The IMU’s Z‑axis physically points upward, but our control system uses the NED frame where Z points downward. 
    // NED (North-East-Down) is standard aerospace frame.
    // X --> forward positive
    // Y --> right positive
    // Z --> down positive
    // The Rawpter is installed like this:
    // IMU X --> Back of drone
    // IMU Y --> Left of drone
    // IMU Z --> Up to the sky
    // So, to map it to the IMU's physical installation in the frame, for a Z now being roll up when compared to NED,
    // and X pointing backwards, Y will be positive in the oppositve direction. Based on how the board is installed, 
    // X is actually pointing to the back of the drone, so its AccX needs flipped. Thus, you call Madgwick like this:
    Madgwick6DOF(configData.B_madgwick, GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, madDeltaTime);
  }

  void Madgwick6DOF(float B_madgwick, float gx, float gy, float gz, float ax, float ay, float az, float dt)
  {
    // Precomputed constants
    constexpr float DEG2RAD = 0.01745329252f; // π/180
    constexpr float RAD2DEG = 57.29577951f;   // 180/π

    // Convert gyro to rad/s
    gx *= DEG2RAD;
    gy *= DEG2RAD;
    gz *= DEG2RAD;

    // Rate of change of quaternion
    const float half = 0.5f;
    qDot1 = half * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = half * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = half * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = half * (q0 * gz + q1 * gy - q2 * gx);

    // Accelerometer valid?
    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f))
    {
      // Normalize accelerometer
      float norm = ax * ax + ay * ay + az * az;
      float recipNorm = 1.0f / sqrtf(norm);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      // Precompute reused terms
      float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
      float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
      float _4q0 = 2.0f * _2q0, _4q1 = 2.0f * _2q1, _4q2 = 2.0f * _2q2;
      float _8q1 = 2.0f * _4q1, _8q2 = 2.0f * _4q2;

      // Gradient descent step
      float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

      recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;

      // Apply feedback
      qDot1 -= B_madgwick * s0;
      qDot2 -= B_madgwick * s1;
      qDot3 -= B_madgwick * s2;
      qDot4 -= B_madgwick * s3;
    }

    // Integrate quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalize quaternion
    float norm = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    recipNorm = 1.0f / sqrtf(norm);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Compute angles (cache reused terms)
    float twoq0q1 = 2.0f * (q0 * q1);
    float twoq2q3 = 2.0f * (q2 * q3);
    float twoq0q2 = 2.0f * (q0 * q2);
    float twoq1q3 = 2.0f * (q1 * q3);
    float twoq0q3 = 2.0f * (q0 * q3);
    float twoq1q2 = 2.0f * (q1 * q2);

    roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
    pitch_IMU = -asin(-2.0f * (q1*q3 - q0*q2))*57.29577951; //degrees
    yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
  }

  void MadgwickInit()
  {
    // Initialize Madgwick filter with current accelerometer values
    // Cuts down on thhe time it takes for Madgwick to converge on a stable attitude.
    while (gpio_get_level((gpio_num_t)IMU_INT_PIN))
      delay(10); // Wait for it to pull low to indicate data ready

    // Burst read: 6 accel + 6 gyro bytes in one transaction
    uint8_t buf[12];
    readBlock(ACCEL_DATA_X1, buf, 12);

    int16_t rawAx = (buf[0] << 8) | buf[1];
    int16_t rawAy = (buf[2] << 8) | buf[3];
    int16_t rawAz = (buf[4] << 8) | buf[5];

    int16_t rawGx = (buf[6] << 8) | buf[7];
    int16_t rawGy = (buf[8] << 8) | buf[9];
    int16_t rawGz = (buf[10] << 8) | buf[11];

    // Apply offsets + scaling
    float ax = (rawAx - AccErrorX) * G_PER_LSB;
    float ay = (rawAy - AccErrorY) * G_PER_LSB;
    float az = (rawAz - AccErrorZ) * G_PER_LSB;

    float norm = sqrtf(ax * ax + ay * ay + az * az);

    if (norm == 0.0f)
      return; // invalid accel
    ax /= norm;
    ay /= norm;
    az /= norm;

    // Compute roll and pitch from gravity
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    float yaw = 0.0f; // no magnetometer, assume heading = 0

    // Convert to quaternion
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    q0 = cr * cp * cy + sr * sp * sy;
    q1 = sr * cp * cy - cr * sp * sy;
    q2 = cr * sp * cy + sr * cp * sy;
    q3 = cr * cp * sy - sr * sp * cy;

    // Normalize quaternion
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;

    // Cache Euler angles for consistency
    roll_IMU = roll * 57.29577951f; // RAD2DEG
    pitch_IMU = -pitch * 57.29577951f; // Flip pitch axis to match NASA
    yaw_IMU = -yaw * 57.29577951f; // Flip yaw axis to match NASA
  }

  void begin()
    {
    // DESCRIPTION: Initialize IMU and set to 2000Hz gyro and 2000Hz accel output rates.
    pinMode(IMU_INT_PIN, INPUT);
    pinMode(SPI_CS, OUTPUT);
    gpio_set_level((gpio_num_t)SPI_CS, 1);              // Deselect device
    IMUSPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI, SPI_CS); // SCLK, MISO, MOSI, CS
    delay(100);
    writeRegister(0x76, 0x0);            // Work with Bank 0
    writeRegister(0x11, 0x01);           // Soft Reset
    delay(200);                          // give time for reset
    uint8_t whoami = readRegister(0x75); // WHO_AM_I register should return 0x3B
    delay(100);
    Serial.print("WHO_AM_I: 0x");
    Serial.println(whoami, HEX);
    if (whoami == 0x3B)
    {
        Serial.println("IMU SPI communication successful.");
    }
    else
    {
        Serial.println("IMU SPI communication failed. Check wiring, CS pin, or SPI settings. Trying again.");
        while (true) delay(10);
        return;
    }

    writeRegister(0x4E, 0x1F);
    delay(10); // Turn the accelerator and gyro in Low Noise mode and power the temperature sensor.
    writeRegister(0x13, 0x05);
    delay(10); // <2ns SPI Slew Rate
    writeRegister(0x16, 0x00);
    delay(10); // Bypass FIFO (don't queue readings)
    writeRegister(0x4F, 0x26);
    delay(10); // Gyro 1000 dps at 1kHz updates downsampled from 32kHz 00100110 = 0x26
    writeRegister(0x50, 0x26);
    delay(10); // Accel 16g and 1Khz updates 00100110 = 0x26
    writeRegister(0x65, 0x08);
    delay(10); // Don't bother interupting after reseet. Interrupt on 1 when data is available.
    writeRegister(0x76, 0x01);
    delay(10); // Work with Bank 1
    writeRegister(0x03, 0x00);
    delay(10); // Bank 1 register: Use every axis measurement including Z (default has it off)
    writeRegister(0x7A, 0x02);
    delay(10); // Bank 1 register: 4 wire SPI mode
    writeRegister(0x76, 0x0);
    delay(10); // Work with Bank 0

    MadgwickInit(); // Initialize Madgwick filter with current accelerometer values
    Serial.println("IMU config complete.");
}

uint8_t readRegister(uint8_t reg)
{
  IMUSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // 1 MHz, Mode 0
  gpio_set_level((gpio_num_t)SPI_CS, 0);
  IMUSPI.transfer(reg | 0x80); // Read bit set
  uint8_t value = IMUSPI.transfer(0x00);
  gpio_set_level((gpio_num_t)SPI_CS, 1);
  IMUSPI.endTransaction();
  return value;
}

int16_t read16(uint8_t reg)
{
  IMUSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // 1 MHz, Mode 0
  gpio_set_level((gpio_num_t)SPI_CS, 0);
  IMUSPI.transfer(reg | 0x80); // Read bit set
  uint8_t high = IMUSPI.transfer(0x00);
  uint8_t low = IMUSPI.transfer(0x00);
  gpio_set_level((gpio_num_t)SPI_CS, 1);
  IMUSPI.endTransaction();
  return (int16_t)((high << 8) | low);
}

inline void readBlock(uint8_t reg, uint8_t *buf, size_t len)
{
  IMUSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // 1 MHz, Mode 0
  // Assert chip select
  gpio_set_level((gpio_num_t)SPI_CS, 0);

  // Send register address with MSB=1 to indicate read
  IMUSPI.transfer(reg | 0x80);

  // Read 'len' bytes
  for (size_t i = 0; i < len; i++)
  {
    buf[i] = IMUSPI.transfer(0x00); // dummy write, read response
  }

  // Deassert chip select
  gpio_set_level((gpio_num_t)SPI_CS, 1);
  IMUSPI.endTransaction();
}

void writeRegister(uint8_t reg, uint8_t value)
{
  IMUSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // 1 MHz, Mode 0
  gpio_set_level((gpio_num_t)SPI_CS, 0);
  IMUSPI.transfer(reg & 0x7F); // Write bit
  IMUSPI.transfer(value);
  gpio_set_level((gpio_num_t)SPI_CS, 1);
  IMUSPI.endTransaction();
}
  
void calculateIMUError()
  {
    Serial.println("Calculating IMU Error with 30000 filtered iterations. Please stand by...");

    // Filtering parameters
    constexpr float ALPHA = 0.05f;                 // Low‑pass filter coefficient (0.0–1.0)
    constexpr float OUTLIER_THRESHOLD_ACC = 0.25f; // g
    constexpr float OUTLIER_THRESHOLD_GYRO = 5.0f; // deg/s

    float fAccX = 0, fAccY = 0, fAccZ = 0;
    float fGyroX = 0, fGyroY = 0, fGyroZ = 0;

    AccErrorX = AccErrorY = AccErrorZ = 0;
    GyroErrorX = GyroErrorY = GyroErrorZ = 0;

    int c = 0;
    while (c < 30000)
    {
      while (digitalRead(IMU_INT_PIN) == HIGH)
        delay(1);

      float rawAccX = read16(ACCEL_DATA_X1) * G_PER_LSB;
      float rawAccY = read16(ACCEL_DATA_X1 + 2) * G_PER_LSB;
      float rawAccZ = read16(ACCEL_DATA_X1 + 4) * G_PER_LSB;

      float rawGyroX = read16(GYRO_DATA_X1) * DPS_PER_LSB;
      float rawGyroY = read16(GYRO_DATA_X1 + 2) * DPS_PER_LSB;
      float rawGyroZ = read16(GYRO_DATA_X1 + 4) * DPS_PER_LSB;
      if (c == 0)
      {
        fAccX = rawAccX;
        fAccY = rawAccY;
        fAccZ = rawAccZ;
        fGyroX = rawGyroX;
        fGyroY = rawGyroY;
        fGyroZ = rawGyroZ;
      }
      else
      {
        // -------- OUTLIER REJECTION --------
        if (fabs(rawAccX - fAccX) < OUTLIER_THRESHOLD_ACC)
          fAccX = fAccX + ALPHA * (rawAccX - fAccX);
        if (fabs(rawAccY - fAccY) < OUTLIER_THRESHOLD_ACC)
          fAccY = fAccY + ALPHA * (rawAccY - fAccY);
        if (fabs(rawAccZ - fAccZ) < OUTLIER_THRESHOLD_ACC)
          fAccZ = fAccZ + ALPHA * (rawAccZ - fAccZ);

        if (fabs(rawGyroX - fGyroX) < OUTLIER_THRESHOLD_GYRO)
          fGyroX = fGyroX + ALPHA * (rawGyroX - fGyroX);
        if (fabs(rawGyroY - fGyroY) < OUTLIER_THRESHOLD_GYRO)
          fGyroY = fGyroY + ALPHA * (rawGyroY - fGyroY);
        if (fabs(rawGyroZ - fGyroZ) < OUTLIER_THRESHOLD_GYRO)
          fGyroZ = fGyroZ + ALPHA * (rawGyroZ - fGyroZ);
      }
      // -------- ACCUMULATE FILTERED VALUES --------
      AccErrorX += fAccX;
      AccErrorY += fAccY;
      AccErrorZ += fAccZ;

      GyroErrorX += fGyroX;
      GyroErrorY += fGyroY;
      GyroErrorZ += fGyroZ;

      c++;

      if (c % 500 == 0)
        Serial.println(String(30000 - c));
    }

    // -------- FINAL AVERAGE --------
    AccErrorX /= c;
    AccErrorY /= c;
    AccErrorZ = (AccErrorZ / c) - 1.0f; // subtract gravity

    GyroErrorX /= c;
    GyroErrorY /= c;
    GyroErrorZ /= c;

    Serial.print("float AccErrorX = ");
    Serial.print(AccErrorX);
    Serial.println(";");
    Serial.print("float AccErrorY = ");
    Serial.print(AccErrorY);
    Serial.println(";");
    Serial.print("float AccErrorZ = ");
    Serial.print(AccErrorZ);
    Serial.println(";");

    Serial.print("float GyroErrorX = ");
    Serial.print(GyroErrorX);
    Serial.println(";");
    Serial.print("float GyroErrorY = ");
    Serial.print(GyroErrorY);
    Serial.println(";");
    Serial.print("float GyroErrorZ = ");
    Serial.print(GyroErrorZ);
    Serial.println(";");

    Serial.println("Paste these values in user specified variables section and comment out calculateIMUError() in void setup.");
    while (true) delay(10);
  }  
};