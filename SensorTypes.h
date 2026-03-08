#pragma once

#include <SparkFun_BMP581_Arduino_Library.h> // BMP581 Pressure Sensor
#include <SparkFun_u-blox_GNSS_v3.h>         // Max10S GPS
#include "driver/gpio.h"                     // ESP32-S3 routines to speed up digitalWrite

#include "Constants.h"                       // Project constants
#include "ConfigData.h"

struct AltitudeData
{
  BMP581 pressureSensor;
  uint8_t i2cAddress = 0x46;      // 0x46 is the Pressure Sensor 0x42 is the GPS
  volatile float altitude = 0.0f; // just a default until it starts to loop.
  float highestAltitude = 0.0f;
  float ceiling = 12.0f;
  volatile float rateFPS = 0.0f;         // Used to ensure it doesn't descend too quickly
  volatile float rateFPSWorking = 0.0f;  // This one is worked by the pressure task and then synced to rateFPS variable in the main loop.
  volatile float altitudeWorking = 0.0f; // This one is worked by the pressure task and then synced to altitude variable in the main loop.
  float maxAltitudeChangeRate = .8f;
  float kp_altitude_rate = .0001f;
  float ki_altitude_rate = .0001f;
  volatile float invGroundPressure = 0.0f;        // reciprocal of baseline pressure at startup
  volatile float invGroundPressureWorking = 0.0f; // This one is worked by the pressure task and then synced to invGroundPressure variable in the main loop.
  TaskHandle_t sensorTaskHandle= nullptr;                     // Handle for the BMP581 task that will run on its own core.
  bool hasBMP581 = false;
  float sessionHoverPWM =1600;
  float targetRateLanding = -0.1f; // ft/sec
  float upGain = 4.0f; // How hared to push back up if descend too quickly on landing
  float fastestAscent = 0.0f; // ft/sec

  void begin(unsigned long failSafeThrottlePWM)
  {
    Serial.println("Setting up BMP581...");
    sessionHoverPWM = failSafeThrottlePWM; 
    if (pressureSensor.beginI2C(i2cAddress) != BMP5_OK)
    {
      Serial.println("BMP581 not found!");
      hasBMP581 = false;
      return;
    }
    Serial.println("BMP581 discovered at address 0x46");

    int8_t err = BMP5_OK;

    // Set ODR directly
    err = pressureSensor.setODRFrequency(BMP5_ODR_100_2_HZ);
    if (err != BMP5_OK)
    {
      Serial.print("Error setting ODR! Error code: ");
      Serial.println(err);
      hasBMP581=false;
      return;
    }
    else
    {
      Serial.println("BMP581 ODR set to 100 Hz (pressure only)");
    }
    bmp5_iir_config config;

    // Configure IIR filters
    config.set_iir_t = BMP5_IIR_FILTER_COEFF_1; // Minimal temp filtering
    config.set_iir_p = BMP5_IIR_FILTER_COEFF_3; // 7 caused too much lag.
    config.shdw_set_iir_t = BMP5_DISABLE;       // Don’t store temp data
    config.shdw_set_iir_p = BMP5_ENABLE;        // Store filtered pressure
    config.iir_flush_forced_en = BMP5_DISABLE;  // No forced flush

    err = pressureSensor.setFilterConfig(&config);
    vTaskDelay(100);
    if (err != BMP5_OK)
    {
      Serial.print("Error setting filter config! Error code: ");
      Serial.println(err);
      hasBMP581 = false;
      return;
    }
    else
    {
      Serial.println("BMP581 Filter configuration applied successfully.");
    }
    hasBMP581 = true;
  }
};

// GPS
struct GPSData
{
  bool hasGPS = false;
  bool useGPS = false;
  SFE_UBLOX_GNSS Max10SGPS;
  double longitude = 0;
  double latitude = 0;
  double longitudeWorking = 0;
  double latitudeWorking = 0;
  uint8_t fixType = 0;
  bool atHome = false;
  bool homeValid = false;
  double homePosLatitude = 0;
  double homePosLongitude = 0;

  void begin() {
    delay(100);
    if (!Max10SGPS.begin(Wire))
    {
      Serial.println("u-blox GNSS not detected over I2C.");
      hasGPS = false;
      return;
    }

    hasGPS = true;
    Serial.println("U-Blox GPS discovered.");
    Serial.print("Firmware Version: ");
    Serial.println(Max10SGPS.getProtocolVersionHigh());
    Max10SGPS.setI2COutput(COM_TYPE_UBX);
    Max10SGPS.setNavigationFrequency(1);
    Max10SGPS.setAutoPVT(true);

    Serial.println("TIMEPULSE configured: 1 Hz, 50% duty, always on.");

    Max10SGPS.newCfgValset(VAL_LAYER_RAM); 

    // 1. Enable the pulse (Using your exact key)
    Max10SGPS.addCfgValset(0x10050007, 1); // CFG-TP-TP1_ENA

    // 2. Set Pulse Definition to Period (0)
    Max10SGPS.addCfgValset(0x20050023, 0); // CFG-TP-PULSE_DEF

    // 3. Set Length Definition to Length [us] (0)
    Max10SGPS.addCfgValset(0x20050030, 0); // CFG-TP-PULSE_LENGTH_DEF

    // 4. Set Period to 1s (1,000,000 us)
    Max10SGPS.addCfgValset(0x40050002, 1000000); // CFG-TP-PERIOD_TP1

    // 5. Set Length to 0.5s (500,000 us)
    Max10SGPS.addCfgValset(0x40050004, 500000);  // CFG-TP-LEN_TP1

    // 6. Don't wait for GNSS Lock (Set to False/0)
    // This is key ID 0x10050009 from your list
    Max10SGPS.addCfgValset(0x10050009, 0); // CFG-TP-USE_LOCKED_TP1

    // 7. Polarity High (1)
    Max10SGPS.addCfgValset(0x1005000b, 1); // CFG-TP-POL_TP1

    if (Max10SGPS.sendCfgValset()) {
      Serial.println("Config sent. Searching for satellites...");
    } else {
      Serial.println("Problem: GPS Module could not be configured!");
    }
  }
  void setHome()
  {
    if (hasGPS && fixType >= 3)
    {
        homePosLatitude = latitude;
        homePosLongitude = longitude;
        homeValid = true;
    } else {
        homeValid = false;
    }
  }
};