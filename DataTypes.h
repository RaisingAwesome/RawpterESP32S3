// DataTypes.h
// Object oriented structs and variables
#pragma once

#include <SparkFun_BMP581_Arduino_Library.h> // BMP581 Pressure Sensor
#include <SparkFun_u-blox_GNSS_v3.h>         // Max10S GPS

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
  TaskHandle_t bmpTaskHandle= nullptr;                     // Handle for the BMP581 task that will run on its own core.
  bool hasBMP581 = false;
  float sessionHoverPWM =1600;
  float targetRateLanding = -0.1f; // ft/sec
  float upGain = 4.0f; // How hared to push back up if descend too quickly on landing
  float fastestAscent = 0.0f; // ft/sec
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
};

struct HomePosition
{
    bool valid = false;
    double lat_deg = 0;
    double lon_deg = 0;
};

struct Limits
{  // Rate Limits - need to expand this to bring in the K constants and desired setpoints
  float maxRate =120; // deg/s
};

struct PIDConstants
{
  // PID parameters (this is where you set the.  It's best to use the WiFi interface to do it live and then update once its tuned.):
  float i_limit_angle = 0.5f;      // Integrator saturation level
  float i_limit_rate = 4.0f; // Integrator saturation level

  float Kp_roll_angle = 3.0f;    // Roll P-gain
  float Ki_roll_angle = 0.001f;  // Roll I-gain

  float Kp_pitch_angle = 3.0f;   // Pitch P-gain
  float Ki_pitch_angle = 0.001f; // Pitch I-gain

  float Kp_roll_rate = 0.0015f;  // Roll P-gain
  float Ki_roll_rate = 0.003f;   // Roll I-gain
  float Kd_roll_rate = 0.0f;     // Roll D-gain

  float Kp_pitch_rate = 0.0015f; // Pitch P-gain
  float Ki_pitch_rate = 0.003f;  // Pitch I-gain
  float Kd_pitch_rate = 0.0f;    // Pitch D-gain

  float Kp_yaw_rate = 0.0015f;   // Yaw P-gain default 30
  float Ki_yaw_rate = 0.003f;    // Yaw I-gain default 5
  float Kd_yaw_rate = 0.0f;      // Yaw D-gain default .015 (be careful when increasing too high, motors will begin to overheat!)
};


