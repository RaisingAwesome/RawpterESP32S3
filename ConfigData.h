#pragma once

#include "Constants.h"                       // Project constants

struct ConfigData
{
  // These are all the items that can be configured on the WebUI
  float stick_dampener = 1.0f;             // 0.1-1 Lower=slower, higher=noiser default 0.7
  float throttleLimit = 0.6f;              // can be overridden with web interface.
  float UP_COEFF = 0.01f;                                       // 0.0 - 1.0 (0 is slower up, 1 is faster up)
  float DOWN_COEFF = 0.03f;                                    // 0.0 - 1.0 (0 is slower down, 1 is faster down)
  float failsafeCoeff = .00001f;                               // 0.0001 - 0.03 (slow to fast)
  int16_t failsafeThrottlePWM = 1650;        // A safe throttle for descent.
  float B_madgwick = 0.02f;
  float trimYaw = 0;
  float trimPitch = 0;
  float trimRoll = 0;
  float maxRoll = 35.0f;  // Max roll angle in degrees the sticks can achieve
  float maxPitch = 35.0f; // Max pitch angle in degrees the sticks can achieve
  float maxYaw = 160.0f;  // Max yaw rate in deg/sec (default 160.0)
  float Gyro_filter = 0.9f;  // Lower is slower to catch up. Higher is faster to track, but can have a lot of noise. 1 is no filter
  float Accel_filter = 0.9f; // Lower is slower to catch up. Higher is faster to track, but can have a lot of noise. 1 is no filter
  bool rateControlMode = false;
  float K_pos2pwm = 50.0f;  // gain for headHome routine

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
