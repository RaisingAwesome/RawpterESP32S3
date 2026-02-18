// Types.h
//Structs and variables
#pragma once

#include "Constants.h"                     // Project constants

struct Limits
{  // Rate Limits - need to expand this to bring in the K constants and desired setpoints
  float maxRate =120; // deg/s
};

struct BatteryMonitor
{
  // Voltage Monitoring and Beeping
  int batteryVoltage = 777;             // just a default for the battery monitoring routine
  float calced_voltage = 14.8;
};

struct PID
{
// PID Controllers:
  float throttle_desired, roll_des, pitch_des, yaw_des; // Normalized desired state
  float roll_PID = 0;
  float pitch_PID = 0;
  float yaw_PID = 0;
  float desiredRateRoll, desiredRatePitch;
  unsigned long PIDCounter = 0;
};
