// RadioData.h
// Radio Stick Variables
#pragma once

#include "RmtPPMReader.h"                    // Raising Awesome's PPM pulse capture code
#include "Constants.h"                       // Project constants
#include "Types.h"                           // General Structs

struct RadioData
{
  // Radio communication:
  int16_t PWM_throttle, PWM_roll, PWM_pitch, PWM_yaw, PWM_ThrottleCutSwitch, PWM_FailsafeSwitch;
  int16_t PWM_throttle_prev, PWM_roll_prev, PWM_pitch_prev, PWM_yaw_prev;
  bool failsafed = false;
  bool throttle_is_cut = true; // used to force the pilot to manually set the throttle to zero after the switch is used to throttle cut
  RmtPPMReader radio;

  int16_t getRadioPWM(int ch_num, int16_t defaultVal)
  {
    // DESCRIPTION: Get current radio commands from interrupt routine

    int16_t valPWM = 0;
    valPWM = latestValidChannelValue(ch_num, defaultVal); // 1000-2000 based on PPM routine return range. Return defaultVal if it isn't valid.
    return valPWM;
  }

  inline int16_t latestValidChannelValue(uint8_t  channel, int16_t defaultValue) {
    // Capture current time once
    static unsigned minChannelPWMValue = 900;       // Used to determine if a channels reported pulse width is realistically valid.
    static unsigned maxChannelPWMValue = 2100;      // Used to determine if a channels reported pulse width is realistically valid.
    static unsigned long failsafeTimeout = 500000L; // The timeout (microseconds) after which the channels which were not updated are considered invalid
    static bool failsafeTriggered = false;
    static unsigned long failsafeTime = 0;
    unsigned long currentMicros = micros();

    int16_t value = defaultValue;
    bool invalid = true;
    
    // Valid channel and within timeout?
    if (channel >= 1 && channel <= CHANNELS) {    
      uint32_t raw = radio.getChannel(channel - 1);
      if (raw >= minChannelPWMValue && raw <= maxChannelPWMValue) {
        value = (int16_t) raw;
        invalid = false;
        failsafeTriggered = false;
        failsafed = false;
        return value;
      }
    } else return 0;

    if (invalid) {
      if (!failsafeTriggered) {
        failsafeTriggered = true;
        failsafeTime = currentMicros;
      } else {
        // Elapsed time since failsafe triggered
        if (currentMicros - failsafeTime > 500000UL) {  // 0.5 seconds
          if (!failsafed) {
            setToFailsafe();
            failsafed = true;
          }
        }
      }
    } else {
      failsafeTriggered = false;
      failsafed = false;
    }

    return value;
  }

  inline void getRadioStickValues(AltitudeData &altitudeData, bool flying)
  {
    static int throttleOverrideCounter = 0;
    static bool landing = false;
    static float integralAltitudeRate = 0 ;
    static unsigned long landingTicks = 0;

    // Read radio PWM
    PWM_throttle = getRadioPWM(throttlePin, 1000);
    PWM_roll = getRadioPWM(rollPin, 1500);
    PWM_pitch = getRadioPWM(pitchPin, 1500);
    PWM_yaw = getRadioPWM(yawPin, 1500);
    PWM_ThrottleCutSwitch = getRadioPWM(throttleCutSwitchPin, 2000);
    PWM_FailsafeSwitch = getRadioPWM(SwitchB, 1000);
    
    // Radio Receiver or Switch B is in Failsafe Mode - Land safely
    // Also, only allow killMotors if flying to prevent a ground accident when worm burning.
    
    if (altitudeData.hasBMP581) 
    {
      if (PWM_FailsafeSwitch < 1900 /*landing*/ && flying)
      {   
          if (!landing)
          {
            landingTicks = 0.0f;
            integralAltitudeRate = 0.0f; // reset integrator on landing start
            throttleOverrideCounter = 0;
            landing = true;
            gps.atHome = false; // reset atHome flag to false to ensure heading home.
          }
          

          if (landingTicks++ > 5000 || altitudeData.altitude < 1.5f) // 1000 ticks per second - if we have been landing for more than 5 seconds, kill it.
          {
            killMotors(); // if we are in this mode 5 seconds, then kill motors to prevent uncontrolled flyaway.
            landing=false;
            return;
          }
          else
          {
              headHome(); // this will override pitch and roll PWM to head home if a GPS exists.
              
              if (++throttleOverrideCounter >= INNER_LOOP_FREQUENCY/ALT_FREQ_HZ) // Only make a move if we are at the altitude control frequency
              {
                  throttleOverrideCounter = 0;
                  float landingRate = altitudeData.targetRateLanding;
                  if (gps.useGPS && !gps.atHome) // Once this is working, change this such that it will go do to a low altitude like 12ft while it heads back.
                  {
                      landingRate = 0.0f;
                  }
                  float rateError  = landingRate - altitudeData.rateFPS;
                  if (rateError > 0.5f && integralAltitudeRate < 0.0f)
                  {   // if going down too fast and the integrator has been decreasing PWM, reset it to zero.
                      integralAltitudeRate = 0.0f;
                  }
                  else if (rateError < 1.0f && integralAltitudeRate > 0.0f)
                  {   // if going up too fast and the integrator has been increasing PWM, reset it to zero.
                      integralAltitudeRate = 0.0f;
                  }
                  // Integrator
                  integralAltitudeRate += rateError * 0.01f;  // 100 Hz is ticks at .01 seconds

                  // Don't accululate beyond PWM limits so you don't over saturate the integral.
                  float lowend  = -(altitudeData.sessionHoverPWM-1500) / altitudeData.ki_altitude_rate;
                  float highend =  (2000 - altitudeData.sessionHoverPWM) / (altitudeData.ki_altitude_rate);
                  integralAltitudeRate = constrain(integralAltitudeRate, lowend, highend);

                  // Asymmetric gain: more aggressive when descent is too fast (rateError > 0)
                  float upGain = (rateError > 0.0f) ? altitudeData.upGain : 1.0f;

                  float pwm = altitudeData.sessionHoverPWM
                            + upGain * altitudeData.kp_altitude_rate * rateError
                            + altitudeData.ki_altitude_rate * integralAltitudeRate;

                  PWM_throttle = constrain(pwm, 1500, 2000);
              }
              else
              {
                  PWM_throttle = PWM_throttle_prev;
              }
          }
      }
      else 
      {
        throttleOverrideCounter = 0;
        landing = false;
      }
    }
    else
    { // No altitude data, so just set throttle to minimum to prevent flyaways.
      if (PWM_FailsafeSwitch < 1900) PWM_throttle = 1500; 
    }

    if (PWM_throttle > telemetry.highestThrottlePWM)
      telemetry.highestThrottlePWM = PWM_throttle;
    else if (PWM_throttle < telemetry.lowestThrottlePWM)
      telemetry.lowestThrottlePWM = PWM_throttle;

    // Smoothing
    float coeff = (PWM_throttle > PWM_throttle_prev) ? configData.UP_COEFF : configData.DOWN_COEFF;
    if (PWM_FailsafeSwitch < 1900) 
    {
      if (altitudeData.hasBMP581) 
        coeff = 1.0f;
      else
        coeff = configData.failsafeCoeff; // Override the stick dampening to not mess up the PID loop
    }
    PWM_throttle = PWM_throttle_prev + coeff * (PWM_throttle - PWM_throttle_prev);
    
    // Bottom limit
    if (PWM_throttle < 1500)
      PWM_throttle = 1500;

    // Stick dampening (optimized form)
    PWM_roll = PWM_roll_prev + configData.stick_dampener * (PWM_roll - PWM_roll_prev);
    PWM_pitch = PWM_pitch_prev + configData.stick_dampener * (PWM_pitch - PWM_pitch_prev);
    PWM_yaw = PWM_yaw_prev + configData.stick_dampener * (PWM_yaw - PWM_yaw_prev);

    // Update prevs
    PWM_throttle_prev = PWM_throttle;
    PWM_roll_prev = PWM_roll;
    PWM_pitch_prev = PWM_pitch;
    PWM_yaw_prev = PWM_yaw;
  }
};
