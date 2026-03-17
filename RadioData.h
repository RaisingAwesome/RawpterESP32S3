// RadioData.h
// Radio Stick Variables
#pragma once

#include "RmtPPMReader.h"                    // Raising Awesome's PPM pulse capture code
#include "Constants.h"                       // Project constants
#include "RawpterIMU.h"
#include "Types.h"
#include "RawpterTelemetry.h"

struct RadioData
{
  // Radio communication:
  RmtPPMReader radio;

  int16_t PWM_throttle=PWM_throttle_zero;
  int16_t PWM_roll=PWM_roll_fs;
  int16_t PWM_pitch=PWM_pitch_fs;
  int16_t PWM_yaw=PWM_yaw_fs;
  int16_t PWM_ThrottleCutSwitch, PWM_FailsafeSwitch;
  int16_t PWM_throttle_prev, PWM_roll_prev, PWM_pitch_prev, PWM_yaw_prev;
  bool failsafed = false;
  bool throttle_is_cut = true; // used to force the pilot to manually set the throttle to zero after the switch is used to throttle cut
    
  void begin(uint8_t pin, uint32_t rmtFreqHz, uint16_t pulses, uint16_t pulseDurationThreshold)
  {
    radio.begin(pin,rmtFreqHz,pulses,pulseDurationThreshold);
  }
  
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

  inline void setToFailsafe()
  {
    // This would only be called if the receiver itself had a major problem or is not set to failsafe
    // in it's failsafe settings. If it loses
    // connection, it gives its own failsafe value. To signal a failsafe state, the
    // receiver must be setup to failsafe switchB to 2000 to flag the break of connection.
    // With that, this would not be called. It only gets called if there is no valid value at all.
    // That's bad news and so this routine basically drops it from the sky as it would be in a hopeless state.
    PWM_throttle = PWM_throttle_fs;
    PWM_roll = PWM_roll_fs;
    PWM_pitch = PWM_pitch_fs;
    PWM_yaw = PWM_yaw_fs;
    PWM_ThrottleCutSwitch = PWM_ThrottleCutSwitch_fs;
    PWM_FailsafeSwitch = PWM_FailsafeSwitch_fs;
  }

  void headHome(GPSData& gps, RawpterIMU& imu, ConfigData& configData, bool reset) {
    static float prev_errN = 0.0f, prev_errE = 0.0f;
    static float integralN = 0.0f, integralE = 0.0f;
    static float velN_filt = 0.0f, velE_filt = 0.0f;
    static unsigned long last_gps_time = 0;
    
    // PID Gains - Tune these carefully!
    const float K_p = 5.0f;
    const float K_i = 0.01f;  // Small I-term for wind
    const float K_d = 0.0f;   // D-term for damping
    const float K_accel2pwm = 5.0f;
    const float Alpha = 0.3f; // Low Pass Filter constant (0.0 to 1.0)
    
    static bool first_run = true;
    
    if (reset) {
        last_gps_time = millis();
        integralN = 0; integralE = 0;
        velN_filt = 0; velE_filt = 0;
        first_run = true;
        return; 
    }

    if (gps.useGPS) {
        float errN, errE;
        gpsErrorToNE(gps.latitude, gps.longitude, gps.homePosLatitude, gps.homePosLongitude, errN, errE);

        if (first_run) {
            prev_errN = errN; prev_errE = errE;
            last_gps_time = millis();
            first_run = false;
            return; 
        }

        unsigned long now = millis();
        float dt = (now - last_gps_time) / 1000.0f;
        if (dt<1) {
          PWM_pitch = PWM_pitch_prev;
          PWM_roll = PWM_roll_prev;
          return;
        }
        float distSq = errN * errN + errE * errE;

        // Ensure we have a valid time step and aren't already "home"
        if (distSq > 1.0f && dt > 0.001f) {
            // 1. Calculate Raw Velocity
            float rawVelN = (errN - prev_errN) / dt;
            float rawVelE = (errE - prev_errE) / dt;

            // 2. Low Pass Filter the velocity (Reduces GPS jitter)
            velN_filt = (Alpha * velN_filt) + ((1.0f - Alpha) * rawVelN);
            velE_filt = (Alpha * velE_filt) + ((1.0f - Alpha) * rawVelE);

            // 3. Accumulate Integral (Wind compensation)
            integralN += errN * dt;
            integralE += errE * dt;
            // Anti-windup: limit the integral impact
            integralN = constrain(integralN, -10.0f, 10.0f);
            integralE = constrain(integralE, -10.0f, 10.0f);

            // 4. PID Output in Global (NED) Frame
            float targetAccelN = (K_p * errN) + (K_i * integralN) + (K_d * velN_filt);
            float targetAccelE = (K_p * errE) + (K_i * integralE) + (K_d * velE_filt);

            float psi = imu.yaw_IMU * DEG_TO_RAD; 
            float c = cosf(psi);
            float s = sinf(psi);

            float accelForward =  c * targetAccelN + s * targetAccelE;
            float accelRight   = -s * targetAccelN + c * targetAccelE;

            // 6. PWM Mapping
            PWM_pitch = 1500.0f - (accelForward * K_accel2pwm);
            PWM_roll  = 1500.0f + (accelRight   * K_accel2pwm);
            
            // Safety Limits (150us deviation max)
            PWM_pitch = constrain(PWM_pitch, 1400, 1600);
            PWM_roll  = constrain(PWM_roll, 1400, 1600);
            
            gps.atHome = false;
        } else {
            // Arrived or stationary: Level out
            PWM_pitch = 1500;
            PWM_roll  = 1500;
            if (distSq <= 1.0f) gps.atHome = true;
        }

        prev_errN = errN;
        prev_errE = errE;
        last_gps_time = now;

    } else {
        PWM_pitch = 1500;
        PWM_roll  = 1500;
        gps.atHome = true; 
    }
  }

  void gpsErrorToNE(double lat_now, double lon_now,
                          double lat_home, double lon_home,
                          float &errN_m, float &errE_m)
  {
      
      double dLat = (lat_home - lat_now) * DEG_TO_RAD;
      double dLon = (lon_home - lon_now) * DEG_TO_RAD;
      double latRad = lat_now * DEG_TO_RAD;

      double R = 6378137.0; // Earth radius
      errN_m = (float)(R * dLat);
      errE_m = (float)(R * cos(latRad) * dLon);
  }

  inline void getRadioStickValues(AltitudeData& altitudeData, GPSData& gps, RawpterTelemetry& telemetry, ConfigData& configData, RawpterIMU& imu)
  {
    static int throttleOverrideCounter = 0;
    static bool landing = false;
    static float integralAltitudeRate = 0 ;
    static unsigned long landingTicks = 0;
    static bool resetGPS = false;
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
      if (PWM_FailsafeSwitch < 1900 /*landing*/ && telemetry.flying)
      {   
          if (!landing)
          {
            landingTicks = 0.0f;
            integralAltitudeRate = 0.0f; // reset integrator on landing start
            throttleOverrideCounter = 0;
            landing = true;
            resetGPS = true;
            gps.atHome = false; // reset atHome flag to false to ensure heading home.
          }
          

          if (landingTicks++ > 20000 || altitudeData.altitude < 1.5f) // 1000 ticks per second - if we have been landing for more than 5 seconds, kill it.
          {
            throttle_is_cut = true; // if we are in this mode 5 seconds, then kill motors to prevent uncontrolled flyaway.
            landing = false;
            PWM_throttle = 1500;
            return;
          }
          else
          {
              if (altitudeData.altitude > 8.0f&&altitudeData.altitude < 11.0f)
                headHome(gps,imu,configData, resetGPS); // this will override pitch and roll PWM to head home if a GPS exists.
              else
              { // Level out when we are adjusting our altitude
                PWM_pitch = 1500;
                PWM_roll = 1500; 
              }
              resetGPS = false;
              if (++throttleOverrideCounter >= INNER_LOOP_FREQUENCY/ALT_FREQ_HZ) // Only make a move if we are at the altitude control frequency
              {
                  throttleOverrideCounter = 0;
                  float landingRate = altitudeData.targetRateLanding; // Default to heading downward.
                  if (gps.useGPS && !gps.atHome) // Once this is working, change this such that it will go do to a low altitude like 12ft while it heads back.
                  {
                      if (altitudeData.altitude < 8.0f) 
                      {
                        landingRate = 0.02f; // Until we are at home, stay between 8.0 - 11.0 ft
                      }
                      else
                      {
                        if (altitudeData.altitude < 11.0f) 
                        {
                          landingRate = 0.0f; // Hold altitude if between 8.0 - 11.0 ft
                        }
                      }
                  }
                  float rateError  = landingRate - altitudeData.rateFPS; // Going up example:  -.1 - 1 = -1.1 (error is negative), Going down example:  -.1 - (-1) = .9
                  
                  // Integrator
                  integralAltitudeRate += rateError * 0.01f; // 100 Hz is ticks at .01 seconds
                  
                  if (rateError > 1.8f && integralAltitudeRate < 0.0f)
                  {   // if going down too fast and the integrator has been decreasing PWM, reset it to zero.
                      integralAltitudeRate = 0.0f;
                  }
                  else if (rateError < -1.8f && integralAltitudeRate > 0.0f)
                  {   // if going up too fast and the integrator has been increasing PWM, reset it to zero.
                      integralAltitudeRate = 0.0f;
                  }

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