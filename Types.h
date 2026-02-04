// Types.h
//Structs and variables
#pragma once

#include "RmtPPMReader.h"                    // Raising Awesome's PPM pulse capture code
#include "Constants.h"                     // Project constants

struct Limits
{  // Rate Limits - need to expand this to bring in the K constants and desired setpoints
  float maxRate =120; // deg/s
};

struct ConfigData
{
  // Radio failsafe values for every channel in the event that bad reciever data is detected.
  // These are for it to stay stable and descend safely versus totally cutting throttle and drop like a rock.
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

struct Motors
{
  // Motor Mixer
  unsigned long ESCWriteCounter = 0;
  float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled;
  float m1_command_scaled_prev, m2_command_scaled_prev, m3_command_scaled_prev, m4_command_scaled_prev;
  int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM;
  
  // Motor Electronic Speed Control Modules (ESC):
  const int motor_pins[] = {m1Pin, m2Pin, m3Pin, m4Pin};
  mcpwm_cmpr_handle_t comparators[4];
  float motor_ramp_step = 1.0f / (RAMP_DURATION_SEC * MOTOR_FREQ_HZ);
  void begin()
  {
    Serial.println("Initializing Split-Group MCPWM with Phase Shift...");

    mcpwm_timer_handle_t timers[2] = {NULL, NULL};
    uint32_t period = 1000000 / ESC_FREQ_HZ;
    
    for (int g = 0; g < 2; g++) {
        mcpwm_timer_config_t timer_config = {};
        timer_config.group_id = g;
        timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
        timer_config.resolution_hz = 1000000;
        timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
        timer_config.period_ticks = period;
        ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timers[g]));
    }

    // --- PHASE SHIFT LOGIC ---
    // Instead of chaining Group 0 to Group 1, we initialize them normally.
    // To stagger them, we manually start Group 1 with an offset.
    
    for (int i = 0; i < 4; i++) {
        int group = (i < 2) ? 0 : 1; 

        mcpwm_oper_handle_t oper = NULL;
        mcpwm_operator_config_t oper_config = { .group_id = group };
        ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timers[group]));

        mcpwm_gen_handle_t gen = NULL;
        mcpwm_generator_config_t gen_config = { .gen_gpio_num = motors.motor_pins[i] };
        ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_config, &gen));

        mcpwm_comparator_config_t comp_config = {};
        comp_config.flags.update_cmp_on_tez = true;
        ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comp_config, &motors.comparators[i]));

        mcpwm_comparator_set_compare_value(motors.comparators[i], 1000);

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motors.comparators[i], MCPWM_GEN_ACTION_LOW)));
    }

    // Enable both
    ESP_ERROR_CHECK(mcpwm_timer_enable(timers[0]));
    ESP_ERROR_CHECK(mcpwm_timer_enable(timers[1]));

    // Start Group 0 immediately
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timers[0], MCPWM_TIMER_START_NO_STOP));
    
    // DELAY start of Group 1 by half the period to stagger the current hit on changes
    delayMicroseconds(period / 2); 
    
    // Start Group 1
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timers[1], MCPWM_TIMER_START_NO_STOP));

    Serial.println("MCPWM: Groups staggered using timed-start.");
    
    // Reset command values
    motors.m1_command_PWM = motors.m2_command_PWM = motors.m3_command_PWM = motors.m4_command_PWM = 1000;
    PWM_throttle = PWM_throttle_zero;
    PWM_roll     = PWM_roll_fs;
    PWM_pitch    = PWM_pitch_fs;
    PWM_yaw      = PWM_yaw_fs;
  }
};


struct Telemetry
{
  //Telemetry
static int16_t highestThrottlePWM = 1500;
static int16_t lowestThrottlePWM = 2000;
static bool flying = false;
};
