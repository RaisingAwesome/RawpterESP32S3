#pragma once

#include "driver/mcpwm_prelude.h"            // ESP32-S3 routines for motor control
#include "Constants.h"
#include "RadioData.h" 

struct RawpterMotors
{
  // Motor Mixer
  unsigned long ESCWriteCounter = 0;
  float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled;
  float m1_command_scaled_prev, m2_command_scaled_prev, m3_command_scaled_prev, m4_command_scaled_prev;
  int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM;
  const int motor_pins[4] = {m1Pin, m2Pin, m3Pin, m4Pin};
  RadioData &radioData;
  RawpterMotors(RadioData &r) : radioData(r) {}

  // Motor Electronic Speed Control Modules (ESC):
  
  mcpwm_cmpr_handle_t comparators[4];
  float motor_ramp_step = 1.0f / (RAMP_DURATION_SEC * MOTOR_FREQ_HZ);

  void begin() {
    Serial.println("Initializing Clean 4-Motor MCPWM (No Glitches)...");
    uint32_t period = 1000000 / ESC_FREQ_HZ;
    
    // 1. Single Timer for all 4 motors = Perfect hardware sync
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
      .group_id = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_PLL160M,
      .resolution_hz = 1000000,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
      .period_ticks = period,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    // 2. Use 2 Operators (3 is the max per group)
    for (int op_idx = 0; op_idx < 2; op_idx++) {
      mcpwm_oper_handle_t oper = NULL;
      mcpwm_operator_config_t oper_config = { .group_id = 0 };
      ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper));
      ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

      // 3. Each Operator drives 2 Generators (A and B)
      for (int gen_idx = 0; gen_idx < 2; gen_idx++) {
        int motor_id = (op_idx * 2) + gen_idx;
        mcpwm_gen_handle_t gen = NULL;
        mcpwm_generator_config_t gen_config = { .gen_gpio_num = motor_pins[motor_id] };
        ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_config, &gen));

        // 4. FIX: Only update on TEZ to prevent runt pulses/overheating
        mcpwm_comparator_config_t comp_config = {
          .flags = { .update_cmp_on_tez = true, .update_cmp_on_tep = false } 
        };
        ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comp_config, &comparators[motor_id]));

        // Define standard PWM behavior
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen,
          MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen,
          MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[motor_id], MCPWM_GEN_ACTION_LOW)));
      }
    }

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
  }

  void sendPWMtoESC(int motorNumber, uint32_t pwm)
  {
    mcpwm_comparator_set_compare_value(comparators[motorNumber-1], pwm);
  }
  
  void kill()
  {
    // sets the PWM to its lowest value to shut off a motor such as when the throttle cut switch is flipped or it gets too steep of an angle.
    radioData.throttle_is_cut = true;
    m1_command_PWM = 1000;      // This is milliseconds for PWM.  1000 is off. 2000 is full throttle.
    m1_command_scaled_prev = 0; // This is 0 to 1.
    m2_command_PWM = 1000;
    m2_command_scaled_prev = 0;
    m3_command_PWM = 1000;
    m3_command_scaled_prev = 0;
    m4_command_PWM = 1000;
    m4_command_scaled_prev = 0;
  }
};
