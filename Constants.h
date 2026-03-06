// Constants.h
// Values that match installed hardware or provide clarity to magic numbers

#pragma once

// Loop Timing
constexpr float INNER_LOOP_FREQUENCY = 1000.0f; // The altitude read takes out of processing time. Could get 2K easy if not for it.
constexpr float IMU_FREQ_HZ = 1000.0f;          // We will get the IMU data as fast as the inner loop so that we can apply our filters to any noise.
constexpr float PID_FREQ_HZ = 250.0f;           // This is limited by how fast your ESCs can change setpoint.
constexpr float MOTOR_FREQ_HZ = 250.0f;         // The motor actuation is limited by the ESC Simonk firmware. 400HZ is the fastest you can make a change of pulse set point. 250 will evenly put it in the inner loop
constexpr float RAMP_DURATION_SEC = 0.35f;      // how long in seconds to take a command a motor to go from 1000 PWM to 2000 PWM. The ConditionCommands() is the routine that ramps it. Keeps it from body slamming itself on a spike.
constexpr float ALT_FREQ_HZ = 100.0f;           // Check altitude just 100 times per second.
constexpr float ESC_FREQ_HZ = 250.0f;           // The full HZ tick width of each Simonk frame for PWM.

// Radio Pins
// pin definition for PPM
constexpr uint8_t stickRightHorizontal = 1; // right horizontal stick, roll
constexpr uint8_t stickRightVertical = 2;   // right vertical stick, pitch
constexpr uint8_t stickLeftVertical = 3;    // left vertical stick, throttle
constexpr uint8_t stickLeftHorizontal = 4;  // left horizontal stick, yaw
constexpr uint8_t SwitchA = 5;              // SWA switch, throttle cut
constexpr uint8_t SwitchB = 6;              // SWB switch, failsafed
constexpr uint8_t PPM_PIN = 42;               // input pin for PPM
constexpr uint8_t CHANNELS = 6;                // number of channels
constexpr uint8_t throttlePin = stickLeftVertical; // throttle - up and down
constexpr uint8_t rollPin = stickRightHorizontal;  // ail (roll)
constexpr uint8_t pitchPin = stickRightVertical;   // elevation (pitch)
constexpr uint8_t yawPin = stickLeftHorizontal;    // rudder (yaw)
constexpr uint8_t throttleCutSwitchPin = SwitchA;  // throttle cut
constexpr uint8_t failsafePin = SwitchB;           // Receiver failsafed - force a landing

//Failsafe PWM Settings
constexpr int16_t PWM_throttle_zero = 1500; // used when we want to take throttle to zero.  Failsafe is something higher as it is expected that failsafe is a value needed to safely land.
constexpr int16_t PWM_throttle_fs = 1500;
constexpr int16_t PWM_roll_fs = 1500;              // it quits turning
constexpr int16_t PWM_pitch_fs = 1500;             // elev
constexpr int16_t PWM_yaw_fs = 1500;               // rudd
constexpr int16_t PWM_ThrottleCutSwitch_fs = 2000; // SWA less than 1300, cut throttle, but we don't want to cut throttle, just allow it to decrease so it lands. - must config a switch to Channel 5 in your remote.
constexpr int16_t PWM_FailsafeSwitch_fs = 1000;    // Used to flag that the receiver had to go to failsafe

// Song notes
// Octave 4
constexpr int Note_C4 = 262;
constexpr int Note_CS4 = 277;
constexpr int Note_D4 = 294;
constexpr int Note_DS4 = 311;
constexpr int Note_E4 = 330;
constexpr int Note_F4 = 349;
constexpr int Note_FS4 = 370;
constexpr int Note_G4 = 392;
constexpr int Note_GS4 = 415;
constexpr int Note_A4 = 440;
constexpr int Note_AS4 = 466;
constexpr int Note_B4 = 494;

constexpr int Note_C5 = 523;
constexpr int Note_CS5 = 554;
constexpr int Note_D5 = 587;
constexpr int Note_DS5 = 622;
constexpr int Note_E5 = 659;
constexpr int Note_F5 = 698;
constexpr int Note_FS5 = 740;
constexpr int Note_G5 = 784;
constexpr int Note_GS5 = 831;
constexpr int Note_A5 = 880;
constexpr int Note_AS5 = 932;
constexpr int Note_B5 = 988;

// The battery alarm code handles 14.8 or 7.4V LIPOs.  Set to your type of battery. If not hooked up, it will pull down and beep often.
constexpr float BATTERYTYPE = 14.8f;
constexpr uint8_t BUZZER_PIN = 47; // Pin 27, GPIO47
constexpr uint8_t BATTERY_PIN = 4;        // GPIO26, Pin 26 - not actually an analog input, so this needs reassigned in the next build. Using it for an output now.

// IMU
constexpr uint8_t IMU_INT_PIN = 9; // GPIO7 (pin 11 on ESP32-S3 MINI)
constexpr uint8_t ACCEL_DATA_X1 = 0x1F;
constexpr uint8_t GYRO_DATA_X1 = 0x25;
constexpr float G_PER_LSB = 1.0f / 2048.0f; // ACCEL_FS_SEL = 1, ±16 g, Page 11 of IMU-icm-40609-d_v1.2.pdf
constexpr float DPS_PER_LSB = 1.0f / 32.8f; // GYRO_FS_SEL = 1, ±1000 dps, Page 10 of IMU-icm-40609-d_v1.2.pdf
constexpr float USEC_TO_SEC = 1.0f / 1000000.0f;

// SPI setup
constexpr uint8_t SPI_SCLK = 10;
constexpr uint8_t SPI_MISO = 14;
constexpr uint8_t SPI_MOSI = 13;
constexpr uint8_t SPI_CS = 12; // Chip select pin for IMU SPI

// Motors
constexpr uint8_t m1Pin = 15; // IO15
constexpr uint8_t m2Pin = 16; // IO16
constexpr uint8_t m3Pin = 7; // IO7
constexpr uint8_t m4Pin = 6; // IO6