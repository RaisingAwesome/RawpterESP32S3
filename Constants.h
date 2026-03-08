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
constexpr uint8_t PPM_PIN = 42;               // IO42 input pin for PPM

constexpr uint8_t stickRightHorizontal = 1; // right horizontal stick, roll
constexpr uint8_t stickRightVertical = 2;   // right vertical stick, pitch
constexpr uint8_t stickLeftVertical = 3;    // left vertical stick, throttle
constexpr uint8_t stickLeftHorizontal = 4;  // left horizontal stick, yaw
constexpr uint8_t SwitchA = 5;              // SWA switch, throttle cut
constexpr uint8_t SwitchB = 6;              // SWB switch, failsafed
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

// The battery alarm code handles 14.8 or 7.4V LIPOs.  Set to your type of battery. If not hooked up, it will pull down and beep often.
constexpr uint8_t BUZZER_PIN = 47; // GPIO47
constexpr uint8_t BATTERY_PIN = 4; // GPIO4
constexpr float BATTERYTYPE = 14.8f; // 4S LIPO

// IMU
constexpr uint8_t IMU_INT_PIN = 9; // GPIO7 (pin 11 on ESP32-S3 MINI)

constexpr uint8_t ACCEL_DATA_X1 = 0x1F; // Register that holds the Accelerometer data. We can burst read 12 bytes from here to get all the accel and gyro data in one transaction.
constexpr uint8_t GYRO_DATA_X1 = 0x25; // Register that holds the Gyro data. We can burst read 12 bytes from here to get all the accel and gyro data in one transaction.
constexpr float G_PER_LSB = 1.0f / 2048.0f; // ACCEL_FS_SEL = 1, ±16 g, Page 11 of IMU-icm-40609-d_v1.2.pdf
constexpr float DPS_PER_LSB = 1.0f / 32.8f; // GYRO_FS_SEL = 1, ±1000 dps, Page 10 of IMU-icm-40609-d_v1.2.pdf
constexpr float USEC_TO_SEC = 1.0f / 1000000.0f;

// SPI IO Pins
constexpr uint8_t SPI_SCLK = 10; // IO10
constexpr uint8_t SPI_MISO = 14; // IO14
constexpr uint8_t SPI_MOSI = 13; // IO13
constexpr uint8_t SPI_CS = 12; // IO12  Chip select pin for IMU SPI

// I2C IO Pins
constexpr uint8_t I2C_SDA = 18; //IO18
constexpr uint8_t I2C_SCL = 8; //IO8

// Motors
constexpr uint8_t m1Pin = 15; // IO15
constexpr uint8_t m2Pin = 16; // IO16
constexpr uint8_t m3Pin = 7; // IO7
constexpr uint8_t m4Pin = 6; // IO6
