// Rawpter 8.0 by Sean J. Miller
// Flight Controller code for ESP32-S3
// Go to https:// raisingawesome.site/projects for more info
// MIT License - use at your own risk

#include <SPI.h>  // SPI Communication Support for IMU
#include <WiFi.h> // ESP32 Adhoc WiFi Support
#include <WebServer.h>
#include <Wire.h>                            // I2C Communication for GPS and Pressure Sensor
#include <SparkFun_BMP581_Arduino_Library.h> // BMP581 Pressure Sensor
#include <SparkFun_u-blox_GNSS_v3.h>         // Max10S GPS
#include <functional>
#include "driver/gpio.h"   // speeds up digitalWrite
#include "driver/rmt_rx.h" // for PPM pulse capture
#include "RmtPPMReader.h"

// ========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //
// ========================================================================================================================//
// EASYCHAIR is used to put in some key dummy variables so you can sit in the easy chair with just the Arduino on a cord wiggling it around and seeing how it responds.
// Set EASYCHAIR to false when you are wanting to install it in the drone.
#define EASYCHAIR false

#define INNER_LOOP_FREQUENCY 2000.0f // The altitude read takes out of processing time. Could get 2K easy if not for it.
#define IMU_FREQ_HZ 2000.0f          // We will get the IMU data as fast as the inner loop so that we can apply our filters to any noise.
#define PID_FREQ_HZ 400.0f           // This is limited by how fast your ESCs can change setpoint.
#define MOTOR_FREQ_HZ 400.0f         // The motor actuation is limited by the ESC Simonk firmware. 400HZ is the fastest you can make a change of pulse set point.
#define RAMP_DURATION_SEC 0.35f      // how long in seconds to take a command a motor to go from 1000 PWM to 2000 PWM. The ConditionCommands() is the routine that ramps it. Keeps it from body slamming itself on a spike.
#define ALT_FREQ_HZ 100.0f           // Check altitude just 100 times per second.

#define ESC_FREQ_HZ 400
#define ESC_RESOLUTION 14
#define ESC_PERIOD_US 2500
#define ESC_MAX_DUTY 16383
// counts per µs (precompute)
#define ESC_US_TO_DUTY 6.5536f

// Parameter Storage
#include <Preferences.h>
Preferences prefs;

// The battery alarm code handles 14.8 or 7.4V LIPOs.  Set to your type of battery. If not hooked up, it will pull down and beep often.
#define BATTERYTYPE 14.8
#define BUZZER_PIN 47         // Pin 27, GPIO47
#define BATTERY_PIN 26        // GPIO26, Pin 26 - not actually an analog input, so this needs reassigned in the next build. Using it for an output now.
#define DEG_TO_RAD 0.0174533f // π / 180

// IMU
#define IMU_INT_PIN 7 // GPIO7 (pin 11 on ESP32-S3 MINI)
#define ACCEL_DATA_X1 0x1F
#define GYRO_DATA_X1 0x25

// SPI setup
#define SPI_SCLK 12
#define SPI_MISO 13
#define SPI_MOSI 11
#define SPI_CS GPIO_NUM_10 // Chip select pin for IMU SPI

// GPS
static SFE_UBLOX_GNSS myGNSS;

// PID parameters (this is where you "tune it".  It's best to use the WiFi interface to do it live and then update once its tuned.):
static float i_limit = 0.5f;      // Integrator saturation level
static float i_limit_rate = 4.0f; // Integrator saturation level

static float Kp_roll_angle = 3.0f;   // Roll P-gain
static float Ki_roll_angle = 0.001f; // Roll I-gain

static float Kp_pitch_angle = 3.0f;   // Pitch P-gain
static float Ki_pitch_angle = 0.001f; // Pitch I-gain

static float Kp_roll_rate = 0.0015f; // Roll P-gain
static float Ki_roll_rate = 0.003f; // Roll I-gain
static float Kd_roll_rate = 0.0f;   // Roll D-gain

static float Kp_pitch_rate = 0.0015f; // Pitch P-gain
static float Ki_pitch_rate = 0.003f; // Pitch I-gain
static float Kd_pitch_rate = 0.0f;   // Pitch D-gain

static float Kp_yaw_rate = 0.003f; // Yaw P-gain default 30
static float Ki_yaw_rate = 0.0001f; // Yaw I-gain default 5
static float Kd_yaw_rate = 0.0f;    // Yaw D-gain default .015 (be careful when increasing too high, motors will begin to overheat!)

struct Limits
{                // Rate Limits - need to expand this to bring in the K constants and desired setpoints
  float maxRate; // deg/s
};

// Instantiate per-axis limits (conservative values for 10" props)
Limits rollLimits = {160.0f}; // maxRate, maxAccel, maxJerk
Limits pitchLimits = {160.0f};

static float desiredRateRoll, desiredRatePitch;

// Radio failsafe values for every channel in the event that bad reciever data is detected.
// These are for it to stay stable and descend safely versus totally cutting throttle and drop like a rock.
static int16_t PWM_throttle_zero = 1500; // used when we want to take throttle to zero.  Failsafe is something higher as it is expected that failsafe is a value needed to safely land.
static int16_t PWM_throttle_fs = 1500;
static int16_t PWM_roll_fs = 1500;              // it quits turning
static int16_t PWM_pitch_fs = 1500;             // elev
static int16_t PWM_yaw_fs = 1500;               // rudd
static int16_t PWM_ThrottleCutSwitch_fs = 2000; // SWA less than 1300, cut throttle, but we don't want to cut throttle, just allow it to decrease so it lands. - must config a switch to Channel 5 in your remote.
static int16_t PWM_FailSafed_fs = 2000;         // Used to flag that the receiver had to go to failsafe
static float stick_dampener = 1.0f;             // 0.1-1 Lower=slower, higher=noiser default 0.7
static bool failsafed = false;
static bool failsafeTriggered = false;
static unsigned long failsafeTime = 0;
static bool throttle_is_cut = true; // used to force the pilot to manually set the throttle to zero after the switch is used to throttle cut

// Madgwick Parameters (the method that calculates angles fromt he IMU)
static float B_madgwick = 0.06f;
static float recipNorm;
static float s0, s1, s2, s3;
static float qDot1, qDot2, qDot3, qDot4;
static float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
static float q0 = 1.0f; // Initialize quaternion for madgwick filter
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;
static unsigned long lastMadgwickUpdateMicros = 0;
static float madDeltaTime = 1 / IMU_FREQ_HZ;

// pin definition for PPM
static const int stickRightHorizontal = 1; // right horizontal stick, roll
static const int stickRightVertical = 2;   // right vertical stick, pitch
static const int stickLeftVertical = 3;    // left vertical stick, throttle
static const int stickLeftHorizontal = 4;  // left horizontal stick, yaw
static const int SwitchA = 5;              // SWA switch, throttle cut
static const int SwitchB = 6;              // SWB switch, failsafed
// end of pin definition

static const int throttlePin = stickLeftVertical; // throttle - up and down
static const int rollPin = stickRightHorizontal;  // ail (roll)
static const int upDownPin = stickRightVertical;  // elevation (pitch)
static const int ruddPin = stickLeftHorizontal;   // rudder (yaw)
static const int throttleCutSwitchPin = SwitchA;  // throttle cut
static const int failsafePin = SwitchB;           // Receiver failsafed - force a landing
static int16_t failsafeThrottlePWM = 1650;        // A safe throttle for descent.
static int16_t highestThrottlePWM = 1500;
static int16_t lowestThrottlePWM = 2000;
static float trimYaw = 0;
static float trimPitch = 0;
static float trimRoll = 0;

// PPM variables
#define PPM_PIN 21    // input pin for PPM
#define CHANNELS 6    // number of channels
#define BLANK_US 4000 // blank time threshold in µs

static volatile byte state = LOW;
static volatile byte previous_state = LOW;
static volatile unsigned long microsAtLastPulse = 0;
static unsigned minChannelPWMValue = 900;       // Used to determine if a channels reported pulse width is realistically valid.
static unsigned maxChannelPWMValue = 2100;      // Used to determine if a channels reported pulse width is realistically valid.
static unsigned long failsafeTimeout = 500000L; // The timeout (microseconds) after which the channels which were not updated are considered invalid
RmtPPMReader radio;

static volatile bool start = false;
static int throttleCutCounter = 0;
static int throttleNotCutCounter = 0;

static rmt_channel_handle_t rx_channel;

// Motor Electronic Speed Control Modules (ESC):
static const int m1Pin = 1; // IO1, Pin 5
static const int m2Pin = 3; // IO3
static const int m3Pin = 5; // IO5
static const int m4Pin = 4; // IO4

static float motor_ramp_step = 1.0f / (RAMP_DURATION_SEC * MOTOR_FREQ_HZ);

// GPS
static bool hasGPS = false;

// Pressure
// Create a new sensor object
struct AltitudeData
{
  BMP581 pressureSensor;
  uint8_t i2cAddress = 0x46;      // 0x46 is the Pressure Sensor 0x42 is the GPS
  volatile float altitude = 0.0f; // just a default until it starts to loop.
  float highestAltitude = 0.0f;
  const float ceiling = 12.0f;
  volatile float rateFPS = 0.0f;         // Used to ensure it doesn't descend too quickly
  volatile float rateFPSWorking = 0.0f;  // This one is worked by the pressure task and then synced to rateFPS variable in the main loop.
  volatile float altitudeWorking = 0.0f; // This one is worked by the pressure task and then synced to altitude variable in the main loop.
  float maxAltitudeChangeRate = .8f;
  float kp_altitude_rate = .0001f;
  float ki_altitude_rate = .0001f;
  volatile float invGroundPressure = 0.0f;        // reciprocal of baseline pressure at startup
  volatile float invGroundPressureWorking = 0.0f; // This one is worked by the pressure task and then synced to invGroundPressure variable in the main loop.
  TaskHandle_t bmpTaskHandle= nullptr;                     // Handle for the BMP581 task that will run on its own core.
};
AltitudeData altitudeData{};
// General stuff for controlling timing of things
static float deltaTime = 0.0f;
static unsigned long innerLoopMicroseconds = 1000000.0 / INNER_LOOP_FREQUENCY; // The microsecond equivalent of our Loop Hz.
static unsigned long lastPIDmicros = 0;
static bool resetTimers = false;
static unsigned long tick_time, prev_time;
static unsigned long print_counter, serial_counter;
static volatile unsigned long debugger;
static bool flying = false;
TaskHandle_t loopDroneHandle; // Handle for the main drone control loop task that will run on its own core.

// Radio communication:
static int16_t PWM_throttle, PWM_roll, PWM_pitch, PWM_yaw, PWM_ThrottleCutSwitch, PWM_Failsafed;
static int16_t PWM_throttle_prev, PWM_roll_prev, PWM_pitch_prev, PWM_yaw_prev;
static float maxRoll = 35.0f;  // Max roll angle in degrees the sticks can achieve
static float maxPitch = 35.0f; // Max pitch angle in degrees the sticks can achieve
static float maxYaw = 160.0f;  // Max yaw rate in deg/sec (default 160.0)

// IMU:
static float AccX, AccY, AccZ;
static float AccX_prev, AccY_prev, AccZ_prev;
static float GyroX, GyroY, GyroZ;
static float GyroX_prev, GyroY_prev, GyroZ_prev;
static float roll_IMU, pitch_IMU, yaw_IMU;
static float roll_IMU_prev, pitch_IMU_prev;
constexpr float G_PER_LSB = 1.0f / 1024.0f; // ±32 g
constexpr float DPS_PER_LSB = 1.0f / 16.4f; // ±2000 dps
constexpr float USEC_TO_SEC = 1.0f / 1000000.0f;

// IMU calibration parameters - calibrate IMU using calculateIMUEerror() in the void setup() to get these values, then comment out calculateIMUError()
static float AccErrorX = 0.01;
static float AccErrorY = -0.03;
static float AccErrorZ = 0.01;
static float GyroErrorX = 0.55;
static float GyroErrorY = -0.85;
static float GyroErrorZ = 0.63;

static SPIClass IMUSPI(HSPI);

static float Gyro_filter = 0.9f;  // Lower is slower to catch up. Higher is faster to track, but can have a lot of noise. 1 is no filter
static float Accel_filter = 0.9f; // Lower is slower to catch up. Higher is faster to track, but can have a lot of noise. 1 is no filter

// PID Controller:
static float throttle_desired, roll_des, pitch_des, yaw_des; // Normalized desired state
static float error_roll, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
static float error_pitch, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
static float error_yaw, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;
static unsigned long PIDCounter = 0;

// Motor Mixer
static unsigned long ESCWriteCounter = 0;
static float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled;
static float m1_command_scaled_prev, m2_command_scaled_prev, m3_command_scaled_prev, m4_command_scaled_prev;
static int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM;

// Voltage Monitoring and Beeping
static unsigned long buzzer_millis;
static int batteryVoltage = 777;             // just a default for the battery monitoring routine
static unsigned long next_voltage_check = 0; // used in loopBuzzer to check for voltage on the main battery.
static bool beeping = false;                 // For tracking beeping when the battery is getting low.
static float calced_voltage = 14.8;
TaskHandle_t buzzerTaskHandle;

// WiFi Variables
static WiFiServer server(80);
TaskHandle_t wifiTaskHandle;
// WiFi End

// Song notes
// Octave 4
#define Note_C4 262
#define Note_CS4 277
#define Note_D4 294
#define Note_DS4 311
#define Note_E4 330
#define Note_F4 349
#define Note_FS4 370
#define Note_G4 392
#define Note_GS4 415
#define Note_A4 440
#define Note_AS4 466
#define Note_B4 494

#define Note_C5 523
#define Note_CS5 554
#define Note_D5 587
#define Note_DS5 622
#define Note_E5 659
#define Note_F5 698
#define Note_FS5 740
#define Note_G5 784
#define Note_GS5 831
#define Note_A5 880
#define Note_AS5 932
#define Note_B5 988

//========================================================================================================================//
// BEGIN THE CLASSIC SETUP AND LOOP
//========================================================================================================================//

void setup()
{
  // Bootup operations
  setupSerial();             // Sets up both Serial and Serial1 for communication.
  setupPPM();                // Setup the tas to detect PPM pulses from the RC Transmitter
  setupMotorCommunication(); // Sets up ledc channels for motor controol
  beginBuzzerTask();     // Setup pin for reading the Battery
  loadParameters();          // overrides coded parameters with the last stored on the chip or defaults if none exist.
  analogReadResolution(12);  // 12 bit ADC.  Used for sensing battery level in loopBuzzer().
  Wire.begin(8, 9);
  delay(100);
  Wire.setClock(400000); // 400KhZ I2C
  beginWifiTask();
  setupGPS();
  setupPressureSensor();
  setupIMU();
  // calculateIMUError(); // Use periodically to obtain the IMU error factors for calibration.
  Serial.println("Waiting for Radio");
  delay(100);
  playStartSong();
  if (!EASYCHAIR)
    waitForRadio();
  Serial.println("Ready...");
  beginAltitudeTask(); // The BMP581 sensor read is blocking and slows the inner loop.  This runs it in a separate task.
  beginDroneLoopTask(); // The inner loop. It runs at 2K on Core 1 by itself. The rest runs on Core 0.
}

void loop()
{ // Not used since we are running tasks, but must remain for the Arduino IDE.
}

void tick()
{
  // Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = tick_time;
  tick_time = micros();
  deltaTime = (tick_time - prev_time) / 1000000.0; // Time since last tick. Division takes it from micros to seconds.  1000000 ms=1 second
}

void loopDrone(void *pvParameters)
{
  while (true)
  {
    tick();
    syncAltitude();                           // Altitude capture runs at 100mHZ in its own FreeRTOS task. This gives a safe way to update variables without thread conflict.
    getRadioStickValues();                    // Gets the PWM from the radio receiver and overrides if necessary. Pulses are captured by hardware with a rmtRead call and double buffering.
    getIMUdata();                             // Pulls raw gyro a daccelerometer data from IMU at 2kHz. IMU output. Is actually downsampled from 32kHz.
    getDesiredAnglesAndThrottleScaledToOne(); // Convert PWM commands to normalized values at 2kHz. Adds a low pass filter to dampen remote stick noise and user twitchiness.
    PIDControlCalcs();                        // The PID functions at 400Hz Hz. Stabilize on angle setpoint from getDesiredAnglesAndThrottle
    motorPipeline();                          // Commands the motors at at 400Hz. This is the max adjustment rate the Simonk can handle.
    tock();                                   // Yields until the end of the 500uS pacing is met to sustain 2kHz
    troubleShooting();
  }
}

// ========================================================================================================================//
//                                                      FUNCTIONS                                                         //
// ========================================================================================================================//
void syncAltitude()
{ // This makes sure the pressure sensing task "bmpTaskHandle", is not writing to our variables so we don't corrupt or data.
  // the bmpTask use the variables "...Working". Once it notifies that the calculations are complete, we consume them.
  if (ulTaskNotifyTake(pdTRUE, 0) > 0)
  {
    altitudeData.altitude = altitudeData.altitudeWorking;
    altitudeData.rateFPS = altitudeData.rateFPSWorking;
  }
}

void beginAltitudeTask()
{
  // Start the BMP581 task on its own core. It is a slow reader, so we'll keep it out of the way of the other tasks
  // on its own core.
  xTaskCreatePinnedToCore(
      bmpTask,        // Task function
      "BMP581 Task",               // Name
      4096,                        // Stack size (bytes)
      NULL,                        // Parameters
      1,                           // Priority (lower than PID loop)
      &altitudeData.bmpTaskHandle, // Task handle
      0                            // Core 0 since it is slower overall
  );
}

inline void setAltitudeRate()
{
  const int N = 15;
  const float dt = 0.01f;

  static float buf[N] = {0};
  static int i = 0;
  static bool filled = false;
  static int M = 0;
  static float S_y = 0.0f;  // sum y
  static float S_ty = 0.0f; // sum k*y, k = age (0 oldest..M-1 newest)

  if (filled)
  {
    float y_old = buf[i];                  // oldest lives at i
    S_y -= y_old;                          // remove oldest
    S_ty -= 0.0f * y_old;                  // oldest had age 0 (no effect)
    S_ty -= S_y;                           // shift ages: (k -> k-1) ⇒ S_ty = S_ty - S_y
    buf[i] = altitudeData.altitudeWorking; // insert newest at age N-1
    S_y += buf[i];
    S_ty += (N - 1) * buf[i];
    i = (i + 1) % N;
  }
  else
  {
    buf[i] = altitudeData.altitudeWorking;
    S_y += buf[i];
    S_ty += M * buf[i]; // newest age = M
    i = (i + 1) % N;
    M++;
    if (M == N)
    {
      filled = true;
    }
  }

  int m = filled ? N : M;
  if (m < 2)
  {
    altitudeData.rateFPSWorking = 0.0f;
    return;
  }

  // Precompute time sums for ages 0..m-1
  float S_t = (m - 1) * m / 2.0f;
  float S_t2 = (m - 1) * m * (2.0f * m - 1) / 6.0f;

  float denom = (m * S_t2 - S_t * S_t);
  float slope_per_index = (m * S_ty - S_t * S_y) / denom;

  altitudeData.rateFPSWorking = slope_per_index / dt;
}

void bmpTask(void *pvParameters)
{
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz

  while (true)
  {
    // Do the sensor read
    getAltitude();

    // Delay until the next absolute 10 ms boundary
    vTaskDelayUntil(&lastWakeTime, period);
  }
}

inline void getAltitude()
{
  static int flyingCounter = 0;
  if (ulTaskNotifyTake(pdTRUE, 0) > 0)
    altitudeData.invGroundPressureWorking = 0.0f; // Reset ground pressure if notified to do so.
  setAltitude();
  setAltitudeRate();
  if (flying && altitudeData.altitudeWorking > altitudeData.highestAltitude)
    altitudeData.highestAltitude = altitudeData.altitudeWorking;
  if (altitudeData.altitudeWorking >= 2.0f && !flying)
  {
    if (++flyingCounter > 4) // make sure it is not a blip
    {
      flying = true;
      flyingCounter = 0;
    }
  }
  else
    flyingCounter = 0;
  xTaskNotifyGive(loopDroneHandle); // Notify the main loop that new data is available
}

void setAltitude()
{
  const float exponent = 0.190284f;   // barometric exponent
  const float scaleFeet = 145366.45f; // conversion to feet
  bmp5_sensor_data data = {0, 0};

  int8_t err = altitudeData.pressureSensor.getSensorData(&data);
  if (err == BMP5_OK)
  {
    if (altitudeData.invGroundPressureWorking == 0.0f)
    {
      // Capture baseline pressure at ground zero
      altitudeData.invGroundPressureWorking = 1.0f / data.pressure;
    }

    // Ratio of current pressure to baseline
    float ratio = data.pressure * altitudeData.invGroundPressureWorking;
    ratio = fmaxf(ratio, 0.000001f);
    // Compute relative altitude in feet
    float newAltitude = (1.0f - expf(exponent * logf(ratio))) * scaleFeet;
    altitudeData.altitudeWorking = 0.6f * altitudeData.altitudeWorking + 0.4f * newAltitude;
  }
  else
  {
    altitudeData.altitudeWorking = 0.0f; // fallback if sensor read fails
  }
}

void beginWifiTask()
{
  // Start the BMP581 task on its own core. It is a slow reader, so we'll keep it out of the way of the other tasks
  // on its own core.
  xTaskCreatePinnedToCore(
    wifiTask,        // Task function
    "Wifi Task",               // Name
    4096,                        // Stack size (bytes)
    NULL,                        // Parameters
    1,                           // Priority (lower than PID loop)
    &wifiTaskHandle, // Task handle
    0                            // Core 0 since it is slower overall
  );
}

void beginBuzzerTask()
{
  // Start task that handles communication with the buzzer.
  xTaskCreatePinnedToCore(
    buzzerTask,        // Task function
    "Buzzer Task",               // Name
    4096,                        // Stack size (bytes)
    NULL,                        // Parameters
    1,                           // Priority (lower than PID loop)
    &buzzerTaskHandle,            // Task handle
    0                            // Core 0 since it is slower overall
  );
}

void troubleShooting()
{
  //  Lowest priority of the running tasks, so it won't slow anything down.  Could be used for logging or non-deadline type tasks.
  //  Print data at 100hz (uncomment one at a time for troubleshooting)
  //  printRadioData();     // Prints radio pwm values (expected: 1000 to 2000)
  //  printDesiredState();  // Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
  //  printGyroData();      // Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
  //  printAccelData();     // Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
  //  printRollPitchYaw();  // Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
  //  printPIDoutput();     // Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
  //  printMotorCommands(); // Prints the values being written to the motors (expected: 1000 to 2000)
  //  printtock();      // Prints the time between loops in microseconds (expected: microseconds between loop iterations and less the Gyro/Acc update speed.  Set by LOOP_TIMING)
  //  printGPS();
  #if EASYCHAIR
    printJSON();
  #endif
  //  printJSON();
}

void setupPressureSensor()
{
  Serial.println("Setting up BMP581...");

  if (altitudeData.pressureSensor.beginI2C(altitudeData.i2cAddress) != BMP5_OK)
  {
    Serial.println("BMP581 not found!");
    halt();
    return;
  }
  Serial.println("BMP581 discovered at address 0x46");

  int8_t err = BMP5_OK;

  // Set ODR directly
  err = altitudeData.pressureSensor.setODRFrequency(BMP5_ODR_100_2_HZ);
  if (err != BMP5_OK)
  {
    Serial.print("Error setting ODR! Error code: ");
    Serial.println(err);
    halt();
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

  err = altitudeData.pressureSensor.setFilterConfig(&config);
  if (err != BMP5_OK)
  {
    Serial.print("Error setting filter config! Error code: ");
    Serial.println(err);
    halt();
  }
  else
  {
    Serial.println("BMP581 Filter configuration applied successfully.");
  }
}

void halt()
{
  while (true)
    delay(100); // lock forever
}

void setupGPS()
{
  delay(100); // give time for Wire to settle.
  if (!myGNSS.begin(Wire))
  {
    Serial.println("u-blox GNSS not detected over I2C. Check wiring and power.");
    hasGPS = false;
    return;
  }
  hasGPS = true;
  Serial.println("U-Blox GPS discovered.");

  // Configure output protocol and update rate
  myGNSS.setI2COutput(COM_TYPE_UBX); // UBX binary only
  myGNSS.setNavigationFrequency(1);  // 1 Hz navigation solution
  myGNSS.setAutoPVT(true);           // Auto PVT messages

  if (myGNSS.getFixType() < 3)
  {
    Serial.println("Waiting for 3D fix...");
  }

  // UBX-CFG-TP5: enable TIMEPULSE at 1 Hz, 50% duty, aligned to GNSS time
  uint8_t ubxTP5[] = {
      0xB5, 0x62, // Sync chars
      0x06, 0x31, // Class = CFG (0x06), ID = TP5 (0x31)
      0x20, 0x00, // Length = 32 bytes

      // Payload (32 bytes)
      0x00,                   // tpIdx = 0 (TIMEPULSE pin)
      0x00,                   // reserved
      0xE8, 0x03, 0x00, 0x00, // freqPeriod = 1000 (1 Hz)
      0xE8, 0x03, 0x00, 0x00, // freqPeriodLock = 1000
      0xF4, 0x01, 0x00, 0x00, // pulseLen = 500 ms (50% duty)
      0xF4, 0x01, 0x00, 0x00, // pulseLenLock = 500 ms
      0x00, 0x00, 0x00, 0x00, // userConfigDelay
      0x81, 0x00, 0x00, 0x00, // flags: enable, aligned to GPS time
      0x00, 0x00, 0x00, 0x00, // reserved
      0x00, 0x00              // checksum placeholder
  };

  // Calculate checksum
  uint8_t ckA, ckB;
  calcChecksum(ubxTP5, sizeof(ubxTP5), ckA, ckB);
  ubxTP5[sizeof(ubxTP5) - 2] = ckA;
  ubxTP5[sizeof(ubxTP5) - 1] = ckB;

  // Send raw UBX message
  myGNSS.pushRawData(ubxTP5, sizeof(ubxTP5));

  Serial.println("UBX-CFG-TP5 sent: TIMEPULSE enabled at 1 Hz.");
}

void calcChecksum(uint8_t *msg, uint16_t len, uint8_t &ckA, uint8_t &ckB)
{
  ckA = 0;
  ckB = 0;
  for (uint16_t i = 2; i < len - 2; i++)
  {
    ckA += msg[i];
    ckB += ckA;
  }
}

void printGPS()
{
  if (tick_time - print_counter > 10000)
  {
    print_counter = micros();
    myGNSS.checkUblox(); // Process incoming data

    if (myGNSS.getInvalidLlh() == false)
    {
      Serial.print("Lat: ");
      Serial.println(myGNSS.getLatitude() / 1e7, 7);
      Serial.print("Lon: ");
      Serial.println(myGNSS.getLongitude() / 1e7, 7);
      Serial.print("Alt: ");
      Serial.println(myGNSS.getAltitude() / 1000.0, 2); // meters
    }
  }
}

inline void controlMixer()
{
  // DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
   * should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
   * normalized (0 to 1) throttle_desired command for throttle control.
   *
   * Relevant variables:
   * throttle_desired - direct throttle control
   * roll_PID, pitch_PID, yaw_PID - stabilized axis variables
   */

  // Thrust increaser: increase thrust whenever a roll or pitch is desired
  float roll_rad = roll_des * DEG_TO_RAD;
  float pitch_rad = pitch_des * DEG_TO_RAD;

  // Approximate total tilt angle (hypotenuse of roll and pitch)
  float tilt_angle_rad = sqrtf(roll_rad * roll_rad + pitch_rad * pitch_rad);

  // Compute thrust compensation factor
  float thrust_compensation = 1.0f / cos(tilt_angle_rad);
  thrust_compensation = constrain(thrust_compensation, 1.0f, 1.3f); // Prevent runaway

  // Apply to throttle
  float throttle_corrected = throttle_desired * thrust_compensation;

  // Quad mixing
  m1_command_scaled = throttle_corrected - pitch_PID + roll_PID + yaw_PID; // Front left
  m2_command_scaled = throttle_corrected - pitch_PID - roll_PID - yaw_PID; // Front right
  m3_command_scaled = throttle_corrected + pitch_PID - roll_PID + yaw_PID; // Back right
  m4_command_scaled = throttle_corrected + pitch_PID + roll_PID - yaw_PID; // Back left

  // ---Normalize if any motor > 1.0 --- This is assuming scaling down has the same thrust dynamics.
  float max_val = fmaxf(fmaxf(m1_command_scaled, m2_command_scaled),
                        fmaxf(m3_command_scaled, m4_command_scaled));

  if (max_val > 1.0f)
  {
    float scale = 1.0f / max_val; // scaling factor
    m1_command_scaled *= scale;
    m2_command_scaled *= scale;
    m3_command_scaled *= scale;
    m4_command_scaled *= scale;
  }

  // Constrain outputs (final safety clamp, although it should never be more than 1 since they are all scaled)
  m1_command_scaled = constrain(m1_command_scaled, 0.0f, 1.0f);
  m2_command_scaled = constrain(m2_command_scaled, 0.0f, 1.0f);
  m3_command_scaled = constrain(m3_command_scaled, 0.0f, 1.0f);
  m4_command_scaled = constrain(m4_command_scaled, 0.0f, 1.0f);

  if (altitudeData.altitude < .5f)
    ConditionCommands(); // Smooth ramping of motor speeds when launching and landing to prevent hard hits.
}

inline void ConditionCommands()
{
  // This prevents sudden motor changes that could jolt the drone.
  // The ramp step controls how quickly motor commands can change per loop iteration.

  if (m1_command_scaled_prev > m1_command_scaled + motor_ramp_step)
  {
    m1_command_scaled = m1_command_scaled_prev - motor_ramp_step;
  }
  else if (m1_command_scaled_prev < m1_command_scaled - motor_ramp_step)
  {
    m1_command_scaled = m1_command_scaled_prev + motor_ramp_step;
  }

  if (m2_command_scaled_prev > m2_command_scaled + motor_ramp_step)
  {
    m2_command_scaled = m2_command_scaled_prev - motor_ramp_step;
  }
  else if (m2_command_scaled_prev < m2_command_scaled - motor_ramp_step)
  {
    m2_command_scaled = m2_command_scaled_prev + motor_ramp_step;
  }

  if (m3_command_scaled_prev > m3_command_scaled + motor_ramp_step)
  {
    m3_command_scaled = m3_command_scaled_prev - motor_ramp_step;
  }
  else if (m3_command_scaled_prev < m3_command_scaled - motor_ramp_step)
  {
    m3_command_scaled = m3_command_scaled_prev + motor_ramp_step;
  }

  if (m4_command_scaled_prev > m4_command_scaled + motor_ramp_step)
  {
    m4_command_scaled = m4_command_scaled_prev - motor_ramp_step;
  }
  else if (m4_command_scaled_prev < m4_command_scaled - motor_ramp_step)
  {
    m4_command_scaled = m4_command_scaled_prev + motor_ramp_step;
  }

  m1_command_scaled_prev = m1_command_scaled;
  m2_command_scaled_prev = m2_command_scaled;
  m3_command_scaled_prev = m3_command_scaled;
  m4_command_scaled_prev = m4_command_scaled;
}

void waitForRadio()
{
  // Wait until the throttle is turned down and throttlecut switch is flipped down before allowing anything else to happen.
  // Using PWM_throttle_prev so that it is seeded
  PWM_throttle_prev = getRadioPWM(throttlePin, 1520);
  PWM_ThrottleCutSwitch = getRadioPWM(throttleCutSwitchPin, 2000);

  while ((PWM_throttle_prev < 1400 || PWM_throttle_prev > 1600 || PWM_ThrottleCutSwitch > 1500) && !EASYCHAIR)
  { // Throttle Cut switch pulled down (flymode) is 2000. Up is 1000 (cut)
    // if the throttle is up or the cut switch is set to Fly (pulled down), then wait until the switches are set.
    PWM_throttle_prev = getRadioPWM(throttlePin, 2000);              // keep it stuck in the loop if it failsafes
    PWM_ThrottleCutSwitch = getRadioPWM(throttleCutSwitchPin, 1520); // keep it stuck in the loop if it failsafes
    tick();
    getIMUdata();
    tock(); // This will warm up the Madgwick while we wait for them to turn on the radio.
    vTaskDelay(1);
  }

  // Seed the rest of the controls' prev(ious) values.
  // This is so the filters don't have to climb up to their true values from zero at their startup
  // which throws off desired values for a brief time.
  PWM_roll_prev = getRadioPWM(rollPin, 1500);
  PWM_pitch_prev = getRadioPWM(upDownPin, 1500);
  PWM_yaw_prev = getRadioPWM(ruddPin, 1500);
}

void setupMotorCommunication()
{
  // Initialize all pins

  bool checker = ledcSetClockSource(LEDC_USE_XTAL_CLK);
  checker = ledcAttach(m1Pin, ESC_FREQ_HZ, ESC_RESOLUTION);
  checker = ledcAttach(m2Pin, ESC_FREQ_HZ, ESC_RESOLUTION);
  checker = ledcAttach(m3Pin, ESC_FREQ_HZ, ESC_RESOLUTION);
  checker = ledcAttach(m4Pin, ESC_FREQ_HZ, ESC_RESOLUTION);

  Serial.println("Set up ledc for Motor Communication.");

  calibrateESCs();

  m1_command_PWM = 1000; // Will send the default for motor stopped for Simonk firmware
  m2_command_PWM = 1000;
  m3_command_PWM = 1000;
  m4_command_PWM = 1000;

  // Just in case, make sure the variables that hold radio values are safe.
  setToFailsafe();
  PWM_throttle = PWM_throttle_zero; // zero may not necessarily be the failsafe, but on startup we want zero.
}

void ESCWriteMicroseconds(int gpio, int us)
{
  int duty = (int)(us * ESC_US_TO_DUTY);
  ledcWrite(gpio, duty);
}

void setupBatteryMonitor()
{
  buzzer_millis = millis();
  pinMode(BUZZER_PIN, OUTPUT);
  bool checker = ledcAttachChannel(BUZZER_PIN, 1000, 8, 7);
  if (!checker)
    Serial.println("Failed to set up Buzzer.");
}

void playStartSong()
{
  // Melody (first bar of Danger Zone, simplified)
  int melody[] = {Note_E4, Note_FS4, Note_A4, Note_B4, Note_A4, Note_E4, Note_FS4, Note_E4, Note_FS4, Note_E4, Note_FS4, Note_E4, Note_FS4};

  // Durations (ms) — quarter = 400ms, half = 800ms
  int noteDurations[] = {200, 200, 200, 580, 690, 200, 400, 200, 400, 200, 400, 200, 1200};

  for (int i = 0; i < 13; i++)
  {
    int duration = noteDurations[i];
    ledcWriteTone(BUZZER_PIN, melody[i]);

    delay(duration * .8); // add pause between notes
    ledcWriteTone(BUZZER_PIN, 0);
    delay(duration * .2);
  }
}

void playReadySong()
{
  // Melody (first bar of Danger Zone, simplified)
  int melody[] = {Note_FS4, Note_FS4, Note_FS4};

  // Durations (ms) — quarter = 400ms, half = 800ms
  int noteDurations[] = {200, 200, 200};

  for (int i = 0; i < 3; i++)
  {
    int duration = noteDurations[i];
    ledcWriteTone(BUZZER_PIN, melody[i] * 3);
    delay(duration * .8); // add pause between notes
    ledcWriteTone(BUZZER_PIN, 0);
    delay(400);
  }
}
void playNope()
{
  // Melody (first bar of Danger Zone, simplified)
  int melody[] = {Note_FS4, Note_FS4};

  // Durations (ms) — quarter = 400ms, half = 800ms
  int noteDurations[] = {50, 50};

  for (int i = 0; i < 2; i++)
  {
    int duration = noteDurations[i];
    ledcWriteTone(BUZZER_PIN, melody[i] / 3);
    delay(duration * .8); // add pause between notes
    ledcWriteTone(BUZZER_PIN, 0);
    delay(duration * .8);
  }
}
inline void loopBuzzer()
{ // this monitors the battery.  the lower it gets, the faster it beeps.
  // disabling for now until we get the wiring correct in the next version
  static unsigned long buzzer_spacing = 30000;
  unsigned long myTime = millis();

  if (!beeping)
  {
    if (myTime - buzzer_millis > (buzzer_spacing))
    {
      ledcWriteTone(BUZZER_PIN, Note_E4);
      beeping = true;
      buzzer_millis = myTime;
    }
  }
  else
  {
    if (myTime - buzzer_millis > 250)
    {
      beeping = false;
      buzzer_millis = myTime;
      ledcWriteTone(BUZZER_PIN, 0);
    }
  }

  if (myTime > next_voltage_check)
  {
    next_voltage_check = myTime + 30000; // checkvoltage once every 10 seconds versus every loop.
    buzzer_spacing = 30000;
    /*
    batteryVoltage = analogRead(BATTERY_PIN);
    if (BATTERYTYPE == 14.8) {
      // based on 330K and 51K voltage divider that takes 16.8V to 2.25V
      if (batteryVoltage = 0) buzzer_spacing = 40000;
      else if (batteryVoltage > 1457) buzzer_spacing = 40000;
      else if (batteryVoltage > 1440) buzzer_spacing = 30000;
      else if (batteryVoltage > 1400) buzzer_spacing = 20000;
      else if (batteryVoltage > 1350) buzzer_spacing = 2000;
      else if (batteryVoltage > 1301) buzzer_spacing = 500;
      else buzzer_spacing = 100;
    } else {
      // Depends on your resistors and battery choice
      if (batteryVoltage = 0) buzzer_spacing = 40000;
      else if (batteryVoltage > 1457) buzzer_spacing = 40000;
      else if (batteryVoltage > 1440) buzzer_spacing = 30000;
      else if (batteryVoltage > 1400) buzzer_spacing = 20000;
      else if (batteryVoltage > 1350) buzzer_spacing = 2000;
      else if (batteryVoltage > 1301) buzzer_spacing = 500;
      else buzzer_spacing = 100;
    }
    */
  }
}

void setupSerial()
{
  Serial.begin(2000000);
  pinMode(44, INPUT);
  Serial1.begin(9600, SERIAL_8N1, 44, 43);
  delay(100);
  Serial.println("USB and Pin Serial enabled...");
  Serial1.println("USB and Pin Serial enabled...");
  delay(100);
}

float getCalcedVoltage()
{
  // For the battery voltage calc, you can use ohm's law on your chosen voltage divider resistors and get the voltage ratio of the 12bit ADC.
  // I simply recorded values against fed voltages from my bench power supply and fit
  // a line.  Good ole' y=mx+b.

  return (((float)batteryVoltage * 0.0397) - 3.6127);
}

void setupIMU()
{
  // DESCRIPTION: Initialize IMU and set to 2000Hz gyro and 2000Hz accel output rates.
  pinMode(IMU_INT_PIN, INPUT);
  pinMode(SPI_CS, OUTPUT);
  gpio_set_level((gpio_num_t)SPI_CS, 1);              // Deselect device
  IMUSPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI, SPI_CS); // SCLK, MISO, MOSI, CS
  delay(100);
  writeRegister(0x76, 0x0);            // Work with Bank 0
  writeRegister(0x11, 0x01);           // Soft Reset
  delay(200);                          // give time for reset
  uint8_t whoami = readRegister(0x75); // WHO_AM_I register should return 0x3B
  delay(100);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoami, HEX);
  if (whoami == 0x3B)
  {
    Serial.println("IMU SPI communication successful.");
  }
  else
  {
    Serial.println("IMU SPI communication failed. Check wiring, CS pin, or SPI settings. Trying again.");
    return;
  }

  writeRegister(0x4E, 0x1F);
  delay(10); // Turn the accelerator and gyro in Low Noise mode and power the temperature sensor.
  writeRegister(0x13, 0x05);
  delay(10); // <2ns SPI Slew Rate
  writeRegister(0x16, 0x00);
  delay(10); // Bypass FIFO (don't queue readings)
  writeRegister(0x4F, 0x05);
  delay(10); // Gyro 2000 dps at 2kHz updates downsampled from 32kHz
  writeRegister(0x50, 0x05);
  delay(10); // Accel 32g and 2Khz updates
  writeRegister(0x65, 0x08);
  delay(10); // Don't bother interupting after reseet. Interrupt on 1 when data is available.
  writeRegister(0x76, 0x01);
  delay(10); // Work with Bank 1
  writeRegister(0x03, 0x00);
  delay(10); // Bank 1 register: Use every axis measurement including Z (default has it off)
  writeRegister(0x7A, 0x02);
  delay(10); // Bank 1 register: 4 wire SPI mode
  writeRegister(0x76, 0x0);
  delay(10); // Work with Bank 0

  MadgwickInit(); // Initialize Madgwick filter with current accelerometer values
  Serial.println("IMU config complete.");
}

inline void MadgwickInit()
{
  // Initialize Madgwick filter with current accelerometer values
  // Cuts down on thhe time it takes for Madgwick to converge on a stable attitude.
  while (gpio_get_level((gpio_num_t)IMU_INT_PIN))
    delay(10); // Wait for it to pull low to indicate data ready

  // Burst read: 6 accel + 6 gyro bytes in one transaction
  uint8_t buf[12];
  readBlock(ACCEL_DATA_X1, buf, 12);

  int16_t rawAx = (buf[0] << 8) | buf[1];
  int16_t rawAy = (buf[2] << 8) | buf[3];
  int16_t rawAz = (buf[4] << 8) | buf[5];

  int16_t rawGx = (buf[6] << 8) | buf[7];
  int16_t rawGy = (buf[8] << 8) | buf[9];
  int16_t rawGz = (buf[10] << 8) | buf[11];

  // Apply offsets + scaling
  float ax = (rawAx - AccErrorX) * G_PER_LSB;
  float ay = (rawAy - AccErrorY) * G_PER_LSB;
  float az = (rawAz - AccErrorZ) * G_PER_LSB;

  float norm = sqrtf(ax * ax + ay * ay + az * az);

  if (norm == 0.0f)
    return; // invalid accel
  ax /= norm;
  ay /= norm;
  az /= norm;

  // Compute roll and pitch from gravity
  float roll = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  float yaw = 0.0f; // no magnetometer, assume heading = 0

  // Convert to quaternion
  float cy = cosf(yaw * 0.5f);
  float sy = sinf(yaw * 0.5f);
  float cp = cosf(pitch * 0.5f);
  float sp = sinf(pitch * 0.5f);
  float cr = cosf(roll * 0.5f);
  float sr = sinf(roll * 0.5f);

  q0 = cr * cp * cy + sr * sp * sy;
  q1 = sr * cp * cy - cr * sp * sy;
  q2 = cr * sp * cy + sr * cp * sy;
  q3 = cr * cp * sy - sr * sp * cy;

  // Normalize quaternion
  norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= norm;
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;

  // Cache Euler angles for consistency
  roll_IMU = roll * 57.29577951f; // RAD2DEG
  pitch_IMU = -pitch * 57.29577951f; // Flip pitch axis to match NASA
  yaw_IMU = -yaw * 57.29577951f; // Flip yaw axis to match NASA
}

inline void getIMUdata()
{
  // Fast GPIO read (ESP32 example)
  if (gpio_get_level((gpio_num_t)IMU_INT_PIN))
    return; // active LOW

  // Burst read: 6 accel + 6 gyro bytes in one transaction
  uint8_t buf[12];
  readBlock(ACCEL_DATA_X1, buf, 12);

  int16_t rawAx = (buf[0] << 8) | buf[1];
  int16_t rawAy = (buf[2] << 8) | buf[3];
  int16_t rawAz = (buf[4] << 8) | buf[5];

  int16_t rawGx = (buf[6] << 8) | buf[7];
  int16_t rawGy = (buf[8] << 8) | buf[9];
  int16_t rawGz = (buf[10] << 8) | buf[11];

  // Apply offsets + scaling
  float ax = (rawAx - AccErrorX) * G_PER_LSB;
  float ay = (rawAy - AccErrorY) * G_PER_LSB;
  float az = (rawAz - AccErrorZ) * G_PER_LSB;

  float gx = (rawGx - GyroErrorX) * DPS_PER_LSB;
  float gy = (rawGy - GyroErrorY) * DPS_PER_LSB;
  float gz = (rawGz - GyroErrorZ) * DPS_PER_LSB;

  // Filter (optimized form)
  AccX = AccX_prev + Accel_filter * (ax - AccX_prev);
  AccY = AccY_prev + Accel_filter * (ay - AccY_prev);
  AccZ = AccZ_prev + Accel_filter * (az - AccZ_prev);

  GyroX = GyroX_prev + Gyro_filter * (gx - GyroX_prev);
  GyroY = GyroY_prev + Gyro_filter * (gy - GyroY_prev);
  GyroZ = GyroZ_prev + Gyro_filter * (gz - GyroZ_prev);

  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;

  // Timing
  unsigned long currentMicros = micros();
  unsigned long tempMicros = (currentMicros - lastMadgwickUpdateMicros);
  if (tempMicros>=0 && tempMicros<1000) madDeltaTime = tempMicros * USEC_TO_SEC; else madDeltaTime=0.0005f;
  lastMadgwickUpdateMicros = currentMicros;

  // Madgwick update
  Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ);
}

void setupPPM()
{
  radio.begin(PPM_PIN, 1000000, 8, 2100); // GPIO Pin Number, 1MZ (1uS ticks) to track PPM pulse width, # of pulses in PPM, duration of sync high pulse
}

inline void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az)
{
  // Precomputed constants
  constexpr float DEG2RAD = 0.01745329252f; // π/180
  constexpr float RAD2DEG = 57.29577951f;   // 180/π

  // Convert gyro to rad/s
  gx *= DEG2RAD;
  gy *= DEG2RAD;
  gz *= DEG2RAD;

  // Rate of change of quaternion
  const float half = 0.5f;
  qDot1 = half * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = half * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = half * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = half * (q0 * gz + q1 * gy - q2 * gx);

  // Accelerometer valid?
  if (!(ax == 0.0f && ay == 0.0f && az == 0.0f))
  {
    // Normalize accelerometer
    float norm = ax * ax + ay * ay + az * az;
    float recipNorm = 1.0f / sqrtf(norm);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Precompute reused terms
    float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
    float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
    float _4q0 = 2.0f * _2q0, _4q1 = 2.0f * _2q1, _4q2 = 2.0f * _2q2;
    float _8q1 = 2.0f * _4q1, _8q2 = 2.0f * _4q2;

    // Gradient descent step
    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  // Integrate quaternion
  q0 += qDot1 * madDeltaTime;
  q1 += qDot2 * madDeltaTime;
  q2 += qDot3 * madDeltaTime;
  q3 += qDot4 * madDeltaTime;

  // Normalize quaternion
  float norm = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
  recipNorm = 1.0f / sqrtf(norm);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  // Compute angles (cache reused terms)
  float twoq0q1 = 2.0f * (q0 * q1);
  float twoq2q3 = 2.0f * (q2 * q3);
  float twoq0q2 = 2.0f * (q0 * q2);
  float twoq1q3 = 2.0f * (q1 * q3);
  float twoq0q3 = 2.0f * (q0 * q3);
  float twoq1q2 = 2.0f * (q1 * q2);

  roll_IMU = atan2f(twoq0q1 + twoq2q3, 1.0f - 2.0f * (q1q1 + q2q2)) * RAD2DEG;
  pitch_IMU = -asinf(twoq0q2 - twoq1q3) * RAD2DEG; // flip pitch axis to match NASA
  yaw_IMU = -atan2f(twoq0q3 + twoq1q2, 1.0f - 2.0f * (q2q2 + q3q3)) * RAD2DEG; // flip yaw axis to match NASA
}

void calculateIMUError()
{
  Serial.println("Calculating IMU Error with 30000 filtered iterations. Please stand by...");

  constexpr float ACC_LSB_PER_G = 1024.0f; // ±32 g
  constexpr float G_PER_LSB = 1.0f / ACC_LSB_PER_G;
  constexpr float GYRO_LSB_PER_DPS = 16.4f; // 2000 dps
  constexpr float DPS_PER_LSB = 1.0f / GYRO_LSB_PER_DPS;

  // Filtering parameters
  constexpr float ALPHA = 0.05f;                 // Low‑pass filter coefficient (0.0–1.0)
  constexpr float OUTLIER_THRESHOLD_ACC = 0.25f; // g
  constexpr float OUTLIER_THRESHOLD_GYRO = 5.0f; // deg/s

  float fAccX = 0, fAccY = 0, fAccZ = 0;
  float fGyroX = 0, fGyroY = 0, fGyroZ = 0;

  AccErrorX = AccErrorY = AccErrorZ = 0;
  GyroErrorX = GyroErrorY = GyroErrorZ = 0;

  int c = 0;
  while (c < 30000)
  {
    while (digitalRead(IMU_INT_PIN) == HIGH)
      delay(1);

    float rawAccX = read16(ACCEL_DATA_X1) * G_PER_LSB;
    float rawAccY = read16(ACCEL_DATA_X1 + 2) * G_PER_LSB;
    float rawAccZ = read16(ACCEL_DATA_X1 + 4) * G_PER_LSB;

    float rawGyroX = read16(GYRO_DATA_X1) * DPS_PER_LSB;
    float rawGyroY = read16(GYRO_DATA_X1 + 2) * DPS_PER_LSB;
    float rawGyroZ = read16(GYRO_DATA_X1 + 4) * DPS_PER_LSB;
    if (c == 0)
    {
      fAccX = rawAccX;
      fAccY = rawAccY;
      fAccZ = rawAccZ;
      fGyroX = rawGyroX;
      fGyroY = rawGyroY;
      fGyroZ = rawGyroZ;
    }
    else
    {
      // -------- OUTLIER REJECTION --------
      if (fabs(rawAccX - fAccX) < OUTLIER_THRESHOLD_ACC)
        fAccX = fAccX + ALPHA * (rawAccX - fAccX);
      if (fabs(rawAccY - fAccY) < OUTLIER_THRESHOLD_ACC)
        fAccY = fAccY + ALPHA * (rawAccY - fAccY);
      if (fabs(rawAccZ - fAccZ) < OUTLIER_THRESHOLD_ACC)
        fAccZ = fAccZ + ALPHA * (rawAccZ - fAccZ);

      if (fabs(rawGyroX - fGyroX) < OUTLIER_THRESHOLD_GYRO)
        fGyroX = fGyroX + ALPHA * (rawGyroX - fGyroX);
      if (fabs(rawGyroY - fGyroY) < OUTLIER_THRESHOLD_GYRO)
        fGyroY = fGyroY + ALPHA * (rawGyroY - fGyroY);
      if (fabs(rawGyroZ - fGyroZ) < OUTLIER_THRESHOLD_GYRO)
        fGyroZ = fGyroZ + ALPHA * (rawGyroZ - fGyroZ);
    }
    // -------- ACCUMULATE FILTERED VALUES --------
    AccErrorX += fAccX;
    AccErrorY += fAccY;
    AccErrorZ += fAccZ;

    GyroErrorX += fGyroX;
    GyroErrorY += fGyroY;
    GyroErrorZ += fGyroZ;

    c++;

    if (c % 500 == 0)
      Serial.println(String(30000 - c));
  }

  // -------- FINAL AVERAGE --------
  AccErrorX /= c;
  AccErrorY /= c;
  AccErrorZ = (AccErrorZ / c) - 1.0f; // subtract gravity

  GyroErrorX /= c;
  GyroErrorY /= c;
  GyroErrorZ /= c;

  Serial.print("static float AccErrorX = ");
  Serial.print(AccErrorX);
  Serial.println(";");
  Serial.print("static float AccErrorY = ");
  Serial.print(AccErrorY);
  Serial.println(";");
  Serial.print("static float AccErrorZ = ");
  Serial.print(AccErrorZ);
  Serial.println(";");

  Serial.print("static float GyroErrorX = ");
  Serial.print(GyroErrorX);
  Serial.println(";");
  Serial.print("static float GyroErrorY = ");
  Serial.print(GyroErrorY);
  Serial.println(";");
  Serial.print("static float GyroErrorZ = ");
  Serial.print(GyroErrorZ);
  Serial.println(";");

  Serial.println("Paste these values in user specified variables section and comment out calculateIMUError() in void setup.");
  halt();
}

inline void getDesiredAnglesAndThrottleScaledToOne()
{
  // DESCRIPTION: Normalizes desired control values to appropriate values
  /*
   * Updates the desired state variables throttle_desired, roll_des, pitch_des, and yaw_des. These are computed by using the raw
   * RC pwm commands and scaling them to be within our limits defined in setup. throttle_desired stays within 0 to 1 range.
   * roll_des and pitch_des are scaled to be within max roll/pitch amount in degrees
   * yaw_des is scaled to be within max yaw in degrees/sec.
   */

  throttle_desired = (PWM_throttle - 1500.0f) / 500.0f;   // Between  0 and 1 because anything under 1500 will be set to 1500 for now.
  roll_des = (PWM_roll - 1500.0f + trimRoll) / 500.0f;    // Between -1 and 1
  pitch_des = (PWM_pitch - 1500.0f + trimPitch) / 500.0f; // Between -1 and 1
  yaw_des = (PWM_yaw - 1500.0f + trimYaw) / 500.0f;       // Between -1 and 1

  // Constrain within normalized bounds
  throttle_desired = constrain(throttle_desired, 0.0f, 1.0f); // Between 0 and 1
  roll_des = constrain(roll_des, -1.0f, 1.0f) * maxRoll;      // Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0f, 1.0f) * maxPitch;   // Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0f, 1.0f) * maxYaw;         // Between -maxYaw and +maxYaw
}

void resetAllTiming()
{
  // Once flicked back into flight mode, this will set all frequencies to the same start light just incase there was drift.
  resetTimers = false;
  unsigned long currentMicros = micros();
  lastPIDmicros = currentMicros;            // Used for Integral windup
  lastMadgwickUpdateMicros = currentMicros; // Used for Madgwick delta time calculations
  PIDCounter = 0;                           // using ticks to keep everything in sync
  ESCWriteCounter = 0;                      // using ticks to keep everything in sync
}

void PIDControlCalcs()
{
  // --- Roll ---
  static float integral_rate_roll = 0.0f;
  static float integral_rate_pitch = 0.0f;
  static float integral_rate_yaw = 0.0f;
  static float prevRateErrorRoll = 0.0f;
  static float prevRateErrorYaw = 0.0f;
  static float prevRateErrorPitch = 0.0f;
  float p = GyroX; // roll rate 
  float q = GyroY; // pitch rate (nose up positive) - pitch is already minused to correct physical orientation to NASA rules
  float r = GyroZ; // yaw rate (nose right positive) - yaw is already minused to correct physical orientation to NASA rules
  
  if (PWM_throttle < 1520) // Reset the control if on the ground. This prevents integral windup and sudden jumps on takeoff.
  {
    integral_rate_roll = integral_rate_pitch = integral_rate_yaw = 0;
    prevRateErrorRoll = prevRateErrorPitch = prevRateErrorYaw = 0;
    roll_PID = pitch_PID = yaw_PID = 0;
    return;
  }

  // --- Roll ---
  float rateErrorRoll = desiredRateRoll - p;
  integral_rate_roll += rateErrorRoll * madDeltaTime;
  integral_rate_roll = constrain(integral_rate_roll, -i_limit_rate, i_limit_rate); // Think of this as the tracked energy required to achieve motor torque

  float derivative_roll = p;
  roll_PID = Kp_roll_rate * rateErrorRoll +
             Ki_roll_rate * integral_rate_roll -
             Kd_roll_rate * derivative_roll;
  prevRateErrorRoll = rateErrorRoll;

  // --- Pitch ---
  float rateErrorPitch = desiredRatePitch - q;
  integral_rate_pitch += rateErrorPitch * madDeltaTime;
  integral_rate_pitch = constrain(integral_rate_pitch, -i_limit_rate, i_limit_rate);

  float derivative_pitch = q;
  pitch_PID = Kp_pitch_rate * rateErrorPitch +
              Ki_pitch_rate * integral_rate_pitch -
              Kd_pitch_rate * derivative_pitch;
  prevRateErrorPitch = rateErrorPitch;

  // --- Yaw ---
  float rateErrorYaw = yaw_des - r;
  integral_rate_yaw += rateErrorYaw * madDeltaTime;
  integral_rate_yaw = constrain(integral_rate_yaw, -i_limit_rate, i_limit_rate);

  float derivative_yaw = r;
  yaw_PID = Kp_yaw_rate * rateErrorYaw +
            Ki_yaw_rate * integral_rate_yaw -
            Kd_yaw_rate * derivative_yaw;
  prevRateErrorYaw = rateErrorYaw;
  AngleLoopCalcs(); // Get the new desired angular rates based on current angle versus desired angle
}

void AngleLoopCalcs()
{
  static unsigned long lastTimeMicros = micros();
  static unsigned long dt;

  if (++PIDCounter < (INNER_LOOP_FREQUENCY / PID_FREQ_HZ))
    return;
  PIDCounter = 0;

  dt = micros() - lastTimeMicros;
  lastTimeMicros = micros();

  // --- Roll ---
  float angleErrorRoll = roll_des - roll_IMU;
  integral_roll += angleErrorRoll * dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit);

  desiredRateRoll = Kp_roll_angle * angleErrorRoll +
                    Ki_roll_angle * integral_roll;
  desiredRateRoll = constrain(desiredRateRoll, -rollLimits.maxRate, rollLimits.maxRate);

  // --- Pitch ---
  float angleErrorPitch = pitch_des - pitch_IMU;
  integral_pitch += angleErrorPitch * dt;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit);

  desiredRatePitch = Kp_pitch_angle * angleErrorPitch +
                     Ki_pitch_angle * integral_pitch;
  desiredRatePitch = constrain(desiredRatePitch, -pitchLimits.maxRate, pitchLimits.maxRate);

  // --- Yaw setpoint comes directly from the stick --- //
}

inline void getRadioStickValues()
{
  static int throttleOverrideCounter = 0;
  static bool ceilingOverride = false;
  static float integralAltitudeRate =0;
  
  // Precomputed constants
  constexpr int CEILING_PERIOD = 2 * INNER_LOOP_FREQUENCY / ALT_FREQ_HZ; // Get at least 2 barameter samples before overriding by barometer
  constexpr int ASCENT_PERIOD = 2 * INNER_LOOP_FREQUENCY / ALT_FREQ_HZ;  // Get at least 2 barometer samples before overriding by barometer
  constexpr float UP_COEFF = 0.9f;                                       // faster up
  constexpr float DOWN_COEFF = 0.03f;                                    // slower down

  // Read radio PWM
  PWM_throttle = getRadioPWM(throttlePin, 1000);
  PWM_roll = getRadioPWM(rollPin, 1500);
  PWM_pitch = getRadioPWM(upDownPin, 1500);
  PWM_yaw = getRadioPWM(ruddPin, 1500);
  PWM_ThrottleCutSwitch = getRadioPWM(SwitchA, 2000);
  PWM_Failsafed = getRadioPWM(SwitchB, 2000);
  if (altitudeData.altitude>altitudeData.ceiling) ceilingOverride = true;
  
  // Radio Receiver or Switch B is in Failsafe Mode - Land safely
  if (PWM_Failsafed > 1500||ceilingOverride)
  {
    if (PWM_Failsafed > 1500) ceilingOverride = false; // If you hit the ceiling, it will descend, but you can cancel it out after under the ceiling by flipping the landing switch.
    if (altitudeData.altitude < 0.5f)
    {
      killMotors();
    }
    else
    {
      if (++throttleOverrideCounter>=INNER_LOOP_FREQUENCY/ALT_FREQ_HZ) // loop at the same frequency as the altitude checks
      { 
        throttleOverrideCounter = 0;
        float rateError;
        if (altitudeData.altitude>12.0f)
          rateError = -0.5f - altitudeData.rateFPS; // Descent rate is faster if above 12 feet altitude
        else if (altitudeData.altitude>6.0f)
          rateError = -0.3f - altitudeData.rateFPS; // Descent rate is slower once between 6 and 12 feet.
        else
          rateError = -0.1f - altitudeData.rateFPS; // Descent rate is slower once under feet for a slow landing.
        integralAltitudeRate += rateError*.01f;
        integralAltitudeRate = constrain(integralAltitudeRate,-200,200);
        PWM_throttle = failsafeThrottlePWM + altitudeData.kp_altitude_rate*rateError + altitudeData.ki_altitude_rate * integralAltitudeRate;
        PWM_throttle = constrain(PWM_throttle,1500,2000);
        if (PWM_throttle<=1500) killMotors();
      }
      else PWM_throttle=PWM_throttle_prev;
    }
  }
  else
  {
    if (flying)
    {
      if (PWM_throttle > highestThrottlePWM)
        highestThrottlePWM = PWM_throttle;
      else if (PWM_throttle < lowestThrottlePWM)
        lowestThrottlePWM = PWM_throttle;
    }
    // Normal smoothing
    throttleOverrideCounter = 0;
    float coeff = (PWM_throttle > PWM_throttle_prev) ? UP_COEFF : DOWN_COEFF;
    PWM_throttle = PWM_throttle_prev + coeff * (PWM_throttle - PWM_throttle_prev);
  }
  // Bottom limit
  if (PWM_throttle < 1500)
    PWM_throttle = 1500;

  // Stick dampening (optimized form)
  PWM_roll = PWM_roll_prev + stick_dampener * (PWM_roll - PWM_roll_prev);
  PWM_pitch = PWM_pitch_prev + stick_dampener * (PWM_pitch - PWM_pitch_prev);
  PWM_yaw = PWM_yaw_prev + stick_dampener * (PWM_yaw - PWM_yaw_prev);

  // Update prevs
  PWM_throttle_prev = PWM_throttle;
  PWM_roll_prev = PWM_roll;
  PWM_pitch_prev = PWM_pitch;
  PWM_yaw_prev = PWM_yaw;
}

inline void setToFailsafe()
{
  PWM_throttle = failsafeThrottlePWM;
  PWM_roll = PWM_roll_fs;
  PWM_pitch = PWM_pitch_fs;
  PWM_yaw = PWM_yaw_fs;
  PWM_ThrottleCutSwitch = PWM_ThrottleCutSwitch_fs;
}

inline int16_t clamp(int16_t val, int16_t min, int16_t max)
{
  return (val < min) ? min : (val > max) ? max
                                         : val;
}

inline void motorPipeline()
{
  ESCWriteCounter++;

  if (ESCWriteCounter < (INNER_LOOP_FREQUENCY / MOTOR_FREQ_HZ))
    return;

  ESCWriteCounter = 0;

  controlMixer();            // Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  scaleMotorCommandsToPWM(); // Scales motor commands from 0-1 to PWM
  throttleCut();             // Directly sets motor commands to off based on channel 5 being switched
  commandMotors();           // Sends command pulses to each ESC pin to drive the motors
}

inline void commandMotors()
{
  // DESCRIPTION: Send pulses to motor pins
  if (EASYCHAIR)
  {
    ESCWriteMicroseconds(m1Pin, 1000);
    ESCWriteMicroseconds(m2Pin, 1000);
    ESCWriteMicroseconds(m3Pin, 1000);
    ESCWriteMicroseconds(m4Pin, 1000);
  }
  else
  {
    ESCWriteMicroseconds(m1Pin, m1_command_PWM);
    ESCWriteMicroseconds(m2Pin, m2_command_PWM);
    ESCWriteMicroseconds(m3Pin, m3_command_PWM);
    ESCWriteMicroseconds(m4Pin, m4_command_PWM);
  }
}

inline void scaleMotorCommandsToPWM()
{
  // DESCRIPTION: Scale normalized actuator commands to values for ESC protocol
  /*
   * The actual pulse width is set at the servo attach.
   */
  // Scale to Servo PWM 1000-2000 microseconds for stop to full speed.  No need to constrain since mx_command_scaled already is.
  m1_command_PWM = m1_command_scaled * 1000 + 1000;
  m2_command_PWM = m2_command_scaled * 1000 + 1000;
  m3_command_PWM = m3_command_scaled * 1000 + 1000;
  m4_command_PWM = m4_command_scaled * 1000 + 1000;
}

inline void calibrateESCs()
{
  // DESCRIPTION: Used in void setup() to allow standard ESC calibration procedure with the radio to take place.
  // If the Throttle kill switch is in the up position, then skip calibration. To calibrate, turn on the
  // radio and switch the kill switch down and then powerup the drone until it does the beep sequence.  It takes around 8 seconds.

  uint32_t raw = radio.getChannel(4);
  Serial.print("Radio Cut PWM: ");
  Serial.println(raw);

  if (getRadioPWM(throttleCutSwitchPin, 2000) > 1500)
    return; // throttle cut switch up, then run the calibration. If off or down, skip it.

  Serial.println("Calibrating ESCs...");
  ESCWriteMicroseconds(m1Pin, 2000);
  ESCWriteMicroseconds(m2Pin, 2000);
  ESCWriteMicroseconds(m3Pin, 2000);
  ESCWriteMicroseconds(m4Pin, 2000);
  delay(2000);
  ESCWriteMicroseconds(m1Pin, 1000);
  ESCWriteMicroseconds(m2Pin, 1000);
  ESCWriteMicroseconds(m3Pin, 1000);
  ESCWriteMicroseconds(m4Pin, 1000);
  delay(1000);
  Serial.println("Calibration complete.");
}

void throttleCut()
{
  // DESCRIPTION: Directly set actuator outputs to minimum value if triggered
  /*
   * Monitors the state of radio command PWM_throttle. This is the last function
   * called before commandMotors() is called so that the last thing checked is if the user is giving permission to command
   * the motors to anything other than minimum value. Safety first.
   */

  if (throttle_is_cut)
  {
    killMotors(); // make sure we keep those motors at 0 if the throttle was cut.

    if (PWM_ThrottleCutSwitch > 1500)
    { // switch is in the down position which means enable flight
      // reset (uncut throttle) only if throttle is down to prevent a jolting suprise
      if (PWM_throttle < 1520 && ++throttleNotCutCounter > 10)
      { // The radio is ready for flight and confirmed not to be just a blip.
        if (PWM_Failsafed > 1500)
        {
          playNope(); // Don't want to accidently start in "land mode", so give the pilot a toot.
          throttleNotCutCounter = 0;
        }
        else
        {
          throttle_is_cut = false;
          throttleNotCutCounter = 0;
          throttleCutCounter = 0;
          flying = false;
          xTaskNotifyGive(altitudeData.bmpTaskHandle); // Notify that we want the altitude to reset to current ground level
          lowestThrottlePWM = 2000;
          highestThrottlePWM = 1500;
          altitudeData.highestAltitude = 0.0f;
          playReadySong();    // This gives us a delay to loop a few times for the other bmpTask altitude variable updates while readying the pilot as well - a win-win strategy./
          MadgwickInit();     // Reset the quarterion based on sitting still.
          resetTimers = true; // This will reset all counters to sync timing on the next tock();
        }
      }
    }
    return;
  }
  else if (PWM_ThrottleCutSwitch < 1300)
  {
    // The switch is in the up position meaning throttle is purposely cut.  ThrottleCutCounter will ensure it is not just a blip that has caused a false cut and drop it from the sky.
    if (++throttleCutCounter > 10)
    {
      throttle_is_cut = true;
      flying = false;
      killMotors();
    }
    return;
  }

  // This attemps to save propellers by not driving motors when it goes full sideways. It is also helpful if it accidently runs in a house to keep it from becoming the Tazmanian devil.
  if (roll_IMU > 75 || roll_IMU < -75 || pitch_IMU > 75 || pitch_IMU < -75)
  {
    killMotors();
    return;
  }

  if (!flying || altitudeData.altitude < 2) // this is tip over protection at take-off
  {
    if (roll_IMU > 15 || roll_IMU < -15 || pitch_IMU > 15 || pitch_IMU < -15)
    {
      // This helps with struggles on take off or it sitting too tilted on the launch pad.
      killMotors();
      return;
    }
  }

  throttleNotCutCounter = 0;
  throttleCutCounter = 0;
}

void killMotors()
{
  // sets the PWM to its lowest value to shut off a motor such as when the throttle cut switch is flipped or it gets too steep of an angle.
  throttle_is_cut = true;
  throttleCutCounter = 10;    // to prevent overflowing float
  m1_command_PWM = 1000;      // This is milliseconds for PWM.  1000 is off. 2000 is full throttle.
  m1_command_scaled_prev = 0; // This is 0 to 1.
  m2_command_PWM = 1000;
  m2_command_scaled_prev = 0;
  m3_command_PWM = 1000;
  m3_command_scaled_prev = 0;
  m4_command_PWM = 1000;
  m4_command_scaled_prev = 0;
}

void tock()
{
  // DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. I have matched
   * it to the Gyro update frequency.  innerLoopMicroseconds is set at the top of the code.
   * about every few seconds, it gets overran, but I don't know why yet on the ESP32-S3.
   */
  // Sit in loop until appropriate time has passed while checking for any WiFi client activity.

  while ((unsigned long)(micros() - tick_time) < innerLoopMicroseconds)
  {
    taskYIELD();
  };

  if (resetTimers)
    resetAllTiming(); // Resets the counters that keep the beat for outer loops
}

void printRadioData()
{
  if (tick_time - print_counter > 10000)
  {
    print_counter = micros();
    Serial.print(F(" CH1: ")); // Roll
    Serial.print(getRadioPWM(1, 24));
    Serial.print(F(" CH2: ")); // Pitch
    Serial.print(getRadioPWM(2, 24));
    Serial.print(F(" CH3: "));
    Serial.print(getRadioPWM(3, 24)); // Throttle
    Serial.print(F(" CH4: "));
    Serial.print(getRadioPWM(4, 24)); // Yaw
    Serial.print(F(" CH5: "));
    Serial.println(getRadioPWM(5, 24));
  }
}

void printDesiredState()
{
  if (tick_time - print_counter > 10000)
  {
    print_counter = micros();
    Serial.print(F("throttle_desired: "));
    Serial.print(throttle_desired);
    Serial.print(F(" roll_des: "));
    Serial.print(roll_des);
    Serial.print(F(" pitch_des: "));
    Serial.print(pitch_des);
    Serial.print(F(" yaw_des: "));
    Serial.println(yaw_des);
  }
}

void printGyroData()
{
  if (tick_time - print_counter > 10000)
  {
    print_counter = micros();
    Serial.print(F("GyroX: "));
    Serial.print(GyroX);
    Serial.print(F(" GyroY: "));
    Serial.print(GyroY);
    Serial.print(F(" GyroZ: "));
    Serial.println(GyroZ);
  }
}

void printAccelData()
{
  if (tick_time - print_counter > 10000)
  {
    print_counter = micros();
    Serial.print(F("AccX: "));
    Serial.print(AccX);
    Serial.print(F(" AccY: "));
    Serial.print(AccY);
    Serial.print(F(" AccZ: "));
    Serial.println(AccZ);
  }
}

void printJSON()
{
  if (tick_time - print_counter > 10000)
  { // Don't go too fast or it slows down the main loop. this is a third of a second.
    print_counter = micros();
    Serial.print(F("{\"roll\": "));
    Serial.print(roll_IMU);
    Serial.print(F(", \"pitch\": "));
    Serial.print(pitch_IMU);
    Serial.print(F(", \"yaw\": "));
    Serial.print(yaw_IMU);
    Serial.print(F(", \"ErrorRoll\": "));
    Serial.print(error_roll);
    Serial.print(F(", \"IntegralRoll\": "));
    Serial.print(integral_roll);
    Serial.print(F(", \"DerivativeRoll\": "));
    Serial.print(derivative_roll);

    Serial.print(F(", \"RollKp\": "));
    Serial.print(Kp_roll_rate);
    Serial.print(F(", \"RollKi\": "));
    Serial.print(Ki_roll_rate);
    Serial.print(F(", \"RollKd\": "));
    Serial.print(Kd_roll_rate);

    Serial.print(F(", \"m1\": "));
    Serial.print(m1_command_PWM);
    Serial.print(F(", \"m2\": "));
    Serial.print(m2_command_PWM);
    Serial.print(F(", \"m3\": "));
    Serial.print(m3_command_PWM);
    Serial.print(F(", \"m4\": "));
    Serial.print(m4_command_PWM);

    Serial.print(F(", \"AccX\": "));
    Serial.print(AccX);
    Serial.print(F(", \"AccY\": "));
    Serial.print(AccY);
    Serial.print(F(", \"AccZ\": "));
    Serial.print(AccZ);

    Serial.print(F(", \"GyroX\": "));
    Serial.print(GyroX);
    Serial.print(F(", \"GyroY\": "));
    Serial.print(GyroY);
    Serial.print(F(", \"GyroZ\": "));
    Serial.print(GyroZ);
    Serial.print(F(", \"RollIMU\": "));
    Serial.print(roll_IMU);
    Serial.print(F(", \"PitchIMU\": "));
    Serial.print(pitch_IMU);
    Serial.print(F(", \"YawIMU\": "));
    Serial.print(yaw_IMU);

    Serial.print(F(", \"ThroDes\": "));
    Serial.print(throttle_desired);
    Serial.print(F(", \"RollDes\": "));
    Serial.print(roll_des);
    Serial.print(F(", \"PitchDes\": "));
    Serial.print(pitch_des);
    Serial.print(F(", \"YawDes\": "));
    Serial.print(yaw_des);
    Serial.print(F(", \"Pitch_PID\": "));
    Serial.print(pitch_PID);
    Serial.print(F(", \"Roll_PID\": "));
    Serial.print(roll_PID);
    Serial.print(F(", \"Yaw_PID\": "));
    Serial.print(yaw_PID);

    Serial.print(F(", \"PWM_throttle\": "));
    Serial.print(PWM_throttle);
    Serial.print(F(", \"PWM_roll\": "));
    Serial.print(PWM_roll);
    Serial.print(F(", \"PWM_pitch\": "));
    Serial.print(PWM_pitch);
    Serial.print(F(", \"PWM_yaw\": "));
    Serial.print(PWM_yaw);
    Serial.print(F(", \"PWM_ThrottleCutSwitch\": "));
    Serial.print(PWM_ThrottleCutSwitch);
    Serial.print(F(", \"PWM_Failsafed\": "));
    Serial.print(PWM_Failsafed);
    
    Serial.print(F(", \"Throttle_is_Cut\": "));
    Serial.print(throttle_is_cut);

    Serial.print(F(", \"Failsafe\": "));
    Serial.print(debugger);

    Serial.print(F(", \"DeltaTime\": "));
    Serial.print(deltaTime * 1000000.0);
    Serial.println("}");
  }
}

void printRollPitchYaw()
{
  if (tick_time - print_counter > 10000)
  {
    print_counter = micros();
    Serial.print(F("roll: "));
    Serial.print(roll_IMU);
    Serial.print(F(" pitch: "));
    Serial.print(pitch_IMU);
    Serial.print(F(" yaw: "));
    Serial.println(yaw_IMU);
  }
}

void printPIDoutput()
{
  if (tick_time - print_counter > 10000)
  {
    print_counter = micros();
    Serial.print(F("roll_PID: "));
    Serial.print(roll_PID);
    Serial.print(F(" pitch_PID: "));
    Serial.print(pitch_PID);
    Serial.print(F(" yaw_PID: "));
    Serial.println(yaw_PID);
  }
}

void printMotorCommands()
{
  if (tick_time - print_counter > 10000)
  {
    print_counter = micros();
    Serial.print(F("m1_command: "));
    Serial.print(m1_command_PWM);
    Serial.print(F(" m2_command: "));
    Serial.print(m2_command_PWM);
    Serial.print(F(" m3_command: "));
    Serial.print(m3_command_PWM);
    Serial.print(F(" m4_command: "));
    Serial.println(m4_command_PWM);
  }
}

void printtock()
{
  if (tick_time - print_counter > 50000)
  {
    print_counter = micros();
    Serial.print(F("deltaTime = "));
    Serial.println(deltaTime * 1000000.0);
  }
}

// ========================================================================================================================//
// PPM routines
int16_t getRadioPWM(int ch_num, int16_t defaultVal)
{
  // DESCRIPTION: Get current radio commands from interrupt routine

  int16_t valPWM = 0;
  valPWM = latestValidChannelValue(ch_num, defaultVal); // 1000-2000 based on PPM routine return range. Return defaultVal if it isn't valid.
  return valPWM;
}

inline int16_t latestValidChannelValue(int channel, int16_t defaultValue) {
  // Capture current time once
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

void beginDroneLoopTask()
{
  // Create the high-priority drone control loop task
  xTaskCreatePinnedToCore(
      loopDrone,         // Task function
      "Drone Loop Task", // Name
      4096,              // Stack size
      NULL,              // Parameters
      2,                 // Priority (higher than BMP task)
      &loopDroneHandle,  // Task handle
      1                  // Pin to core 1, its faster
  );
}

void wifiTask(void *pvParameters)
{
  setupWiFi();
  vTaskDelay(100);
  while (true)
  {
    // Do the sensor read
    if (throttle_is_cut) loopWiFi();
    vTaskDelay(250);
  }
}
void buzzerTask(void *pvParameters)
{
  setupBatteryMonitor();
  vTaskDelay(100);
  while (true)
  {
    // Do the sensor read
    if (throttle_is_cut) loopBuzzer();
    vTaskDelay(500);
  }
}
void setupWiFi()
{
  const char *ssid = "_Rawpter";
  const char *pass = "12345678";

  IPAddress localIP(192, 168, 2, 4);
  IPAddress gateway(192, 168, 2, 4);
  IPAddress subnet(255, 255, 255, 0);

  // Configure AP IP
  WiFi.softAPConfig(localIP, gateway, subnet);

  // Configure AP with fixed channel and WPA2
  while (!WiFi.softAP(ssid, pass, 1, 0, 4))
  {
    Serial.println("AP setup failed, retrying...");
    delay(2000);
  }

  WiFi.setSleep(false);
  vTaskDelay(500);
  server.begin();
  Serial.println("Web server started on port 80.");
  vTaskDelay(100);
}

void loopWiFi()
{
  WiFiClient client = server.available();

  int waitCount = 0;
  while (!client.available()) { // wait up to 3 s
      vTaskDelay(1);
      if (++waitCount>3000) break;
  }

  if (!client.available()) {
      client.stop();
      return;
  }

  Serial.println("Got Wifi Client.");
  // Read request line
  String req = client.readStringUntil('\r');
  if (client.available()) client.readStringUntil('\n'); // consume newline
  Serial.println("Request: " + req);

  // --- iOS captive portal probe ---
  if (req.startsWith("GET /hotspot-detect.html"))
  {
    const char *body = "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>";
    int len = strlen(body);
    Serial.println("Serving iOS.");
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.print("Content-Length: ");
    client.println(len);
    client.println();
    client.print(body);
    Serial.println("Served iOS.");
  }
  // --- Android captive portal probe ---
  else if (req.startsWith("GET /generate_204"))
  {
    // Android expects a 204 No Content response
    client.println("HTTP/1.1 204 No Content");
    client.println("Connection: close");
    client.println("Content-Length: 0");
    client.println();
  }
  else if (req.startsWith("CONNECT "))
  {
    Serial.println("Handling CONNECT request: " + req);

    // Minimal response: tell the client the tunnel is "established"
    client.println("HTTP/1.1 200 Connection Established");
    client.println("Connection: close");
    client.println();
  }

  // --- Favicon requests (avoid clutter) ---
  else if (req.startsWith("GET /favicon.ico"))
  {
    client.println("HTTP/1.1 204 No Content");
    client.println("Connection: close");
    client.println();
  }
  else if (req.startsWith("GET /apple-touch-icon"))
  {
    Serial.println("Ignoring iOS touch icon request.");
    client.println("HTTP/1.1 204 No Content");
    client.println("Connection: close");
    client.println();
  }

  // --- Your form submission ---
  else if (req.startsWith("GET /?"))
  {
    setValuesFromUserForm(req);

    const char *body =
        "<!DOCTYPE html>"
        "<html><head>"
        "<meta http-equiv='refresh' content='0.2;url=http://192.168.2.4/'>"
        "<title>Redirecting</title>"
        "</head>"
        "<body>"
        "<div style='font-size:60pt;'>Settings saved. Refreshing...</div>"
        "</body></html>";

    int len = strlen(body);

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.print("Content-Length: ");
    client.println(len);
    client.println();
    client.print(body);

    client.stop();
    return;   // clean end of request
  }
  // --- Default page (redirect everything else) ---
  else
  {
    Serial.println("Generating the page...");
    GenerateDefaultPage(client);
    Serial.println("Served the page.");
  }

  client.stop();
  taskYIELD();
}

void setValuesFromUserForm(String req)
{
  // Read the full request line (already done in loopWiFi)

  int qIndex = req.indexOf('?');
  int hIndex = req.indexOf("HTTP");
  Serial.print("qIndex: ");
  Serial.println(String(qIndex));
  Serial.print(" hIndex: ");
  Serial.println(String(hIndex));
  if (qIndex == -1 || hIndex == -1)
    return;

  String query = req.substring(qIndex + 1, hIndex - 1); // everything between ? and " HTTP"
  // Split by '&'
  int start = 0;
  while (start < query.length())
  {
    int amp = query.indexOf('&', start);
    if (amp == -1)
      amp = query.length();
    String pair = query.substring(start, amp);
    start = amp + 1;

    int eq = pair.indexOf('=');
    if (eq == -1)
      continue;
    String key = pair.substring(0, eq);
    String value = pair.substring(eq + 1);

    // Decode URL encoding if needed (spaces, %20, etc.)
    value.replace('+', ' ');

    // Assign values
    if (key == "stick_dampener")
      stick_dampener = value.toFloat();
    else if (key == "i_limit_rate")
      i_limit_rate = value.toFloat();
    else if (key == "i_limit")
      i_limit = value.toFloat();
    else if (key == "Accel_filter")
      Accel_filter = value.toFloat();
    else if (key == "Gyro_filter")
      Gyro_filter = value.toFloat();
    else if (key == "B_madgwick")
      B_madgwick = value.toFloat();
    else if (key == "kp_roll_angle")
      Kp_roll_angle = value.toFloat();
    else if (key == "ki_roll_angle")
      Ki_roll_angle = value.toFloat();
    else if (key == "kp_pitch_angle")
      Kp_pitch_angle = value.toFloat();
    else if (key == "ki_pitch_angle")
      Ki_pitch_angle = value.toFloat();
    else if (key == "kp_roll_rate")
      Kp_roll_rate = value.toFloat();
    else if (key == "ki_roll_rate")
      Ki_roll_rate = value.toFloat();
    else if (key == "kd_roll_rate")
      Kd_roll_rate = value.toFloat();
    else if (key == "kp_pitch_rate")
      Kp_pitch_rate = value.toFloat();
    else if (key == "ki_pitch_rate")
      Ki_pitch_rate = value.toFloat();
    else if (key == "kd_pitch_rate")
      Kd_pitch_rate = value.toFloat();
    else if (key == "kp_yaw_rate")
      Kp_yaw_rate = value.toFloat();
    else if (key == "ki_yaw_rate")
      Ki_yaw_rate = value.toFloat();
    else if (key == "kd_yaw_rate")
      Kd_yaw_rate = value.toFloat();
    else if (key == "roll_maxRate")
      rollLimits.maxRate = value.toFloat();
    else if (key == "pitch_maxRate")
      pitchLimits.maxRate = value.toFloat();
    else if (key == "failsafeThrottlePWM")
      failsafeThrottlePWM = value.toInt();
    else if (key == "kp_altitude_rate") altitudeData.kp_altitude_rate = value.toFloat();
    else if (key == "ki_altitude_rate") altitudeData.ki_altitude_rate = value.toFloat();
    else if (key == "ceiling") altitudeData.ceiling = value.toFloat();
    else if (key == "trimPitch")
      trimPitch = value.toFloat();
    else if (key == "trimYaw")
      trimYaw = value.toFloat();
    else if (key == "trimRoll")
      trimRoll = value.toFloat();
    else if (key == "action")
    {
      if (value.indexOf("SAVE TO") != -1) {
          saveParameters();
      }
    }
  }
}

uint8_t readRegister(uint8_t reg)
{
  IMUSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // 1 MHz, Mode 0
  gpio_set_level((gpio_num_t)SPI_CS, 0);
  IMUSPI.transfer(reg | 0x80); // Read bit set
  uint8_t value = IMUSPI.transfer(0x00);
  gpio_set_level((gpio_num_t)SPI_CS, 1);
  IMUSPI.endTransaction();
  return value;
}

int16_t read16(uint8_t reg)
{
  IMUSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // 1 MHz, Mode 0
  gpio_set_level((gpio_num_t)SPI_CS, 0);
  IMUSPI.transfer(reg | 0x80); // Read bit set
  uint8_t high = IMUSPI.transfer(0x00);
  uint8_t low = IMUSPI.transfer(0x00);
  gpio_set_level((gpio_num_t)SPI_CS, 1);
  IMUSPI.endTransaction();
  return (int16_t)((high << 8) | low);
}

inline void readBlock(uint8_t reg, uint8_t *buf, size_t len)
{
  IMUSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // 1 MHz, Mode 0
  // Assert chip select
  gpio_set_level((gpio_num_t)SPI_CS, 0);

  // Send register address with MSB=1 to indicate read
  IMUSPI.transfer(reg | 0x80);

  // Read 'len' bytes
  for (size_t i = 0; i < len; i++)
  {
    buf[i] = IMUSPI.transfer(0x00); // dummy write, read response
  }

  // Deassert chip select
  gpio_set_level((gpio_num_t)SPI_CS, 1);
  IMUSPI.endTransaction();
}

void writeRegister(uint8_t reg, uint8_t value)
{
  IMUSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // 1 MHz, Mode 0
  gpio_set_level((gpio_num_t)SPI_CS, 0);
  IMUSPI.transfer(reg & 0x7F); // Write bit
  IMUSPI.transfer(value);
  gpio_set_level((gpio_num_t)SPI_CS, 1);
  IMUSPI.endTransaction();
}

void saveParameters()
{
  prefs.begin("rawpter", false); // namespace "rawpter", RW mode
  prefs.putInt("fsThrottlePWM", failsafeThrottlePWM);
  prefs.putFloat("ceiling", altitudeData.ceiling;
  prefs.putFloat("kpaltrate", altitudeData.kp_altitude_rate);
  prefs.putFloat("kialtrate", altitudeData.ki_altitude_rate);
  prefs.putFloat("trimPitch", trimPitch);
  prefs.putFloat("trimRoll", trimRoll);
  prefs.putFloat("trimYaw", trimYaw);
  prefs.putFloat("stick_dampener", stick_dampener);
  prefs.putFloat("i_limit", i_limit);
  prefs.putFloat("i_limit_rate", i_limit_rate);
  prefs.putFloat("B_madgwick", B_madgwick);
  prefs.putFloat("Accel_filter", Accel_filter);
  prefs.putFloat("Gyro_filter", Gyro_filter);
  prefs.putFloat("Kp_roll_rate", Kp_roll_rate);
  prefs.putFloat("Ki_roll_rate", Ki_roll_rate);
  prefs.putFloat("Kd_roll_rate", Kd_roll_rate);
  prefs.putFloat("Kp_roll_angle", Kp_roll_angle);
  prefs.putFloat("Ki_roll_angle", Ki_roll_angle);
  prefs.putFloat("Kp_pitch_rate", Kp_pitch_rate);
  prefs.putFloat("Ki_pitch_rate", Ki_pitch_rate);
  prefs.putFloat("Kd_pitch_rate", Kd_pitch_rate);
  prefs.putFloat("Kp_pitch_angle", Kp_pitch_angle);
  prefs.putFloat("Ki_pitch_angle", Ki_pitch_angle);
  prefs.putFloat("roll_maxRate", rollLimits.maxRate);
  prefs.putFloat("pitch_maxRate", pitchLimits.maxRate);
  prefs.putFloat("Kp_yaw_rate", Kp_yaw_rate);
  prefs.putFloat("Ki_yaw_rate", Ki_yaw_rate);
  prefs.putFloat("Kd_yaw_rate", Kd_yaw_rate);
  prefs.end(); // close namespace
  Serial.println("Free entries: " + String(prefs.freeEntries()));
}

void loadParameters()
{
  prefs.begin("rawpter", true); // namespace "rawpter", read-only

  // Use current variable values as defaults
  failsafeThrottlePWM = prefs.getInt("fsThrottlePWM", failsafeThrottlePWM);
  altitudeData.kp_altitude_rate = prefs.getFloat("kpaltrate", altitudeData.kp_altitude_rate);
  altitudeData.ki_altitude_rate = prefs.getFloat("kialtrate", altitudeData.ki_altitude_rate);
  altitudeData.ceiling = prefs.getFloat("ceiling", altitudeData.ceiling);
  trimPitch = prefs.getFloat("trimPitch", trimPitch);
  trimRoll = prefs.getFloat("trimRoll", trimRoll);
  trimYaw = prefs.getFloat("trimYaw", trimYaw);
  
  stick_dampener = prefs.getFloat("stick_dampener", stick_dampener);
  i_limit = prefs.getFloat("i_limit", i_limit);
  i_limit_rate = prefs.getFloat("i_limit_rate", i_limit_rate);
  B_madgwick = prefs.getFloat("B_madgwick", B_madgwick);
  Accel_filter = prefs.getFloat("Accel_filter", Accel_filter);
  Gyro_filter = prefs.getFloat("Gyro_filter", Gyro_filter);

  Kp_roll_rate = prefs.getFloat("Kp_roll_rate", Kp_roll_rate);
  Ki_roll_rate = prefs.getFloat("Ki_roll_rate", Ki_roll_rate);
  Kd_roll_rate = prefs.getFloat("Kd_roll_rate", Kd_roll_rate);
  Kp_roll_angle = prefs.getFloat("Kp_roll_angle", Kp_roll_angle);
  Ki_roll_angle = prefs.getFloat("Ki_roll_angle", Ki_roll_angle);
  Kp_pitch_rate = prefs.getFloat("Kp_pitch_rate", Kp_pitch_rate);
  Ki_pitch_rate = prefs.getFloat("Ki_pitch_rate", Ki_pitch_rate);
  Kd_pitch_rate = prefs.getFloat("Kd_pitch_rate", Kd_pitch_rate);
  Kp_pitch_angle = prefs.getFloat("Kp_pitch_angle", Kp_pitch_angle);
  Ki_pitch_angle = prefs.getFloat("Ki_pitch_angle", Ki_pitch_angle);

  rollLimits.maxRate = prefs.getFloat("roll_maxRate", rollLimits.maxRate);
  pitchLimits.maxRate = prefs.getFloat("pitch_maxRate", pitchLimits.maxRate);

  Kp_yaw_rate = prefs.getFloat("Kp_yaw_rate", Kp_yaw_rate);
  Ki_yaw_rate = prefs.getFloat("Ki_yaw_rate", Ki_yaw_rate);
  Kd_yaw_rate = prefs.getFloat("Kd_yaw_rate", Kd_yaw_rate);

  prefs.end();
  Serial.println("Parameters loaded from NVS (or defaults if none stored).");
}

void GenerateDefaultPage(WiFiClient &client) 
{ 
  Serial.println("About to MakeWebPage.");
  // Build HTML body first (so we can compute Content-Length)
  String body; 
  body.reserve(8192); 
  // optional: pre-allocate to reduce reallocs
  // Begin HTML 
  body += "<!DOCTYPE html><html lang='en'><head>";
  body += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  body += "<style>";
  body += ".btn { padding:6px 12px; border-radius:4px; cursor:pointer; }"; 
  body += ".btn-primary { background:#0d6efd; color:#fff; border:1px solid #0d6efd; }";
  body += ".alert { padding:8px; border-radius:4px; margin-top:8px; }"; 
  body += ".alert-success { background:#d1e7dd; color:#0f5132; }"; 
  body += ".alert-warning { background:#fff3cd; color:#664d03; }";
  body += ".alert-danger { background:#f8d7da; color:#842029; }";
  body += "table { border-collapse:collapse; }";
  body += "td { padding:4px; }";
  body += "input[type=text], input[type=number] {";
  body += "font-size:20px; padding:8px; width:180px; box-sizing:border-box;";
  body += "border:1px solid #ccc; border-radius:6px; margin-top:6px; }";
  body += "</style></head><body><div id='myHider'>"; 
  body += "<form method=get><div class='container'>";
  // Header and snapshot
  body += "<h1 class='alert alert-success mb-0 pt-0'>Rawpter V8"; 
  body += "<span style='font-size:10px;'> by Raising Awesome <a href='/' style='margin-top:4px; padding:3px 6px; background-color:#0d6efd; color:#fff; border:1px solid #0d6efd; border-radius:2px; width:100%; cursor:pointer; font-size:10px;'>refresh</a></span></h1><hr>";
  body += "<b>Snapshot:</b><br>Desired Roll=" + String(roll_des) + "&#176; IMU Roll=" + String(roll_IMU) + "&#176;<br>";
  body += "Desired Pitch=" + String(pitch_des) + "&#176; IMU Pitch=" + String(pitch_IMU) + "&#176;<br>";
  body += "Loop Time=" + String(int(round((deltaTime) / 1000000))) + " Throttle PWM=" + String(PWM_throttle) + "<br>";
  body += "Battery=" + String(calced_voltage, 1) + "V (" + String(batteryVoltage) + ")<br>";
  body += "Highest Altitude=" + String(altitudeData.highestAltitude) + "<br>";
  body += "Highest Throttle=" + String(highestThrottlePWM) + "<br>";
  body += "Lowest Throttle=" + String(lowestThrottlePWM) + "<br>";
  body += "Battery=" + String(calced_voltage, 1) + "V (" + String(batteryVoltage) + ")<br>";
  body += "Altitude=" + String(altitudeData.altitude) + " ft<br><hr>";
  if (calced_voltage < 13) { body += "<br><div class='alert alert-danger'>DANGER: BATTERY CRITICAL!</div><br>"; } else if (calced_voltage < 14) { body += "<br><div class='alert alert-warning'>Warning: BATTERY LOW!</div><br>"; }
  // Parameters table
  body += "<table class=table><thead class=thead-dark><th></th><th>Kp</th><th>Ki</th><th>Kd</th></thead>";
  // Roll row 
  body += "<tr><td>Roll Angle:</td><td><input name=kp_roll_angle style='width:80px;' type=number step=.0001 value='" +
  String(Kp_roll_angle, 4) + "'></td><td><input name=ki_roll_angle style='width:80px;' type=number step=.0001 value='" +
  String(Ki_roll_angle, 4) + "'></td></tr>";
  body += "<tr><td>Roll Rate:</td><td><input name=kp_roll_rate style='width:80px;' type=number step=.0001 value='" +
  String(Kp_roll_rate, 4) + "'></td><td><input style='width:80px;' name=ki_roll_rate type=number step=.0001 value='" +
  String(Ki_roll_rate, 4) + "'></td><td><input name=kd_roll_rate type=number step=.0001 style='width:80px;' value='" +
  String(Kd_roll_rate, 4) + "'></td></tr>";
  // Pitch row 
  body += "<tr><td>Pitch Angle:</td><td><input name=kp_pitch_angle style='width:80px;' type=number step=.0001 value='" +
  String(Kp_pitch_angle, 4) + "'></td><td><input style='width:80px;' name=ki_pitch_angle type=number step=.0001 value='" +
  String(Ki_pitch_angle, 4) + "'></td></tr>";
  body += "<tr><td>Pitch Rate:</td><td><input name=kp_pitch_rate style='width:80px;' type=number step=.0001 value='" +
  String(Kp_pitch_rate, 4) + "'></td><td><input style='width:80px;' name=ki_pitch_rate type=number step=.0001 value='" +
  String(Ki_pitch_rate, 4) + "'></td><td><input name=kd_pitch_rate type=number step=.0001 style='width:80px;' value='" +
  String(Kd_pitch_rate, 4) + "'></td></tr>";
  // Yaw row
  body += "<tr><td>Yaw Rate:</td><td><input name=kp_yaw_rate type=number step=0.0001 style='width:80px;' value='" +
  String(Kp_yaw_rate, 4) + "'></td><td><input style='width:80px;' type=number step=.0001 name=ki_yaw_rate value='" +
  String(Ki_yaw_rate, 4) + "'></td><td><input type=number step=.0001 name=kd_yaw_rate style='width:80px;' value='" +
  String(Kd_yaw_rate, 4) + "'></td></tr>";
  // Integral limit row
  body += "<tr><td>Angle Integral Max:</td><td><input type=number step=.0001 name=i_limit style='width:90px;' value='" +
  String(i_limit) + "'></td></tr>"; body += "<tr><td>Rate Integral Max:</td>"
        "<td><input type=number step=.0001 name=i_limit_rate style='width:90px;' value='" +
        String(i_limit_rate) + "'></td></tr>";

  body += "<tr><td>B_Madgwick (0.06 default):</td><td><input type=number step=.0001 name=B_madgwick style='width:90px;' value='" +
  String(B_madgwick) + "'></td></tr>";
  body += "</table><br>";
  body += "<table class=table><thead class=thead-dark><th></th><th>Max Rate</th></thead>";
  body += "<tr><td>Roll:</td><td><input name=roll_maxRate style='width:90px;' type=number value='" + String(rollLimits.maxRate) + "'></td></tr>";
  // Pitch row
  body += "<tr><td>Pitch:</td><td><input name=pitch_maxRate style='width:90px;' type=number value='" + String(pitchLimits.maxRate) + "'></td></tr>";
  body += "</table><br>Tip: Before powering down, Save to Storage - but don't bother otherwise to extend the flash life.<br>";
  // Additional parameters table 
  body += "<table>";
  body += "<tr><td>Ceiling:</td><td><input type=number name=ceiling style='width:80px;' step = .1 value='" + String(altitudeData.ceiling,1) + "'></td></tr>";
  body += "<tr><td>Failsafe Throttle (1500 to 2000):</td><td><input type=number name=failsafeThrottlePWM style='width:80px;' value='" + String(failsafeThrottlePWM) + "'></td></tr>";
  body += "<tr><td>Kp Altitude Rate:</td><td><input type=number name=kp_altitude_rate style='width:100px;' step = .00001 value='" + String(altitudeData.kp_altitude_rate,5) + "'></td></tr>";
  body += "<tr><td>Ki Altitude Rate:</td><td><input type=number name=ki_altitude_rate style='width:100px;' step = .00001 value='" + String(altitudeData.ki_altitude_rate,5) + "'></td></tr>";
  body += "<tr><td>Trim - Pitch (-500 to 500):</td><td><input type=number name=trimPitch style='width:80px;' value='" + String(trimPitch) + "'></td></tr>";
  body += "<tr><td>Trim - Roll (-500 to 500):</td><td><input type=number name=trimRoll style='width:80px;' value='" + String(trimRoll) + "'></td></tr>";
  body += "<tr><td>Trim - Yaw (-500 to 500):</td><td><input type=number name=trimYaw style='width:80px;' value='" + String(trimYaw) + "'></td></tr>";
  body += "<tr><td>Stick Dampening (0.01-1.0):<br>0.1=slow/steady, 1.0=noisy/fast</td><td><input type=number step=0.001 name=stick_dampener style='width:80px;' value='" +  String(stick_dampener, 3) + "'></td></tr>";
  body += "<tr><td>Accel Dampening (0.1-1.0):<br>0.1=slow/steady, 1.0=noisy/fast</td><td><input type=number step=0.01 name=Accel_filter style='width:80px;' value='" + String(Accel_filter) + "'></td></tr>";
  body += "<tr><td>Gyro Dampening (0.1-1.0):<br>0.1=slow/steady, 1.0=noisy/fast</td><td><input type=number step=0.01 name=Gyro_filter style='width:80px;' value='" + String(Gyro_filter) + "'></td></tr>";
  body += "</table>";
  // Buttons 
  body += "<br><input "
        "style='margin-top:8px; padding:6px 12px; background-color:#0d6efd; color:#fff; " 
        "border:1px solid #0d6efd; border-radius:4px; width:100%; cursor:pointer; font-size:20px;' " 
        "type='submit' name='action' value='SUBMIT' " 
        "onclick=\"document.getElementById('myHider').style.display='none'; document.getElementById('shower').style.display='inline'; return true;\" />";  

  body += "<br><br><br><input "
        "style='margin-top:8px; padding:6px 12px; background-color:#ff0000; color:#fff; "
        "border:1px solid #0d6efd; border-radius:4px; width:100%; cursor:pointer; font-size:20px;' "
        "type='submit' name='action' value='SAVE TO STORAGE' "
        "onclick=\"document.getElementById('myHider').style.display='none'; document.getElementById('shower').style.display='inline'; return true;\" />";
  
  body += "</div></div>"
    "<div style='display:none' id='shower'>"
    "<br><br><br>"
      "<div style='font-size:30pt; display:block; margin: 0 auto;'>"
        "Saving..."
      "</div>"
    "</div></form></body></html>";

  // Build headers now that we know the exact body length 
  String headers;
  headers.reserve(128); 
  headers += "HTTP/1.1 200 OK\r\n"; headers += "Content-Type: text/html\r\n";
  headers += "Connection: close\r\n";
  headers += "Content-Length: " + String(body.length()) + "\r\n\r\n";
  // Send headers + body in one go 
  client.write((const uint8_t *)headers.c_str(), headers.length());
  client.write((const uint8_t *)body.c_str(), body.length());  
  Serial.println("Made Web Page.");
}
