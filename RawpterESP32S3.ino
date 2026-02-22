// Rawpter 9.0 by Sean J. Miller
// Flight Controller code for ESP32-S3
// Go to https:// raisingawesome.site/projects for more info
// MIT License - use at your own risk

#include "Project.h"

constexpr bool BENCH_TESTING = false; // Used for bench testing safely with USB power only
constexpr bool CALIBRATE_ESCS = false; // used for bench calibration - be careful!!!!
constexpr bool CALCULATE_IMU_ERROR = false; // used to generate the error parameters

Preferences prefs; // Stores key flight controller configuration to ESP32S3 onboard storage
AltitudeData altitudeData{};
GPSData gps{};
WiFiServer server(80);
ConfigData configData{};
Limits rollLimits = {160.0f}; 
Limits pitchLimits = {160.0f};
BatteryMonitor batteryMonitor{};
RadioData radioData{};
RawpterMotors motors(radioData);
PID pid{};
RawpterTelemetry telemetry{};
RawpterIMU imu{};

// General stuff for controlling timing of things
unsigned long innerLoopMicroseconds = 1000000.0 / INNER_LOOP_FREQUENCY; // The microsecond equivalent of our Loop Hz.
bool resetTimers = false;
unsigned long tick_time, prev_time;
unsigned long print_counter;
volatile unsigned long debugger;
bool playingSong = false;
TaskHandle_t loopDroneHandle = NULL; // Handle for the main drone control loop task that will run on its own core.

//========================================================================================================================//
// BEGIN THE CLASSIC SETUP AND LOOP
//========================================================================================================================//

void setup()
{
  // Bootup operations
  loopDroneHandle = xTaskGetCurrentTaskHandle(); // for messeging between tasks
  Serial.begin(2000000);     // Sets up for debug info.
  radioData.begin(PPM_PIN, 1000000, 8, 2100); // GPIO Pin Number, 1MZ (1uS ticks) to track PPM pulse width, # of pulses in PPM, duration of sync high pulse
  motors.begin(); // Sets up hardware Motor Control PWM for commanding the motors
  if (CALIBRATE_ESCS) calibrateESCs();
  loadParameters();          // overrides coded parameters with the last stored on the chip or defaults if none exist.
  setupPeripherals();        // Sets up I2C, ADC resolution, and any other peripherals needed
  setupBatteryMonitor();     // Sets up the ADC to monitor battery voltage
  WiFiBegin();               // Sets up the WiFi access point and web server
  gps.begin();                // Checks for a Max10S GPS module and initializes it if found enabling return to home
  altitudeData.begin(configData.failsafeThrottlePWM);     // Gets some default info and ensures their is a working BMP581 on board
  imu.begin();                // Ensures there is a working IMU on board - it's imperative.
  if (CALCULATE_IMU_ERROR) imu.calculateIMUError(); // Use periodically to obtain the IMU error factors for calibration.
  
  if (!BENCH_TESTING)
  {
    playStartSong();
    Serial.println("Waiting for Radio");
    waitForRadio();
  }
  Serial.println("Ready...");
  if (altitudeData.hasBMP581) beginAltitudeTask(); // The BMP581 sensor read is blocking and slows the inner loop.  This runs it in a separate task.
}

void loop()
{
  tick();                                   // Starts the loop pacing
  syncSensors();                            // Altitude capture runs at 100mHZ in its own FreeRTOS task. This gives a safe way to update variables without thread conflict. ~12 microseconds to execute
  radioData.getRadioStickValues(altitudeData, gps, telemetry, configData, imu); // Gets the PWM from the radio receiver and overrides if necessary. Pulses are captured by hardware with a rmtRead call and double buffering. ~38 microseconds
  imu.getIMUdata(configData);              // Pulls raw gyro a daccelerometer data from IMU at 1kHz. IMU output. Is actually downsampled from 32kHz. 
  getDesiredAnglesAndThrottleScaledToOne(); // Convert PWM commands to normalized values at 2kHz. Adds a low pass filter to dampen remote stick noise and user twitchiness. 
  PIDControlCalcs();                        // The PID functions at 400Hz Hz. Stabilize on angle setpoint from getDesiredAnglesAndThrottle 
  motorPipeline();                          // Commands the motors at at 400Hz. This is the max adjustment rate the Simonk can handle. 
  tock();                                   // Yields until the end of the the inner loop pacing
  if (BENCH_TESTING) printJSON();
}

inline void tick()
{
  // Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = tick_time;
  tick_time = micros();
}

// ========================================================================================================================//
//                                                      FUNCTIONS                                                         //
// ========================================================================================================================//
void setupPeripherals()
{
  analogReadResolution(12);  // 12 bit ADC.  Used for sensing battery level in loopBuzzer().
  Wire.begin(8, 9); // I2C communication on pin 8 and 9
  delay(100);
  Wire.setClock(400000); // 400KhZ I2C
}

inline void syncSensors()
{ // This makes sure the pressure sensing task "bmpTaskHandle", is not writing to our variables so we don't corrupt or data.
  // the bmpTask use the variables "...Working". Once it notifies that the calculations are complete, we consume them.
  if (ulTaskNotifyTake(pdTRUE, 0) > 0)
  {
    altitudeData.altitude = altitudeData.altitudeWorking;
    altitudeData.rateFPS = altitudeData.rateFPSWorking;
    gps.latitude = gps.latitudeWorking;
    gps.longitude = gps.longitudeWorking;
    setAltitudeRate();
  }
}

void beginAltitudeTask()
{
  // Start the BMP581 task on its own core. It is a slow reader, so we'll keep it out of the way of the other tasks
  // on its own core.
  xTaskCreatePinnedToCore(
      bmpAndgpsTask,               // Task function
      "BMP and GPS Task",          // Name
      4096,                        // Stack size (bytes)
      NULL,                        // Parameters
      1,                           // Priority (Highest priority on core)
      &altitudeData.bmpTaskHandle, // Task handle
      0                            // Core 0 since it is slower overall
  );
}

void bmpAndgpsTask(void *pvParameters)
{
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz
  int ticker=0;

  while (true)
  {
    // Do the sensor read
    getAltitudeFromSensor();
    if (gps.hasGPS)
    {
      if (++ticker>100) // 1 Hz, get GPS data
      {
        ticker=0;
        gps.latitudeWorking =  gps.Max10SGPS.getLatitude() / 1e7;
        gps.longitudeWorking = gps.Max10SGPS.getLongitude() / 1e7;
        gps.fixType = gps.Max10SGPS.getFixType();
        if (gps.homeValid && gps.fixType >= 3) gps.useGPS = true;
        else gps.useGPS = false; 
      }
    }
    // Delay until the next absolute 10 ms boundary
    vTaskDelayUntil(&lastWakeTime, period);
  }
}

inline void getAltitudeFromSensor()
{
  static int flyingCounter = 0;
  
  if (!altitudeData.hasBMP581) return;
  
  if (ulTaskNotifyTake(pdTRUE, 0) > 0)
    altitudeData.invGroundPressureWorking = 0.0f; // Reset ground pressure if notified to do so.
  
  setAltitude();
  
  if (telemetry.flying && altitudeData.altitudeWorking > altitudeData.highestAltitude)
    altitudeData.highestAltitude = altitudeData.altitudeWorking;
  if (altitudeData.fastestAscent < altitudeData.rateFPSWorking)
    altitudeData.fastestAscent = altitudeData.rateFPSWorking;
  if (altitudeData.altitudeWorking >= 2.0f && !telemetry.flying)
  {
    if (++flyingCounter > 4) // make sure it is not a blip
    {
      telemetry.flying = true;
      flyingCounter = 0;
    }
  }
  else
    flyingCounter = 0;
  
  xTaskNotifyGive(loopDroneHandle); // Notify the main loop that new data is available
}

void setAltitudeRate()
{
    static unsigned long lastTime = micros();
    static float lastAltitude = 0.0f;
    static int rateCounter =0;

    unsigned long currentTime = micros();
    float currentAltitude = altitudeData.altitude;

    if (++rateCounter > 50)
    {
      rateCounter=0;
      float dt = (currentTime - lastTime) * 1e-6f;   // seconds
      if (dt <= 0.0f)
      {
          altitudeData.rateFPS = 0.0f;
          return;
      }

      float dy = currentAltitude - lastAltitude;     // feet
      altitudeData.rateFPS = dy / dt;         // feet per second

      lastAltitude = currentAltitude;
      lastTime = currentTime;
    }
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
    altitudeData.altitudeWorking += 0.20f * (newAltitude - altitudeData.altitudeWorking);  //1.0 would make newAltitude be the current value, .1 would make the prior value hold the most weight
  }
  else
  {
    altitudeData.altitudeWorking = 0.0f; // fallback if sensor read fails
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
  /* 

  // Suppressing throttle compensation when tilted to gain computational speed 
  // Thrust increaser: increase thrust whenever a roll or pitch is desired
  float roll_rad = roll_des * DEG_TO_RAD;
  float pitch_rad = pitch_des * DEG_TO_RAD;

  // Approximate total tilt angle (hypotenuse of roll and pitch)
  float tilt_angle_rad = sqrtf(roll_rad * roll_rad + pitch_rad * pitch_rad);

  // Compute thrust compensation factor
  float thrust_compensation = 1.0f / cos(tilt_angle_rad);
  thrust_compensation = constrain(thrust_compensation, 1.0f, 1.15f); // Prevent runaway
  
  // Apply to throttle
  float throttle_corrected = throttle_desired * thrust_compensation;
  */

  // Quad mixing
  motors.m1_command_scaled = pid.throttle_desired - pid.pitch_PID + pid.roll_PID + pid.yaw_PID; // Front left
  motors.m2_command_scaled = pid.throttle_desired - pid.pitch_PID - pid.roll_PID - pid.yaw_PID; // Front right
  motors.m3_command_scaled = pid.throttle_desired + pid.pitch_PID - pid.roll_PID + pid.yaw_PID; // Back right
  motors.m4_command_scaled = pid.throttle_desired + pid.pitch_PID + pid.roll_PID - pid.yaw_PID; // Back left

  // Constrain outputs (final safety clamp, although it should never be more than 1 since they are all scaled)
  motors.m1_command_scaled = constrain(motors.m1_command_scaled, 0.0f, 1.0f);
  motors.m2_command_scaled = constrain(motors.m2_command_scaled, 0.0f, 1.0f);
  motors.m3_command_scaled = constrain(motors.m3_command_scaled, 0.0f, 1.0f);
  motors.m4_command_scaled = constrain(motors.m4_command_scaled, 0.0f, 1.0f);

  //ConditionCommands(); // Smooth ramping of motor speeds when launching and landing to prevent hard hits.
}

inline void ConditionCommands()
{
  // This prevents sudden motor changes that could jolt the drone.
  // The ramp step controls how quickly motor commands can change per loop iteration.

  if (motors.m1_command_scaled_prev > motors.m1_command_scaled + motors.motor_ramp_step)
  {
    motors.m1_command_scaled = motors.m1_command_scaled_prev - motors.motor_ramp_step;
  }
  else if (motors.m1_command_scaled_prev < motors.m1_command_scaled - motors.motor_ramp_step)
  {
    motors.m1_command_scaled = motors.m1_command_scaled_prev + motors.motor_ramp_step;
  }

  if (motors.m2_command_scaled_prev > motors.m2_command_scaled + motors.motor_ramp_step)
  {
    motors.m2_command_scaled = motors.m2_command_scaled_prev - motors.motor_ramp_step;
  }
  else if (motors.m2_command_scaled_prev < motors.m2_command_scaled - motors.motor_ramp_step)
  {
    motors.m2_command_scaled = motors.m2_command_scaled_prev + motors.motor_ramp_step;
  }

  if (motors.m3_command_scaled_prev > motors.m3_command_scaled + motors.motor_ramp_step)
  {
    motors.m3_command_scaled = motors.m3_command_scaled_prev - motors.motor_ramp_step;
  }
  else if (motors.m3_command_scaled_prev < motors.m3_command_scaled - motors.motor_ramp_step)
  {
    motors.m3_command_scaled = motors.m3_command_scaled_prev + motors.motor_ramp_step;
  }

  if (motors.m4_command_scaled_prev > motors.m4_command_scaled + motors.motor_ramp_step)
  {
    motors.m4_command_scaled = motors.m4_command_scaled_prev - motors.motor_ramp_step;
  }
  else if (motors.m4_command_scaled_prev < motors.m4_command_scaled - motors.motor_ramp_step)
  {
    motors.m4_command_scaled = motors.m4_command_scaled_prev + motors.motor_ramp_step;
  }

  motors.m1_command_scaled_prev = motors.m1_command_scaled;
  motors.m2_command_scaled_prev = motors.m2_command_scaled;
  motors.m3_command_scaled_prev = motors.m3_command_scaled;
  motors.m4_command_scaled_prev = motors.m4_command_scaled;
}

void waitForRadio()
{
  // Wait until the throttle is turned down and throttlecut switch is flipped down before allowing anything else to happen.
  // Using PWM_throttle_prev so that it is seeded
  radioData.PWM_throttle_prev = radioData.getRadioPWM(throttlePin, 1520);
  radioData.PWM_ThrottleCutSwitch = radioData.getRadioPWM(throttleCutSwitchPin, 2000);
  radioData.PWM_FailsafeSwitch = radioData.getRadioPWM(failsafePin, 1000);

  while ((radioData.PWM_throttle_prev < 1400 || radioData.PWM_throttle_prev > 1519 || radioData.PWM_ThrottleCutSwitch > 1500 || radioData.PWM_FailsafeSwitch < 1500) && !BENCH_TESTING)
  { // Throttle Cut switch pulled down (flymode) is 2000. Up is 1000 (cut)
    // if the throttle is up or the cut switch is set to Fly (pulled down), then wait until the switches are set.
    radioData.PWM_throttle_prev = radioData.getRadioPWM(throttlePin, 1520);              // keep it stuck in the loop if it failsafes
    radioData.PWM_ThrottleCutSwitch = radioData.getRadioPWM(throttleCutSwitchPin, 2000); // keep it stuck in the loop if it failsafes
    radioData.PWM_FailsafeSwitch = radioData.getRadioPWM(failsafePin, 1000);
    tick();
    imu.getIMUdata(configData);
    loopWiFi();
    tock(); // This will warm up the Madgwick while we wait for them to turn on the radio.
    vTaskDelay(1);
  }

  // Seed the rest of the controls' prev(ious) values.
  // This is so the filters don't have to climb up to their true values from zero at their startup
  // which throws off desired values for a brief time.
  radioData.PWM_roll_prev = radioData.getRadioPWM(rollPin, 1500);
  radioData.PWM_pitch_prev = radioData.getRadioPWM(pitchPin, 1500);
  radioData.PWM_yaw_prev = radioData.getRadioPWM(yawPin, 1500);
}

void setupBatteryMonitor()
{
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
  playingSong = true;
  for (int i = 0; i < 13; i++)
  {
    int duration = noteDurations[i];
    ledcWriteTone(BUZZER_PIN, melody[i]);

    delay(duration * .8); // add pause between notes
    ledcWriteTone(BUZZER_PIN, 0);
    delay(duration * .2);
  }
  playingSong = false;
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
  playingSong = true;
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
  playingSong = false;
}

inline void loopBuzzer()
{ // this monitors the battery.  the lower it gets, the faster it beeps.
  // disabling for now until we get the wiring correct in the next version
  static bool beeping = false;                 // For tracking beeping when the battery is getting low.
  static unsigned long buzzer_spacing = 30000;
  static unsigned long buzzer_millis = millis();
  unsigned long myTime = millis();
  
  if (playingSong||BENCH_TESTING) return;
  if (radioData.PWM_FailsafeSwitch < 1500)
    buzzer_spacing = 100;
  else 
    buzzer_spacing = 30000;

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
}

float getCalcedVoltage()
{
  // For the battery voltage calc, you can use ohm's law on your chosen voltage divider resistors and get the voltage ratio of the 12bit ADC.
  // I simply recorded values against fed voltages from my bench power supply and fit
  // a line.  Good ole' y=mx+b.

  return (((float)batteryMonitor.batteryVoltage * 0.0397) - 3.6127);
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

  pid.throttle_desired = (radioData.PWM_throttle - 1500.0f) / 500.0f;   // Between  0 and 1 because anything under 1500 will be set to 1500 for now.
  pid.roll_des = (radioData.PWM_roll - 1500.0f + configData.trimRoll) / 500.0f;    // Between -1 and 1
  pid.pitch_des = (radioData.PWM_pitch - 1500.0f + configData.trimPitch) / 500.0f; // Between -1 and 1
  pid.yaw_des = -(radioData.PWM_yaw - 1500.0f + configData.trimYaw) / 500.0f;       // Between -1 and 1

  // Constrain within normalized bounds
  pid.throttle_desired = constrain(pid.throttle_desired, 0.0f, 1.0f); // Between 0 and 1
  pid.roll_des = constrain(pid.roll_des, -1.0f, 1.0f) * configData.maxRoll;      // Between -maxRoll and +maxRoll
  pid.pitch_des = constrain(pid.pitch_des, -1.0f, 1.0f) * configData.maxPitch;   // Between -maxPitch and +maxPitch
  pid.yaw_des = constrain(pid.yaw_des, -1.0f, 1.0f) * configData.maxYaw;         // Between -maxYaw and +maxYaw
}

inline void resetAllTiming()
{
  // Once flicked back into flight mode, this will set all frequencies to the same start light just incase there was drift.
  resetTimers = false;
  unsigned long currentMicros = micros();
  imu.lastMadgwickUpdateMicros = currentMicros; // Used for Madgwick delta time calculations
  pid.PIDCounter = 0;                           // using ticks to keep everything in sync
  motors.ESCWriteCounter = 0;                   // using ticks to keep everything in sync
}

void PIDControlCalcs()
{
  // --- Roll ---
  static float integral_rate_roll = 0.0f;
  static float integral_rate_pitch = 0.0f;
  static float integral_rate_yaw = 0.0f;

  float p = imu.GyroX; // roll rate 
  float q = imu.GyroY; // pitch rate (nose up positive) - pitch is already minused to correct physical orientation to NASA rules
  float r = imu.GyroZ; 
  
  if (radioData.PWM_throttle < 1520) // Reset the control if on the ground. This prevents integral windup and sudden jumps on takeoff.
  {
    integral_rate_roll = integral_rate_pitch = integral_rate_yaw = 0;
    pid.roll_PID = pid.pitch_PID = pid.yaw_PID = 0;
    return;
  }

  if (configData.rateControlMode)
  {
    pid.desiredRateRoll = (pid.roll_des-configData.trimRoll)/configData.maxRoll * rollLimits.maxRate;    // Between -500 and 500 deg/sec
    pid.desiredRatePitch =  (pid.pitch_des-configData.trimPitch)/configData.maxPitch * pitchLimits.maxRate; // Between -500 and 500 deg/sec
    pid.desiredRateRoll = constrain(pid.desiredRateRoll, -rollLimits.maxRate, rollLimits.maxRate);
    pid.desiredRatePitch = constrain(pid.desiredRatePitch, -pitchLimits.maxRate, pitchLimits.maxRate);
  }
  else AngleLoopCalcs(); // Get the new desired angular rates based on current angle versus desired angle

  // --- Roll ---
  float rateErrorRoll = pid.desiredRateRoll - p;
  integral_rate_roll += rateErrorRoll * imu.madDeltaTime;
  integral_rate_roll = constrain(integral_rate_roll, -configData.i_limit_rate, configData.i_limit_rate); // Think of this as the tracked energy required to achieve motor torque

  float derivative_roll = p;
  pid.roll_PID = configData.Kp_roll_rate * rateErrorRoll +
             configData.Ki_roll_rate * integral_rate_roll -
             configData.Kd_roll_rate * derivative_roll;

  // --- Pitch ---
  float rateErrorPitch = pid.desiredRatePitch - q;
  integral_rate_pitch += rateErrorPitch * imu.madDeltaTime;
  integral_rate_pitch = constrain(integral_rate_pitch, -configData.i_limit_rate, configData.i_limit_rate);

  float derivative_pitch = q;
  pid.pitch_PID = configData.Kp_pitch_rate * rateErrorPitch +
              configData.Ki_pitch_rate * integral_rate_pitch -
              configData.Kd_pitch_rate * derivative_pitch;

  // --- Yaw ---
  float rateErrorYaw = pid.yaw_des - r;
  integral_rate_yaw += rateErrorYaw * imu.madDeltaTime;
  integral_rate_yaw = constrain(integral_rate_yaw, -configData.i_limit_rate, configData.i_limit_rate);

  float derivative_yaw = r;
  pid.yaw_PID = configData.Kp_yaw_rate * rateErrorYaw +
            configData.Ki_yaw_rate * integral_rate_yaw -
            configData.Kd_yaw_rate * derivative_yaw;
}

void AngleLoopCalcs()
{
  static unsigned long lastTimeMicros = micros();
  static unsigned long dt;
  static float integral_roll = 0.0f;
  static float integral_pitch = 0.0f;

  if (++pid.PIDCounter < (INNER_LOOP_FREQUENCY / PID_FREQ_HZ))
    return;
  pid.PIDCounter = 0;

  dt = micros() - lastTimeMicros;
  lastTimeMicros = micros();

  // --- Roll ---
  float angleErrorRoll = pid.roll_des - imu.roll_IMU;
  integral_roll += angleErrorRoll * dt;
  integral_roll = constrain(integral_roll, -configData.i_limit_angle, configData.i_limit_angle);

  pid.desiredRateRoll = configData.Kp_roll_angle * angleErrorRoll +
                    configData.Ki_roll_angle * integral_roll;
  pid.desiredRateRoll = constrain(pid.desiredRateRoll, -rollLimits.maxRate, rollLimits.maxRate);

  // --- Pitch ---
  float angleErrorPitch = pid.pitch_des - imu.pitch_IMU;
  integral_pitch += angleErrorPitch * dt;
  integral_pitch = constrain(integral_pitch, -configData.i_limit_angle, configData.i_limit_angle);

  pid.desiredRatePitch = configData.Kp_pitch_angle * angleErrorPitch +
                     configData.Ki_pitch_angle * integral_pitch;
  pid.desiredRatePitch = constrain(pid.desiredRatePitch, -pitchLimits.maxRate, pitchLimits.maxRate);

  // --- Yaw setpoint comes directly from the stick --- //
}

inline void motorPipeline()
{
  motors.ESCWriteCounter++;

  if (motors.ESCWriteCounter < (INNER_LOOP_FREQUENCY / MOTOR_FREQ_HZ))
    return;

  motors.ESCWriteCounter = 0;

  controlMixer();            // Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  scaleMotorCommandsToPWM(); // Scales motor commands from 0-1 to PWM
  throttleCut();             // Directly sets motor commands to off based on channel 5 being switched
  commandMotors();           // Sends command pulses to each ESC pin to drive the motors
}

inline void commandMotors()
{
    if (BENCH_TESTING)
    {
      // Minimum throttle (1000 µs)
      motors.sendPWMtoESC(1, 1000);
      motors.sendPWMtoESC(2, 1000);
      motors.sendPWMtoESC(3, 1000);
      motors.sendPWMtoESC(4, 1000);
    }
    else
    {
      motors.sendPWMtoESC(1, motors.m1_command_PWM);
      motors.sendPWMtoESC(2, motors.m2_command_PWM);
      motors.sendPWMtoESC(3, motors.m3_command_PWM);
      motors.sendPWMtoESC(4, motors.m4_command_PWM);
    }
}

inline void scaleMotorCommandsToPWM()
{
  // DESCRIPTION: Scale normalized actuator commands to values for ESC protocol
  // The actual pulse frame is set at the setupMotorCommunication.
  //
  // Scale to Servo PWM 1000-2000 microseconds for stop to full speed.  No need to constrain since mx_command_scaled already is.
  motors.m1_command_PWM = configData.throttleLimit * motors.m1_command_scaled * 1000 + 1000;
  motors.m2_command_PWM = configData.throttleLimit * motors.m2_command_scaled * 1000 + 1000;
  motors.m3_command_PWM = configData.throttleLimit * motors.m3_command_scaled * 1000 + 1000;
  motors.m4_command_PWM = configData.throttleLimit * motors.m4_command_scaled * 1000 + 1000;
}

inline void calibrateESCs()
{
    // DESCRIPTION: Used in void setup() to allow standard ESC calibration procedure with the radio to take place.
    // If the Throttle kill switch is in the up position, then skip calibration. To calibrate, turn on the
    // radio and switch the kill switch down and then powerup the drone until it does the beep sequence.  
    // It takes around 8 seconds.
    
    if (!CALIBRATE_ESCS) return; // only do without props - WRITTEN IN BLOOD!

    Serial.println("Calibrating ESCs...");

    // Full throttle (2000 µs)
      motors.sendPWMtoESC(1, 2000);
      motors.sendPWMtoESC(2, 2000);
      motors.sendPWMtoESC(3, 2000);
      motors.sendPWMtoESC(4, 2000);

    delay(2000);

    // Minimum throttle (1000 µs)
      motors.sendPWMtoESC(1, 1000);
      motors.sendPWMtoESC(2, 1000);
      motors.sendPWMtoESC(3, 1000);
      motors.sendPWMtoESC(4, 1000);

    delay(1000);

    Serial.println("Calibration complete.");

    // Stall out to force the user to comment this back out after calibration.
    while (true) delay(10);
}

void throttleCut()
{
  // DESCRIPTION: Directly set actuator outputs to minimum value if triggered
  /*
   * Monitors the state of radio command PWM_throttle. This is the last function
   * called before commandMotors() is called so that the last thing checked is if the user is giving permission to command
   * the motors to anything other than minimum value. Safety first.
   */
  static int throttleCutCounter = 0;
  static int throttleNotCutCounter = 0;

  if (radioData.throttle_is_cut)
  {
    motors.kill(); // make sure we keep those motors at 0 if the throttle was cut.

    if (radioData.PWM_ThrottleCutSwitch > 1500)
    { // switch is in the down position which means enable flight
      // reset (uncut throttle) only if throttle is down to prevent a jolting suprise
      if (radioData.PWM_throttle < 1520 && ++throttleNotCutCounter > 10)
      { // The radio is ready for flight and confirmed not to be just a blip.
        if (radioData.PWM_FailsafeSwitch < 1900 /*landing*/)
        {
          playNope(); // Don't want to accidently start in "land mode", so give the pilot a toot.
          throttleNotCutCounter = 0;
        }
        else
        { // Uncut throttle and prepare for flight
          throttleNotCutCounter = 0;
          throttleCutCounter = 0;
          radioData.throttle_is_cut = false;
          telemetry.flying = false;
          if (altitudeData.hasBMP581) xTaskNotifyGive(altitudeData.bmpTaskHandle); // Notify that we want the altitude to reset to current ground level
          telemetry.lowestThrottlePWM = 2000; // Reset for tracking highest and lowest throttle during flight
          telemetry.highestThrottlePWM = 1500;
          altitudeData.highestAltitude = 0.0f;
          altitudeData.fastestAscent = 0.0f;
          playReadySong();    // This gives us a delay to loop a few times for the other bmpTask altitude variable updates while readying the pilot as well - a win-win strategy./
          gps.setHome();         // Set home position on throttle uncut.
          imu.MadgwickInit();     // Reset the quarterion based on sitting still.
          resetTimers = true; // This will reset all counters to sync timing on the next tock();
        }
      }
    }
    return;
  }
  else if (radioData.PWM_ThrottleCutSwitch < 1300)
  {
    // The switch is in the up position meaning throttle is purposely cut.  ThrottleCutCounter will ensure it is not just a blip that has caused a false cut and drop it from the sky.
    if (++throttleCutCounter > 10)
    {
      radioData.throttle_is_cut = true;
      throttleCutCounter = 0;
      telemetry.flying = false;
      motors.kill();
    }
    return;
  }

  // This attemps to save propellers by not driving motors when it goes full sideways. It is also helpful if it accidently runs in a house to keep it from becoming the Tazmanian devil.
  if (radioData.PWM_throttle<1600&&!BENCH_TESTING)
  {
    if (imu.roll_IMU > 75 || imu.roll_IMU < -75 || imu.pitch_IMU > 75 || imu.pitch_IMU < -75)
    {
      motors.kill();
      return;
    }
  }

  if (altitudeData.hasBMP581)
  {
    if (!telemetry.flying && altitudeData.altitude < 2 && !BENCH_TESTING) // this is tip over protection at take-off
    {
      if (imu.roll_IMU > 15 || imu.roll_IMU < -15 || imu.pitch_IMU > 15 || imu.pitch_IMU < -15)
      {
        // This helps with struggles on take off or it sitting too tilted on the launch pad.
        motors.kill();
        return;
      }
    }
  }
  throttleNotCutCounter = 0;
  throttleCutCounter = 0;
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
    if (radioData.throttle_is_cut)
    {
      loopBuzzer();                             // Monitors battery voltage and beeps if low. Lowest priority task.
      loopWiFi();
    }
    taskYIELD();
  };

  if (resetTimers)
    resetAllTiming(); // Resets the counters that keep the beat for outer loops
}

void printJSON()
{
  if (tick_time - print_counter > 10000)
  { // Don't go too fast or it slows down the main loop. this is a third of a second.
    print_counter = micros();
    Serial.print(F("{\"roll\": "));
    Serial.print(imu.roll_IMU);
    Serial.print(F(", \"pitch\": "));
    Serial.print(imu.pitch_IMU);
    Serial.print(F(", \"yaw\": "));
    Serial.print(imu.yaw_IMU);
    Serial.print(F(", \"RollKp\": "));
    Serial.print(configData.Kp_roll_rate);
    Serial.print(F(", \"RollKi\": "));
    Serial.print(configData.Ki_roll_rate);
    Serial.print(F(", \"RollKd\": "));
    Serial.print(configData.Kd_roll_rate);

    Serial.print(F(", \"m1\": "));
    Serial.print(motors.m1_command_PWM);
    Serial.print(F(", \"m2\": "));
    Serial.print(motors.m2_command_PWM);
    Serial.print(F(", \"m3\": "));
    Serial.print(motors.m3_command_PWM);
    Serial.print(F(", \"m4\": "));
    Serial.print(motors.m4_command_PWM);

    Serial.print(F(", \"AccX\": "));
    Serial.print(imu.AccX);
    Serial.print(F(", \"AccY\": "));
    Serial.print(imu.AccY);
    Serial.print(F(", \"AccZ\": "));
    Serial.print(imu.AccZ);

    Serial.print(F(", \"GyroX\": "));
    Serial.print(imu.GyroX);
    Serial.print(F(", \"GyroY\": "));
    Serial.print(imu.GyroY);
    Serial.print(F(", \"GyroZ\": "));
    Serial.print(imu.GyroZ);
    Serial.print(F(", \"Flying\": "));
    Serial.print(telemetry.flying);
    Serial.print(F(", \"Altitude\": "));
    Serial.print(altitudeData.altitude);
    Serial.print(F(", \"YawIMU\": "));
    Serial.print(imu.yaw_IMU);

    Serial.print(F(", \"ThroDes\": "));
    Serial.print(pid.throttle_desired);
    Serial.print(F(", \"RollDes\": "));
    Serial.print(pid.roll_des);
    Serial.print(F(", \"PitchDes\": "));
    Serial.print(pid.pitch_des);
    Serial.print(F(", \"YawDes\": "));
    Serial.print(pid.yaw_des);
    Serial.print(F(", \"Pitch_PID\": "));
    Serial.print(pid.pitch_PID);
    Serial.print(F(", \"Roll_PID\": "));
    Serial.print(pid.roll_PID);
    Serial.print(F(", \"Yaw_PID\": "));
    Serial.print(pid.yaw_PID);

    Serial.print(F(", \"PWM_throttle\": "));
    Serial.print(radioData.PWM_throttle);
    Serial.print(F(", \"PWM_roll\": "));
    Serial.print(radioData.PWM_roll);
    Serial.print(F(", \"PWM_pitch\": "));
    Serial.print(radioData.PWM_pitch);
    Serial.print(F(", \"PWM_yaw\": "));
    Serial.print(radioData.PWM_yaw);
    Serial.print(F(", \"PWM_ThrottleCutSwitch\": "));
    Serial.print(radioData.PWM_ThrottleCutSwitch);
    Serial.print(F(", \"PWM_FailsafeSwitch\": "));
    Serial.print(radioData.PWM_FailsafeSwitch);
    
    Serial.print(F(", \"Throttle_is_Cut\": "));
    Serial.print(radioData.throttle_is_cut);

    Serial.print(F(", \"Failsafe\": "));
    Serial.print(debugger);

    Serial.print(F(", \"DeltaTime\": "));
    Serial.print(imu.madDeltaTime * 1000000.0);
    Serial.println("}");
  }
}

void WiFiBegin()
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

  if (!client.available()) {
      client.stop();
      return;
  }

  // Read request line
  String req = client.readStringUntil('\r');
  if (client.available()) client.readStringUntil('\n'); // consume newline
  
  // --- iOS captive portal probe ---
  if (req.startsWith("GET /hotspot-detect.html"))
  {
    const char *body = "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>";
    int len = strlen(body);
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.print("Content-Length: ");
    client.println(len);
    client.println();
    client.print(body);
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
    if (BENCH_TESTING) Serial.println("Generating the page...");
    GenerateDefaultPage(client);
    if (BENCH_TESTING) Serial.println("Served the page.");
  }

  client.stop();
  taskYIELD();
}

void setValuesFromUserForm(String req)
{
  // Read the full request line (already done in loopWiFi)

  int qIndex = req.indexOf('?');
  int hIndex = req.indexOf("HTTP");
  if (BENCH_TESTING) Serial.print("qIndex: ");
  if (BENCH_TESTING) Serial.println(String(qIndex));
  if (BENCH_TESTING) Serial.print(" hIndex: ");
  if (BENCH_TESTING) Serial.println(String(hIndex));
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
      configData.stick_dampener = value.toFloat();
    else if (key == "i_limit_rate")
      configData.i_limit_rate = value.toFloat();
    else if (key == "i_limit_angle")
      configData.i_limit_angle = value.toFloat();
    else if (key == "Accel_filter")
      configData.Accel_filter = value.toFloat();
    else if (key == "Gyro_filter")
      configData.Gyro_filter = value.toFloat();
    else if (key == "B_madgwick")
      configData.B_madgwick = value.toFloat();
    else if (key == "kp_roll_angle")
      configData.Kp_roll_angle = value.toFloat();
    else if (key == "ki_roll_angle")
      configData.Ki_roll_angle = value.toFloat();
    else if (key == "kp_pitch_angle")
      configData.Kp_pitch_angle = value.toFloat();
    else if (key == "ki_pitch_angle")
      configData.Ki_pitch_angle = value.toFloat();
    else if (key == "kp_roll_rate")
      configData.Kp_roll_rate = value.toFloat();
    else if (key == "ki_roll_rate")
      configData.Ki_roll_rate = value.toFloat();
    else if (key == "kd_roll_rate")
      configData.Kd_roll_rate = value.toFloat();
    else if (key == "kp_pitch_rate")
      configData.Kp_pitch_rate = value.toFloat();
    else if (key == "ki_pitch_rate")
      configData.Ki_pitch_rate = value.toFloat();
    else if (key == "kd_pitch_rate")
      configData.Kd_pitch_rate = value.toFloat();
    else if (key == "kp_yaw_rate")
      configData.Kp_yaw_rate = value.toFloat();
    else if (key == "ki_yaw_rate")
      configData.Ki_yaw_rate = value.toFloat();
    else if (key == "kd_yaw_rate")
      configData.Kd_yaw_rate = value.toFloat();
    else if (key == "maxRoll")
    {
      configData.maxRoll = value.toFloat();
      configData.maxRoll = constrain(configData.maxRoll,5,70);
    }
    else if (key == "maxPitch")
    {
      configData.maxPitch = value.toFloat();
      configData.maxPitch = constrain(configData.maxPitch,5,70);
    }
    else if (key == "rateControlMode")
      configData.rateControlMode = value.toInt();
    else if (key == "roll_maxRate")
      rollLimits.maxRate = value.toFloat();
    else if (key == "pitch_maxRate")
      pitchLimits.maxRate = value.toFloat();
    else if (key =="maxYaw")
      configData.maxYaw = value.toFloat();
    else if (key =="K_pos2pwm")
      configData.K_pos2pwm = value.toFloat();
    else if (key == "failsafeThrottlePWM")
    {
      configData.failsafeThrottlePWM = value.toInt();
      altitudeData.sessionHoverPWM = configData.failsafeThrottlePWM; 
    }
    else if (key == "kp_altitude_rate") altitudeData.kp_altitude_rate = value.toFloat();
    else if (key == "ki_altitude_rate") altitudeData.ki_altitude_rate = value.toFloat();
    else if (key == "upGain") altitudeData.upGain = value.toFloat();
    else if (key == "targetRateLanding") altitudeData.targetRateLanding = value.toFloat();
    else if (key == "ceiling") altitudeData.ceiling = value.toFloat();
    else if (key == "trimPitch")
      configData.trimPitch = value.toFloat();
    else if (key == "trimYaw")
      configData.trimYaw = value.toFloat();
    else if (key == "trimRoll")
      configData.trimRoll = value.toFloat();
    else if (key =="throttleLimit")
    {
      configData.throttleLimit = value.toFloat();
      configData.throttleLimit = constrain(configData.throttleLimit,0.0,1.0); //protect from a fat finger
    }
    else if (key == "UP_COEFF")
    {
      configData.UP_COEFF = value.toFloat();
      configData.UP_COEFF = constrain(configData.UP_COEFF, 0.0, 1.0);
    }
    else if (key == "failsafeCoeff")
    {
      configData.failsafeCoeff = value.toFloat();
      configData.failsafeCoeff = constrain(configData.failsafeCoeff, 0.00001, 1.0);
    }
    else if (key == "DOWN_COEFF")
    {
      configData.DOWN_COEFF = value.toFloat();
      configData.DOWN_COEFF = constrain(configData.DOWN_COEFF, 0.0, 1.0);
    }
    else if (key == "action")
    {
      if (value.indexOf("SAVE TO") != -1) {
          saveParameters();
      }
    }
  }
}

void saveParameters()
{
  configData.throttleLimit = constrain(configData.throttleLimit,0.0,1.0); //protect from a fat finger
  configData.UP_COEFF = constrain(configData.UP_COEFF,0.0,1.0); //protect from a fat finger
  configData.DOWN_COEFF = constrain(configData.DOWN_COEFF,0.0,1.0); //protect from a fat finger
  configData.failsafeCoeff = constrain(configData.failsafeCoeff, 0.00001,1.0);
  prefs.begin("rawpter", false); // namespace "rawpter", RW mode
  prefs.putInt("fsThrottlePWM", configData.failsafeThrottlePWM);
  prefs.putFloat("ceiling", altitudeData.ceiling);
  prefs.putFloat("throttleLimit", configData.throttleLimit);
  prefs.putFloat("UP_COEFF", configData.UP_COEFF);
  prefs.putFloat("DOWN_COEFF", configData.DOWN_COEFF);
  prefs.putFloat("failsafeCoeff", configData.failsafeCoeff);
  prefs.putFloat("kpaltrate", altitudeData.kp_altitude_rate);
  prefs.putFloat("kialtrate", altitudeData.ki_altitude_rate);
  prefs.putFloat("upGain" , altitudeData.upGain);
  prefs.putFloat("targetRateLanding", altitudeData.targetRateLanding);
  prefs.putFloat("trimPitch", configData.trimPitch);
  prefs.putFloat("trimRoll", configData.trimRoll);
  prefs.putFloat("trimYaw", configData.trimYaw);
  prefs.putFloat("stick_dampener", configData.stick_dampener);
  prefs.putFloat("i_limit_angle", configData.i_limit_angle);
  prefs.putFloat("i_limit_rate", configData.i_limit_rate);
  prefs.putFloat("B_madgwick", configData.B_madgwick);
  prefs.putFloat("Accel_filter", configData.Accel_filter);
  prefs.putFloat("Gyro_filter", configData.Gyro_filter);
  prefs.putFloat("Kp_roll_rate", configData.Kp_roll_rate);
  prefs.putFloat("Ki_roll_rate", configData.Ki_roll_rate);
  prefs.putFloat("Kd_roll_rate", configData.Kd_roll_rate);
  prefs.putFloat("Kp_roll_angle", configData.Kp_roll_angle);
  prefs.putFloat("Ki_roll_angle", configData.Ki_roll_angle);
  prefs.putFloat("Kp_pitch_rate", configData.Kp_pitch_rate);
  prefs.putFloat("Ki_pitch_rate", configData.Ki_pitch_rate);
  prefs.putFloat("Kd_pitch_rate", configData.Kd_pitch_rate);
  prefs.putFloat("Kp_pitch_angle", configData.Kp_pitch_angle);
  prefs.putFloat("Ki_pitch_angle", configData.Ki_pitch_angle);
  prefs.putFloat("roll_maxRate", rollLimits.maxRate);
  prefs.putFloat("pitch_maxRate", pitchLimits.maxRate);
  prefs.putFloat("maxRoll", configData.maxRoll);
  prefs.putFloat("maxPitch", configData.maxPitch);
  prefs.putInt("rateControlMode", configData.rateControlMode);
  prefs.putFloat("maxYaw", configData.maxYaw);
  prefs.putFloat("K_pos2pwm", configData.K_pos2pwm);
  prefs.putFloat("Kp_yaw_rate", configData.Kp_yaw_rate);
  prefs.putFloat("Ki_yaw_rate", configData.Ki_yaw_rate);
  prefs.putFloat("Kd_yaw_rate", configData.Kd_yaw_rate);
  prefs.end(); // close namespace
  if (BENCH_TESTING) Serial.println("Free entries: " + String(prefs.freeEntries()));
}

void loadParameters()
{
  prefs.begin("rawpter", true); // namespace "rawpter", read-only

  // Use current variable values as defaults
  configData.failsafeThrottlePWM = prefs.getInt("fsThrottlePWM", configData.failsafeThrottlePWM);
  altitudeData.kp_altitude_rate = prefs.getFloat("kpaltrate", altitudeData.kp_altitude_rate);
  altitudeData.ki_altitude_rate = prefs.getFloat("kialtrate", altitudeData.ki_altitude_rate);
  altitudeData.targetRateLanding = prefs.getFloat("targetRateLanding", altitudeData.targetRateLanding);
  altitudeData.upGain = prefs.getFloat("upGain", altitudeData.upGain);
  altitudeData.ceiling = prefs.getFloat("ceiling", altitudeData.ceiling);
  configData.trimPitch = prefs.getFloat("trimPitch", configData.trimPitch);
  configData.trimRoll = prefs.getFloat("trimRoll", configData.trimRoll);
  configData.trimYaw = prefs.getFloat("trimYaw", configData.trimYaw);
  configData.throttleLimit = prefs.getFloat("throttleLimit", configData.throttleLimit);
  configData.throttleLimit = constrain(configData.throttleLimit,0.0,1.0); //protect from a fat finger
  configData.UP_COEFF = prefs.getFloat("UP_COEFF", configData.UP_COEFF);
  configData.DOWN_COEFF = prefs.getFloat("DOWN_COEFF", configData.DOWN_COEFF);
  configData.failsafeCoeff = prefs.getFloat("failsafeCoeff", configData.failsafeCoeff);
  configData.stick_dampener = prefs.getFloat("stick_dampener", configData.stick_dampener);
  configData.i_limit_angle = prefs.getFloat("i_limit_angle", configData.i_limit_angle);
  configData.i_limit_rate = prefs.getFloat("i_limit_rate", configData.i_limit_rate);
  configData.B_madgwick = prefs.getFloat("B_madgwick", configData.B_madgwick);
  configData.Accel_filter = prefs.getFloat("Accel_filter", configData.Accel_filter);
  configData.Gyro_filter = prefs.getFloat("Gyro_filter", configData.Gyro_filter);

  configData.Kp_roll_rate = prefs.getFloat("Kp_roll_rate", configData.Kp_roll_rate);
  configData.Ki_roll_rate = prefs.getFloat("Ki_roll_rate", configData.Ki_roll_rate);
  configData.Kd_roll_rate = prefs.getFloat("Kd_roll_rate", configData.Kd_roll_rate);
  configData.Kp_roll_angle = prefs.getFloat("Kp_roll_angle", configData.Kp_roll_angle);
  configData.Ki_roll_angle = prefs.getFloat("Ki_roll_angle", configData.Ki_roll_angle);
  configData.Kp_pitch_rate = prefs.getFloat("Kp_pitch_rate", configData.Kp_pitch_rate);
  configData.Ki_pitch_rate = prefs.getFloat("Ki_pitch_rate", configData.Ki_pitch_rate);
  configData.Kd_pitch_rate = prefs.getFloat("Kd_pitch_rate", configData.Kd_pitch_rate);
  configData.Kp_pitch_angle = prefs.getFloat("Kp_pitch_angle", configData.Kp_pitch_angle);
  configData.Ki_pitch_angle = prefs.getFloat("Ki_pitch_angle", configData.Ki_pitch_angle);

  rollLimits.maxRate = prefs.getFloat("roll_maxRate", rollLimits.maxRate);
  pitchLimits.maxRate = prefs.getFloat("pitch_maxRate", pitchLimits.maxRate);
  configData.maxRoll = prefs.getFloat("maxRoll", configData.maxRoll);
  configData.maxPitch = prefs.getFloat("maxPitch", configData.maxPitch);
  configData.rateControlMode = prefs.getInt("rateControlMode", configData.rateControlMode);
  configData.maxRoll = constrain(configData.maxRoll,5,70);
  configData.maxPitch = constrain(configData.maxPitch, 5, 70);
  configData.maxYaw = prefs.getFloat("maxYaw", configData.maxYaw);
  configData.K_pos2pwm = prefs.getFloat("K_pos2pwm", configData.K_pos2pwm);
  configData.Kp_yaw_rate = prefs.getFloat("Kp_yaw_rate", configData.Kp_yaw_rate);
  configData.Ki_yaw_rate = prefs.getFloat("Ki_yaw_rate", configData.Ki_yaw_rate);
  configData.Kd_yaw_rate = prefs.getFloat("Kd_yaw_rate", configData.Kd_yaw_rate);

  prefs.end();
  if (BENCH_TESTING) Serial.println("Parameters loaded from NVS (or defaults if none stored).");
}

void GenerateDefaultPage(WiFiClient &client) 
{ 
  if (BENCH_TESTING) Serial.println("About to MakeWebPage.");
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
  body += "<b>Snapshot:</b><br>";
  body += "<table>";
  body += "<tr><td>Desired Roll=" + String(pid.roll_des) + "&#176;</td><td>IMU Roll=" + String(imu.roll_IMU) + "&#176;</td></tr>";
  body += "<tr><td>Desired Pitch=" + String(pid.pitch_des) + "&#176;</td><td>IMU Pitch=" + String(imu.pitch_IMU) + "&#176;</td></tr>";
  body += "<tr><td>Loop Time=" + String(int(round((imu.madDeltaTime) / 1000000))) + "</td><td>Throttle PWM=" + String(radioData.PWM_throttle) + "</td></tr>";
  body += "<tr><td>Battery=" + String(batteryMonitor.calced_voltage, 1) + "V (" + String(batteryMonitor.batteryVoltage) + ")</td><td>Fastest Ascent=" + String(altitudeData.fastestAscent) + "</td></tr>";
  body += "<tr><td>Highest Altitude=" + String(altitudeData.highestAltitude) + "</td><td>Highest Throttle=" + String(telemetry.highestThrottlePWM) + "</td></tr>";
  body += "<tr><td>Lowest Throttle=" + String(telemetry.lowestThrottlePWM) + "</td><td>Battery=" + String(batteryMonitor.calced_voltage, 1) + "V (" + String(batteryMonitor.batteryVoltage) + ")</td></tr>";
  body += "<tr><td>Longitude = " + String(gps.longitude,2) + "&#176;</td><td>Latitude = " + String(gps.latitude, 2) + "&#176;</td></tr>";
  body += "<tr><td>GPS Fix=" + String(gps.fixType) +"</td><td>Altitude=" + String(altitudeData.altitude) + " ft</td></tr>";
  body += "<tr><td>PWM Yaw=" + String(radioData.PWM_yaw) +"</td><td>PWM Row=" + String(radioData.PWM_roll) + " ft</td></tr>";
  body += "<tr><td>PWM Pitch=" + String(radioData.PWM_pitch) +"</td></tr>";
  body += "</table><hr>";
  body += "<b>PID Constants:</b><br>";
  if (batteryMonitor.calced_voltage < 13) { body += "<br><div class='alert alert-danger'>DANGER: BATTERY CRITICAL!</div><br>"; } else if (batteryMonitor.calced_voltage < 14) { body += "<br><div class='alert alert-warning'>Warning: BATTERY LOW!</div><br>"; }
  // Parameters table
  body += "<table class=table><thead class=thead-dark><th></th><th>Kp</th><th>Ki</th><th>Kd</th></thead>";
  // Roll row 
  body += "<tr><td>Roll Angle:</td><td><input name=kp_roll_angle style='width:80px;' type=number step=.0001 value='" +
  String(configData.Kp_roll_angle, 4) + "'></td><td><input name=ki_roll_angle style='width:80px;' type=number step=.0001 value='" +
  String(configData.Ki_roll_angle, 4) + "'></td></tr>";
  body += "<tr><td>Roll Rate:</td><td><input name=kp_roll_rate style='width:80px;' type=number step=.0001 value='" +
  String(configData.Kp_roll_rate, 4) + "'></td><td><input style='width:80px;' name=ki_roll_rate type=number step=.0001 value='" +
  String(configData.Ki_roll_rate, 4) + "'></td><td><input name=kd_roll_rate type=number step=.00001 style='width:100px;' value='" +
  String(configData.Kd_roll_rate, 5) + "'></td></tr>";
  // Pitch row 
  body += "<tr><td>Pitch Angle:</td><td><input name=kp_pitch_angle style='width:80px;' type=number step=.0001 value='" +
  String(configData.Kp_pitch_angle, 4) + "'></td><td><input style='width:80px;' name=ki_pitch_angle type=number step=.0001 value='" +
  String(configData.Ki_pitch_angle, 4) + "'></td></tr>";
  body += "<tr><td>Pitch Rate:</td><td><input name=kp_pitch_rate style='width:80px;' type=number step=.0001 value='" +
  String(configData.Kp_pitch_rate, 4) + "'></td><td><input style='width:80px;' name=ki_pitch_rate type=number step=.0001 value='" +
  String(configData.Ki_pitch_rate, 4) + "'></td><td><input name=kd_pitch_rate type=number step=.00001 style='width:100px;' value='" +
  String(configData.Kd_pitch_rate, 5) + "'></td></tr>";
  // Yaw row
  body += "<tr><td>Yaw Rate:</td><td><input name=kp_yaw_rate type=number step=0.0001 style='width:80px;' value='" +
  String(configData.Kp_yaw_rate, 4) + "'></td><td><input style='width:80px;' type=number step=.0001 name=ki_yaw_rate value='" +
  String(configData.Ki_yaw_rate, 4) + "'></td><td><input type=number step=.00001 name=kd_yaw_rate style='width:100px;' value='" +
  String(configData.Kd_yaw_rate, 5) + "'></td></tr>";
  // Integral limit row
  body += "<tr><td>Angle Integral Max:</td><td><input type=number step=.0001 name=i_limit_angle style='width:90px;' value='" + String(configData.i_limit_angle) + "'></td>";
  body += "<td>Rate Integral Max:</td><td><input type=number step=.0001 name=i_limit_rate style='width:90px;' value='" + String(configData.i_limit_rate) + "'></td></tr>";
  body += "<tr><td>B_Madgwick (0.03 default):</td><td><input type=number step=.0001 name=B_madgwick style='width:90px;' value='" +
  String(configData.B_madgwick) + "'></td></tr>";
  body += "</table><br>";
  body += "<b>Rate and Angle Limits:</b><br>";
  body += "<table class=table>";
  body += "<thead class=thead-dark><th></th><th>Max Rate</th></thead>";
  body += "<tr><td>Rate Control Mode:</td><td>"
        "<input type='hidden' name='rateControlMode' value='0'>"
        "<input type='checkbox' name='rateControlMode' value='1' " 
        + String(configData.rateControlMode ? "checked" : "") + ">"
        "</td></tr>";
  body += "<tr><td>Roll Rate (deg/s):</td><td><input name=roll_maxRate style='width:90px;' type=number value='" + String(rollLimits.maxRate) + "'></td>";
  body += "<td>Roll (max angle):</td><td><input name=maxRoll style='width:90px;' type=number value='" + String(configData.maxRoll) + "'></td></tr>";
  body += "<tr><td>Pitch Rate (deg/s):</td><td><input name=pitch_maxRate style='width:90px;' type=number value='" + String(pitchLimits.maxRate) + "'></td>";
  body += "<td>Pitch (max angle):</td><td><input name=maxPitch style='width:90px;' type=number value='" + String(configData.maxPitch) + "'></td></tr>";
  body += "<tr><td>Yaw Rate (deg/s):</td><td><input name=maxYaw style='width:90px;' type=number value='" + String(configData.maxYaw) + "'></td>";
  body += "<td>Home Pitch Gain (K_pos2pwm):</td><td><input name=K_pos2pwm style='width:90px;' step=.1 type=number value='" + String(configData.K_pos2pwm,1) + "'></td></tr>";
  body += "</table>";
  body += "<br>Tip: Before powering down, Save to Storage - but if just experimenting during the session hold off saving to extend the Flash Storage life.<br>";
  // Additional parameters table 
  body += "<table>";
  body += "<tr><td>Ceiling:</td><td><input type=number name=ceiling style='width:80px;' step = .1 value='" + String(altitudeData.ceiling,1) + "'></td></tr>";
  body += "<tr><td>Land Mode Start Throttle (1500 to 2000):</td><td><input type=number name=failsafeThrottlePWM style='width:80px;' value='" + String(configData.failsafeThrottlePWM) + "'></td></tr>";
  body += "<tr><td>Landing Target Rate (ft/sec):</td><td><input type=number name=targetRateLanding style='width:100px;' step = .01 value='" + String(altitudeData.targetRateLanding,2) + "'></td></tr>";
  body += "<tr><td>Landing Up Gain Multiplier:</td><td><input type=number name=upGain style='width:100px;' step = .01 value='" + String(altitudeData.upGain,2) + "'></td></tr>";
  body += "<tr><td>Kp Altitude Rate:</td><td><input type=number name=kp_altitude_rate style='width:100px;' step = .00001 value='" + String(altitudeData.kp_altitude_rate,5) + "'></td></tr>";
  body += "<tr><td>Ki Altitude Rate:</td><td><input type=number name=ki_altitude_rate style='width:100px;' step = .00001 value='" + String(altitudeData.ki_altitude_rate,5) + "'></td></tr>";
  body += "<tr><td>Trim - Pitch (-500 to 500):</td><td><input type=number name=trimPitch style='width:80px;' value='" + String(configData.trimPitch) + "'></td></tr>";
  body += "<tr><td>Trim - Roll (-500 to 500):</td><td><input type=number name=trimRoll style='width:80px;' value='" + String(configData.trimRoll) + "'></td></tr>";
  body += "<tr><td>Trim - Yaw (-500 to 500):</td><td><input type=number name=trimYaw style='width:80px;' value='" + String(configData.trimYaw) + "'></td></tr>";
  body += "<tr><td>Throttle Limit (0.01-1.0):<br>0.1=slow, 1.0=fast</td><td><input type=number step=0.01 name=throttleLimit style='width:80px;' value='" +  String(configData.throttleLimit, 2) + "'></td></tr>";
  body += "<tr><td>Up Dampening (0.001-1.0):<br>0.1=slow, 1.0=fast</td><td><input type=number step=0.001 name=UP_COEFF style='width:80px;' value='" +  String(configData.UP_COEFF, 3) + "'></td></tr>";
  body += "<tr><td>Down Dampening (0.00001-1.0):<br>0.1=slow, 1.0=fast</td><td><input type=number step=0.00001 name=DOWN_COEFF style='width:80px;' value='" +  String(configData.DOWN_COEFF, 5) + "'></td></tr>";
  body += "<tr><td>Failsafe Down Dampening (0.00001-0.03):<br>slow to fast<br>default 0.0006</td><td><input type=number step=0.00001 name=failsafeCoeff style='width:100px;' value='" +  String(configData.failsafeCoeff, 6) + "'></td></tr>";
  body += "<tr><td>Stick Dampening (0.01-1.0):<br>0.1=slow/steady, 1.0=noisy/fast</td><td><input type=number step=0.001 name=stick_dampener style='width:80px;' value='" +  String(configData.stick_dampener, 3) + "'></td></tr>";
  body += "<tr><td>Accel Dampening (0.1-1.0):<br>0.1=slow/steady, 1.0=noisy/fast</td><td><input type=number step=0.01 name=Accel_filter style='width:80px;' value='" + String(configData.Accel_filter) + "'></td></tr>";
  body += "<tr><td>Gyro Dampening (0.1-1.0):<br>0.1=slow/steady, 1.0=noisy/fast</td><td><input type=number step=0.01 name=Gyro_filter style='width:80px;' value='" + String(configData.Gyro_filter) + "'></td></tr>";
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
  if (BENCH_TESTING) Serial.println("Made Web Page.");
}
