// Pressure
// Create a new sensor object

#ifndef DATATYPES_H
#define DATATYPES_H
#include <SparkFun_BMP581_Arduino_Library.h> // BMP581 Pressure Sensor
#include <SparkFun_u-blox_GNSS_v3.h>         // Max10S GPS
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

// Radio PPM
#define PPM_PIN 21    // input pin for PPM
#define CHANNELS 6    // number of channels
#define BLANK_US 4000 // blank time threshold in µs

// Motors
#define m1Pin 1 // IO1
#define m2Pin 3 // IO3
#define m3Pin 5 // IO5
#define m4Pin 4 // IO4

struct AltitudeData
{
  BMP581 pressureSensor;
  uint8_t i2cAddress = 0x46;      // 0x46 is the Pressure Sensor 0x42 is the GPS
  volatile float altitude = 0.0f; // just a default until it starts to loop.
  float highestAltitude = 0.0f;
  float ceiling = 12.0f;
  volatile float rateFPS = 0.0f;         // Used to ensure it doesn't descend too quickly
  volatile float rateFPSWorking = 0.0f;  // This one is worked by the pressure task and then synced to rateFPS variable in the main loop.
  volatile float altitudeWorking = 0.0f; // This one is worked by the pressure task and then synced to altitude variable in the main loop.
  float maxAltitudeChangeRate = .8f;
  float kp_altitude_rate = .0001f;
  float ki_altitude_rate = .0001f;
  volatile float invGroundPressure = 0.0f;        // reciprocal of baseline pressure at startup
  volatile float invGroundPressureWorking = 0.0f; // This one is worked by the pressure task and then synced to invGroundPressure variable in the main loop.
  TaskHandle_t bmpTaskHandle= nullptr;                     // Handle for the BMP581 task that will run on its own core.
  bool hasBMP581 = false;
  float sessionHoverPWM =1600;
  float targetRateLanding = -0.1f; // ft/sec
  float upGain = 4.0f; // How hared to push back up if descend too quickly on landing
  float fastestAscent = 0.0f; // ft/sec
};

// GPS
struct GPSData
{
  bool hasGPS = false;
  bool useGPS = false;
  SFE_UBLOX_GNSS Max10SGPS;
  double longitude = 0;
  double latitude = 0;
  double longitudeWorking = 0;
  double latitudeWorking = 0;
  uint8_t fixType = 0;
  bool atHome = false;
};

struct HomePosition
{
    bool valid = false;
    double lat_deg = 0;
    double lon_deg = 0;
};

struct Limits
{  // Rate Limits - need to expand this to bring in the K constants and desired setpoints
  float maxRate; // deg/s
};
#endif


