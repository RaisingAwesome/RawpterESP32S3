// Project.h
// Library dependencies for the complete project
#pragma once

#include "DataTypes.h"
#include "Constants.h"
#include <SPI.h>                             // SPI Communication Support for IMU
#include <WiFi.h>                            // ESP32 Adhoc WiFi Support
#include <WebServer.h>
#include <Wire.h>                            // I2C Communication for GPS and Pressure Sensor
#include <functional>
#include "driver/gpio.h"                     // ESP32-S3 routines to speed up digitalWrite
#include "driver/rmt_rx.h"                   // ESP32-S3 routines for remote PPM pulse capture
#include "RmtPPMReader.h"                    // Raising Awesome's PPM pulse capture code
#include "driver/mcpwm_prelude.h"            // ESP32-S3 routines for motor control
#include <Preferences.h>
#include <SparkFun_BMP581_Arduino_Library.h> // BMP581 Pressure Sensor
#include <SparkFun_u-blox_GNSS_v3.h>         // Max10S GPS