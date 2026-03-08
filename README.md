# INTRODUCTION
Rawpter is a custom flight controller for a quadcopter. It is based on the ESP32-S3. It makes use of the ESP32's ability to run tasks on independent cores and multithreaded. In turn, it achieves a fast 1K inner loop on 32kHz downsampled gyro data.

# Hardware
- Microcontroller: ESP32-S3-Mini-1U
- IMU: ICM 40609 (6 axis accelerometer and gyro, SPI)
- Altitude: BMP581 pressure sensor (I2C)
- GPS: Ublox Max-M10S (I2C)
- PCB: Custom Design

# Features
- Web UI for field input and adjustment of all flight controller parameters through your phone
- 1K Inner loop using cascaded angles setpoints from outloop and IMU data to control angular velocity on each axis
- 250Hz Outer Loop Angle PI controller based on remote radio sticks
- 250Hz Command Mixer to drive Simonk Firmware driven ESCs
- Hardware Motor Control PWM (MCPWM) @ 250Hz with offset timers to dampen amperage surges
- 8 Channel PPM Remote Receiver Hardware Pulse Capture using rmtRead @ 50HZ PPM Frame Duration per Flysky FS-i6X Transmitter. Non-blocking, hardware driven
- GPS Autonomous Return-to-Location and Landing
- Failsafe Autonomous Landing and Watchdog Reset
- Two cores strategically orchestrated with FreeRTOS
- Clever variable management with FreeRTOS Task Notification and double buffering to ensure no corrupt data from multiple threads across cores for sensor fusion
- Clever Radio Failsafe to automatically descend safely via optional GPS return to location
- Battery Voltage monitoring and buzzer signalling
- iBus Communication from Radio Transmitter
