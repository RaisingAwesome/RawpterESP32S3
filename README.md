# INTRODUCTION
Rawpter is a custom flight controller for a quadcopter. It is based on the ESP32-S3. It makes use of the ESP32's ability to run tasks on independent cores and multithreaded. In turn, it achieves a fast 2K inner loop on 32kHz downsampled gyro data.

# Hardware
- Microcontroller: ESP32-S3-Mini-1U
- IMUI: ICM 40906 (6 axis accelerometer and gyro, SPI)
- Altitude: BMP581 pressure sensor (I2C)
- GPS: Ublox Max-10S (I2C)
- PCB: Custom Design

# Features
- 400Hz Outer Loop Angle PI controller based on remote radio sticks.
- 2K Inner loop using cascaded angles setpoints from outloop and IMU data to control angular velocity on each axis
- 400Hz Command Mixer to drive Simonk Firmware driven ESCs
- Motor PWM driven by ESP32 LEDC @ 400Hz, 16Bit resolution
- 6 Channel PPM Remote Receiver Hardware Pulse Capture using rmtRead @ 50HZ PPM Frame Duration per Flysky FS-i6X Transmitter
- Two cores strategically orchestrated with FreeRTOS
- Clever variable management with FreeRTOS Task Notification and double buffering to ensure no corrupt data from multiple threads across cores
- Clever Radio Failsafe to automatically descend safely
- Battery Voltage monitoring and buzzer signalling
- iBus Communication to Radio Transmitter
- GPS (TBD)

  
