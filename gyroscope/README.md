# Gyroscope

## Hardware

MPU-6050 Modul 3-Axis-Gyroscope and Acceleration Sensor

## Connection

- VCC -> 3V Power (1)
- GND -> Ground
- SDA -> SDA (3)
- SCL -> SCL (5)

## Setup

1. Activation of I2C:
   Enter `sudo raspi-config` and select `5 - Interfacing Options` > `P5 - I2C` and confirm the activation
2. Edit the `/etc/modules` file to contain the following entries `i2c-bcm2708` and `i2c-dev`
3. Install the software modules needed via `sudo apt-get install i2c-tools python-smbus`
4. Make sure the Gyroscope is connected correctly and run `sudo i2cdetect -y 1`. If the connection is working, there should be a device at address 0x68
5. Connect to the device via sudo `i2cget -y 1 0x68 0x75`

## Python Module used

smbus
