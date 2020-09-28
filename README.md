# MCP2221 Sensors Studio
The MCP2221 Sensors studio is a desktop app that interfaces with MCP2221 analog inputs, MMA8452Q 3-Axis accelerometer, MPU9250 9 DOF IMU, and uBlox Neo-6 GPS module. The main purpose of the project is to have an "Easy-to-use technology demonstrator" for educators and hobbyists who want to learn about different motion/location sensors.
This project is basically a collection of MCP2221 projects, I have created in the past, all combined together, these are the original projects:
- Qt Geolocator (uBlox Neo-6 GPS interface with MCP2221): : https://github.com/kelray/Qt-Geolocator
- MPU9250 9DOF IMU USB Viewer: https://github.com/kelray/USB-9-DOF-IMU-Viewer
- USB 3D Accelerometer Viewer: https://github.com/kelray/USB-3D-Accelerometer-Viewer

The project requires the following libraries in order to compile and build:
- Qt framework (version 15.5.0 is the one I used for this project).
- QMapControl Qt maps library: https://github.com/kaiwinter/QMapControl
- Microchip MCP2221 library and DLL for Windows OS (included in the project files).

I would like to credit the following libraries, the MMA8452Q and MPU9250 drivers were mostly derived from them: 
- MPU9250 library: https://github.com/bolderflight/MPU9250
- SparkFun MMA8452Q Arduino library: https://github.com/sparkfun/SparkFun_MMA8452Q_Arduino_Library
- Arduino and MMA8452 sensor example blog post listed here: http://arduinolearning.com/code/arduino-and-mma8452-sensor-example.php

If you are new to the MCP2221 USB to UART/I2C bridge, the following tutorials can be your starting point:
- MCP2221 Tutorial - I2C Interfacing made easy: http://elrayescampaign.blogspot.com/2018/06/mcp2221-i2c-interfacing-tutorial.html
- MCP2221 Tutorial - USB interfacing made easy: http://elrayescampaign.blogspot.com/2016/06/mcp2221-tutorial-easy-usb-interfacing.html

**MMA8452Q module (picture from SparkFun):**

<p align="center"> <img width="450" alt="mma8452q" src="https://user-images.githubusercontent.com/8460504/94381563-aaa3a580-00ee-11eb-9a1f-785db2440a4b.jpg">
  
**MPU9250 module (picture from Amazon.ca):**

<p align="center"> <img width="450" alt="mpu9250_module" src="https://user-images.githubusercontent.com/8460504/94381565-abd4d280-00ee-11eb-82c4-b9bc24ea9f3c.jpeg">

**uBlox Neo-6 GPS module:**

<p align="center"> <img width="450" alt="ublox neo 6 gps" src="https://user-images.githubusercontent.com/8460504/94381566-ac6d6900-00ee-11eb-8135-9399fac4878d.jpg">

**Screenshots:**

**MPU9250 Tab:**

<img width="676" alt="Sensors studio v22_1" src="https://user-images.githubusercontent.com/8460504/94200565-740c2780-fe6f-11ea-89b4-fcb1b612ebd5.png">

**MMA8452 3-axis accelerometer and uBlox Neo-6 GPS Tab:**

<img width="676" alt="Sensors studio v22_2" src="https://user-images.githubusercontent.com/8460504/94200567-74a4be00-fe6f-11ea-89c7-6aa7bc18cdb2.png">

**Analog Inputs Tab:**

<img width="676" alt="Sensors studio v22_3" src="https://user-images.githubusercontent.com/8460504/94200569-74a4be00-fe6f-11ea-98c4-ad58e0333d5c.png">
