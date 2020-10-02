# MCP2221 Sensors Studio
The MCP2221 Sensors studio is a desktop app that acquires data from analog inputs, MMA8452Q 3-Axes accelerometer, MPU9250 9-DOF IMU and uBlox Neo-6 GPS module via MCP2221 USB-to-UART/I2C bridge chip. The main purpose of the project is to have an "Easy-to-use" technology demonstrator for educators and hobbyists who want to learn about different motion and/or location sensors.
This project is basically a collection of MCP2221 projects I have created in the past, all combined together. These are the original projects:
- Qt Geolocator (uBlox Neo-6 GPS interface with MCP2221): : https://github.com/kelray/Qt-Geolocator
- MPU9250 9DOF IMU USB Viewer: https://github.com/kelray/USB-9-DOF-IMU-Viewer
- USB 3D Accelerometer Viewer: https://github.com/kelray/USB-3D-Accelerometer-Viewer

The project requires the following tools and libraries in order to compile and build:
- Qt framework (version 15.5.0 is the one I used for this project), with MingW C/C++ compiler. Visual Studio can be used as a replacement.
- MapGraphics, a C++ maps Library: https://github.com/raptorswing/MapGraphics
- Microchip MCP2221 library and DLL for Windows OS (included in the project files).

**Worth noting:**
- The maps library used in this project differs from the one used in "Qt Geolocator" project back from 2018. In this project, MapGraphics has been used since it showed faster response in testing.
- The app works with MPU6050 6-DOF IMU since it shares with the MPU9250 the same registers set for the accelerometer and gyroscope, and of course the magnetometer data is gibberish since it doesn't have one. The full-scale ranges are different as well, yet the app is capable of collecting acceleration and gyration data.

I would like to credit the following libraries, the MMA8452Q and MPU9250 drivers for the MCP2221 are mostly derived from them: 
- MPU9250 library: https://github.com/bolderflight/MPU9250
- SparkFun MMA8452Q Arduino library: https://github.com/sparkfun/SparkFun_MMA8452Q_Arduino_Library
- Arduino and MMA8452 sensor example: http://arduinolearning.com/code/arduino-and-mma8452-sensor-example.php

If you are new to the MCP2221 USB to UART/I2C bridge, the following tutorials can be your starting point:
- MCP2221 Tutorial - I2C Interfacing made easy: 
http://elrayescampaign.blogspot.com/2018/06/mcp2221-i2c-interfacing-tutorial.html
- MCP2221 Tutorial - USB interfacing made easy: 
http://elrayescampaign.blogspot.com/2016/06/mcp2221-tutorial-easy-usb-interfacing.html

**If you don't want to re-compile the project**, Windows executable and DLLs are available in "build-SensorsStudioV2-Desktop_Qt_5_15_0_MinGW_32_bit-Release.zip". Download and unzip it, and the exectutable will be ready to run.

**MMA8452Q module (picture from SparkFun):**

<p align="center"> <img width="450" alt="mma8452q" src="https://user-images.githubusercontent.com/8460504/94381563-aaa3a580-00ee-11eb-9a1f-785db2440a4b.jpg">
  
**MPU9250 module (picture from Amazon.ca):**

<p align="center"> <img width="450" alt="mpu9250_module" src="https://user-images.githubusercontent.com/8460504/94381565-abd4d280-00ee-11eb-82c4-b9bc24ea9f3c.jpeg">

**uBlox Neo-6 GPS module:**

<p align="center"> <img width="450" alt="ublox neo 6 gps" src="https://user-images.githubusercontent.com/8460504/94381566-ac6d6900-00ee-11eb-8135-9399fac4878d.jpg">

**Screenshots:**

**MPU9250 Tab:**

<p align="center"> <img width="676" alt="Sensors Studio_v2_mpu9250 tab" src="https://user-images.githubusercontent.com/8460504/94498974-5cef7180-01b0-11eb-87a5-bccf73e2cecb.png">

**MMA8452 3-axis accelerometer and uBlox Neo-6 GPS Tab:**

<p align="center"> <img width="676" alt="Sensors Studio_v2_mma8452 tab" src="https://user-images.githubusercontent.com/8460504/94498971-5c56db00-01b0-11eb-8888-fe8a7d9421eb.png">

**Analog Inputs Tab:**

<p align="center"> <img width="676" alt="Sensors studio v22_3" src="https://user-images.githubusercontent.com/8460504/94200569-74a4be00-fe6f-11ea-98c4-ad58e0333d5c.png">
  
**License:**

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain this copyright notice,
      this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are
 those of the author and should not be interpreted as representing official
 policies, either expressed or implied, by the author.
 
Please let me know if you notice any errors or problems with the source code.

**Disclaimer: Cannot be used for any life-critical application, use it at your own risk.**
