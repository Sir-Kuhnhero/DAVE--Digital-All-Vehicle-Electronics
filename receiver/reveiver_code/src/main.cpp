// 
    // Updates should (hopefully) always be available at https://github.com/Sir-Kuhnhero/DAVE--Digital-All-Vehicle-Electronics
    //
    // Changelog:
    //      

    /* ============================================
    DAVE code is placed under the MIT license. This does not include code provided by a library covered under the original author's license.
    Copyright (c) 2022 Sir-Kuhnhero

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
    ===============================================
    */

#pragma region AccelStepper
    // Copyright (C) 2009 Mike McCauley
    // $Id: MultiStepper.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $
#pragma endregion
#pragma region NRF24
    /*
     * See documentation at https://nRF24.github.io/RF24
     * See License information at root directory of this library
     * Author: Brendan Doherty (2bndy5)
     */
#pragma endregion
#pragma region MPU6050
    // I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
    // 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
    // Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
    //
    // Changelog:
    //      2013-05-08 - added seamless Fastwire support
    //                 - added note about gyro calibration
    //      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
    //      2012-06-20 - improved FIFO overflow handling and simplified read process
    //      2012-06-19 - completely rearranged DMP initialization code and simplification
    //      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
    //      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
    //      2012-06-05 - add gravity-compensated initial reference frame acceleration output
    //                 - add 3D math helper file to DMP6 example sketch
    //                 - add Euler output and Yaw/Pitch/Roll output formats
    //      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
    //      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
    //      2012-05-30 - basic DMP initialization working

    /* ============================================
    I2Cdev device library code is placed under the MIT license
    Copyright (c) 2012 Jeff Rowberg

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
    ===============================================
    */
#pragma endregion
#pragma region BMP280
    /***************************************************************************
      This is a library for the BMP280 humidity, temperature & pressure sensor
      This example shows how to take Sensor Events instead of direct readings

      Designed specifically to work with the Adafruit BMP280 Breakout
      ----> http://www.adafruit.com/products/2651

      These sensors use I2C or SPI to communicate, 2 or 4 pins are required
      to interface.

      Adafruit invests time and resources providing this open source code,
      please support Adafruit and open-source hardware by purchasing products
      from Adafruit!

      Written by Limor Fried & Kevin Townsend for Adafruit Industries.
      BSD license, all text above must be included in any redistribution
     ***************************************************************************/
#pragma endregion
#pragma region HMC5883
    /*!
    * @file getCompassdata.ino
    * @brief Output the compass data
    * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
    * @license     The MIT License (MIT)
    * @author      [dexian.huang](952838602@qq.com)
    * @version  V1.0
    * @date  2017-7-3
    * @url https://github.com/DFRobot/DFRobot_QMC5883
    */

       /**
    * @brief  Set declination angle on your location and fix heading
    * @n      You can find your declination on: http://magnetic-declination.com/
    * @n      (+) Positive or (-) for negative
    * @n      For Bytom / Poland declination angle is 4'26E (positive)
    * @n      Formula: (deg + (min / 60.0)) / (180 / PI);
    */
#pragma endregion

#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include <AccelStepper.h>
#include <I2Cdev.h>
#include "Wire.h"

//#include "radio.h"
//#include "sensor.h"
//#include "sd.h"
#include "header.h"


//#define SERVO
//#define STEPPER
#ifdef SD_Card
    #define dataLogging
    #ifdef dataLogging
        #define NRF24_LOG
        //#define READ_VOLTAGE_LOG
        //#define IMU_LOG
        //#define BMP280_LOG
        //#define HMC5883_LOG
    #endif
#endif

#define DEBUG
#ifdef DEBUG
    #define SERIAL_OUTPUT  // if defined all sensor data will be printed
#endif

char LED_PIN = 23;
bool LED_state = false;

int loopTime;


#ifdef SERVO
    struct chServo {
      Servo servo;
      int value = 128;
      int minValue = 0;
      float valueScaler = 1;
      char pin;
      bool valueAsAngle = false;
    } chServo[3];
#endif

#ifdef STEPPER
    struct chStepper {
      AccelStepper stepper;
      int value = 128;
      int minValue = 0;
      float valueScaler = 4;
      float maxSpeed, acceleration;
      int stepPin, dirPin, enablePin;
      bool valueAsAngle = false;
      bool active = true;
    } chStepper[3];
#endif


void updateOutputChannels() {
  #ifdef SERVO
      // Servo / ESC
      for (int i = 0; i < int (sizeof(chServo) / sizeof(chServo[0])); i++) {
        int value = int (chServo[i].value * chServo[i].valueScaler) + chServo[i].minValue;

        if (!chServo[i].valueAsAngle) {
          value = map(value, 0, 255, 0, 180);
        }

        chServo[i].servo.write(value);
      }
  #endif

  #ifdef STEPPER
      // Stepper
      for (int i = 0; i < int (sizeof(chStepper) / sizeof(chStepper[0])); i++) {
        if (!chStepper[i].active) {
          digitalWrite(chStepper[i].enablePin, HIGH);
        }
        else {
          digitalWrite(chStepper[i].enablePin, LOW);

          int value = int (chStepper[i].value * chStepper[i].valueScaler) + chStepper[i].minValue;

          if (chStepper[i].valueAsAngle) {
            chStepper[i].stepper.moveTo(value);
            chStepper[i].stepper.run();
          }
          else {
            chStepper[i].stepper.setSpeed(value - int (128 * chStepper[i].valueScaler));
            chStepper[i].stepper.runSpeed();
          }
        }
      }
  #endif
}

void criticalError(int errorCode) {
  // this funktion will keep on looping until an input to the Serial line restarts the Teensy
  if (Serial.available()) {
    Serial.println("restart");
    Serial.clear();
    setup();
    return;
  }
  
  Serial.println(errorCode);
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  
  delay(500);

  criticalError(errorCode);
}

void outputDataOverSerial() {
  #ifdef SERIAL_OUTPUT
      #pragma region displayLoopTime
          Serial.print("loopTime: ");
          Serial.print(loopTime);

          if (loopTime < 10)
            Serial.print("   ");
          else if (loopTime < 100)
            Serial.print("  ");
          Serial.print(" || ");
      #pragma endregion

      #ifdef NRF24
          Serial.print("reviveTime: ");
          Serial.print(receiveTime);
          Serial.print(" || ");

          for (int i = 0; i < int (sizeof(data_receive.channel) / sizeof(data_receive.channel[0])); i++) {
            Serial.print("ch: ");
            Serial.print(i);
            Serial.print(" - ");
            Serial.print(data_receive.channel[i]);

            if (data_receive.channel[i] < 10)
              Serial.print("  ");
            else if (data_receive.channel[i] < 100)
              Serial.print(" ");
            Serial.print(" || ");
          }
      #endif

      #ifdef VOLTAGE
          for (int i = 0; i < int (sizeof(chVoltage) / sizeof(chVoltage[0])); i++) {
            Serial.print("v: ");
            Serial.print(i);
            Serial.print(" - ");
            Serial.print(chVoltage[i].value);
            Serial.print(" || ");
          }
      #endif

      #ifdef IMU
          #ifdef OUTPUT_READABLE_QUATERNION
              // display quaternion values in easy matrix form: w x y z
              Serial.print("quat: ");
              Serial.print(q.w);
              Serial.print(", ");
              Serial.print(q.x);
              Serial.print(", ");
              Serial.print(q.y);
              Serial.print(", ");
              Serial.print(q.z);
              Serial.print(" || ");
          #endif

          #ifdef OUTPUT_READABLE_EULER
              // display Euler angles in degrees
              Serial.print("euler: ");
              Serial.print(euler[0] * 180/M_PI);
              Serial.print(", ");
              Serial.print(euler[1] * 180/M_PI);
              Serial.print(", ");
              Serial.print(euler[2] * 180/M_PI);
              Serial.print(" || ");
          #endif

          #ifdef OUTPUT_READABLE_YAWPITCHROLL
              // display Euler angles in degrees
              Serial.print("ypr: ");
              Serial.print(ypr[0] * 180/M_PI);
              Serial.print(", ");
              Serial.print(ypr[1] * 180/M_PI);
              Serial.print(", ");
              Serial.print(ypr[2] * 180/M_PI);
              Serial.print(" || ");
          #endif

          #ifdef OUTPUT_READABLE_REALACCEL
              // display real acceleration, adjusted to remove gravity
              Serial.print("areal: ");
              Serial.print(aaReal.x);
              Serial.print(", ");
              Serial.print(aaReal.y);
              Serial.print(", ");
              Serial.print(aaReal.z);
              Serial.print(" || ");
          #endif

          #ifdef OUTPUT_READABLE_WORLDACCEL
              // display initial world-frame acceleration, adjusted to remove gravity
              // and rotated based on known orientation from quaternion
              Serial.print("aworld: ");
              Serial.print(aaWorld.x);
              Serial.print(", ");
              Serial.print(aaWorld.y);
              Serial.print(", ");
              Serial.println(aaWorld.z);
              Serial.print(" || ");
          #endif
      #endif

      #ifdef BMP280
          Serial.print(F("Temperature = "));
          Serial.print(temp_event.temperature);
          Serial.print(" *C");
          Serial.print(" || ");

          Serial.print(F("Pressure = "));
          Serial.print(pressure_event.pressure);
          Serial.print(" hPa");
          Serial.print(" || ");
      #endif

      #ifdef HMC5883
          Serial.print("X:");
          Serial.print(mag.XAxis);
          Serial.print(" Y:");
          Serial.print(mag.YAxis);
          Serial.print(" Z:");
          Serial.print(mag.ZAxis);
          Serial.print(" | ");
          Serial.print("Degress = ");
          Serial.print(mag.HeadingDegress);
          Serial.print(" || ");
      #endif

      Serial.println();
  #endif
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  Serial.begin(9600);

  analogReadRes(12);

  pinMode(LED_PIN, OUTPUT);
  
  #ifdef DEBUG
      // wait for USB connection
      while (!Serial) {
            delay(100);
      }

      Serial.println("press any button to start");

      // wait for serial input
      while (!Serial.available()) {
            delay(100);
      }
      Serial.clear();
  #endif


  #ifdef NRF24
      if (!NRF_init()) {
        criticalError(0);
      }
  #endif

  #ifdef SERVO
      // configure channels
      chServo[0].pin = 3;
      chServo[1].pin = 4;
      chServo[2].pin = 5;
      chServo[3].pin = 6;
      chServo[4].pin = 9;
      chServo[5].pin = 10;
      chServo[6].pin = 14;
      chServo[7].pin = 15;
      chServo[8].pin = 33;
      chServo[9].pin = 36;
      chServo[10].pin = 37;

      for (int i = 0; i < int (sizeof(chServo) / sizeof(chServo[0])); i++) {
        chServo[i].servo.attach(chServo[i].pin, 1000, 2000);
      }
  #endif

  #ifdef STEPPER
      chStepper[0].stepPin = 20;
      chStepper[0].dirPin = 21;
      chStepper[0].enablePin = 22;
      chStepper[1].stepPin = 30;
      chStepper[1].dirPin = 31;
      chStepper[1].enablePin = 32;
      chStepper[2].stepPin = 34;
      chStepper[2].dirPin = 35;
      chStepper[2].enablePin = 38;
      chStepper[3].stepPin = 39;
      chStepper[3].dirPin = 40;
      chStepper[3].enablePin = 41;

      chStepper[0].maxSpeed = 2000.00;
      chStepper[1].maxSpeed = 2000.00;
      chStepper[2].maxSpeed = 2000.00;
      chStepper[3].maxSpeed = 2000.00;
      chStepper[0].acceleration = 75.00;
      chStepper[1].acceleration = 75.00;
      chStepper[2].acceleration = 75.00;
      chStepper[3].acceleration = 75.00;

      for (int i = 0; i < int (sizeof(chStepper) / ( sizeof(chStepper[0]))); i++) {
        pinMode(chStepper[i].enablePin, OUTPUT);
        AccelStepper stepperTemp(1, chStepper[i].stepPin, chStepper[i].dirPin);
        stepperTemp.setMaxSpeed(chStepper[i].maxSpeed);
        stepperTemp.setAcceleration(chStepper[i].acceleration);
        chStepper[i].stepper = stepperTemp;
      }
  #endif

  #ifdef VOLTAGE
      if (!Voltage_init()) {
        criticalError(1);
      }
  #endif

  #ifdef IMU
      if(!IMU_init()) {
        criticalError(2);
      }
  #endif

  #ifdef BMP280
      if(!BMP_init()) {
        criticalError(3);
      }
  #endif

  #ifdef HMC5883
      if(!HMC_init()) {
        criticalError(4);
      }
  #endif

  #ifdef SD_Card
      if (!SD_init()) {
        criticalError(5);
      }
      if (!SD_write_logHeader()) {
        criticalError(6);
      }   
  #endif

  #ifdef DEBUG
      for (int i = 0; i < 10; i++) {
        Serial.print(".");
        delay(500);
      }
  #endif
}


// ================================================================
// ===                        MAIN LOOP                         ===
// ================================================================
void loop() {
  int long curTime = millis();

  #ifdef NRF24
      // receive Data
      if (!NRF_receive()) {
        // if there is no input received -> reset data to default values
        NRF_failsave();
      }

      // send Data
      NRF_send();
  #endif

  #ifdef VOLTAGE
      Voltage_read();
  #endif

  #ifdef IMU
      IMU_read();
  #endif

  #ifdef BMP280
      BMP_read();
  #endif

  #ifdef HMC5883
      HMC_read();
  #endif

  
  // alloocate received data to outputs
  #ifdef SERVO
      
  #endif

  #ifdef STEPPER
      
  #endif

  
  updateOutputChannels();
  
  LED_state = !LED_state;  // blink LED
  digitalWrite(LED_PIN, LED_state);

  outputDataOverSerial();  // output all Values to Serial monitor (if defined)
  SD_write_log();

  loopTime = millis() - curTime;
}