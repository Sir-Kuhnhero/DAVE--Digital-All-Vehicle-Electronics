#pragma region License
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
#pragma endregion


#include <Arduino.h>
#include <SPI.h>
#include <I2Cdev.h>
#include "Wire.h"

#include "header.h"


char LED_PIN = 23;
bool LED_state = false;

int loopTime;


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


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  Serial.begin(9600);

  analogReadRes(12);

  pinMode(LED_PIN, OUTPUT);
  
  #ifdef DEBUG
      Debug_WaitForSerial();
  #endif


  #ifdef NRF24
      if (!NRF_init()) {
        criticalError(0);
      }
  #endif

  #ifdef SERVO
      Servo_init();
  #endif

  #ifdef STEPPER
      Stepper_init();
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

  XBee_init();

  #ifdef DEBUG
      Debug_delay();
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



  
  // alloocate received data to outputs and update them
  #ifdef SERVO
      Servo_update();
  #endif

  #ifdef STEPPER
      Stepper_update();
  #endif

  


  #ifdef DEBUG
      // Debug_Serial_out();
  #endif

  #ifdef log_ENABLE
      SD_write_log();
  #endif




  LED_state = !LED_state;  // blink LED
  digitalWrite(LED_PIN, LED_state);


  loopTime = millis() - curTime;


  //Debug_WaitForSerial();

  Serial.println("Try sending...");
  XBee_send();

  //Debug_delay();

  Serial.print("Try receiving...");

  if (XBee_receive())
    Serial.print("True");

  Serial.println();

  delay(1000);
}