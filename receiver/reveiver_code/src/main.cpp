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
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <AccelStepper.h>
#include <I2Cdev.h>
#include "Wire.h"
#include <MPU6050_6Axis_MotionApps20.h>
#include "MPU6050.h"
#include <Adafruit_BMP280.h>
#include <DFRobot_QMC5883.h>
#include "SdFat.h"


#define NRF24
//#define SERVO
//#define STEPPER
//#define READ_VOLTAGE
//#define MPU6050_READ
//#define BMP280
//#define HMC5883
#define SD_Card
#ifdef SD_Card
    #define dataLogging
    #ifdef dataLogging
        #define NRF24_LOG
        //#define SERVO_LOG
        //#define STEPPER_LOG
        //#define READ_VOLTAGE_LOG
        //#define MPU6050_READ_LOG
        //#define BMP280_LOG
        //#define HMC5883_LOG
    #endif
#endif

#define DEBUG
#ifdef DEBUG
    //#define SERIAL_OUTPUT  // if defined all sensor data will be printed
#endif

char LED_PIN = 23;
bool LED_state = false;

int loopTime;



#ifdef NRF24
    RF24 radio(7, 8);  // CE, CSN

    const byte address[6] = "00001";

    struct Data_Package_receive {
      byte channel[6];
    } data_receive;

    struct Data_Package_send {
      byte x = 100;
    } data_send;

    int receiveTime;  // time the NRF24 took to receive data
    const int maxReceiveTime = 250;  // max time the NRF24 will try reciving data
    boolean NRF_receive = false;
#endif

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

#ifdef READ_VOLTAGE
    struct chVoltage {
      float value;
      char pin;
      float R1 = 1.00, R2 = 1.00;
    } chVoltage[4];
#endif

#ifdef MPU6050_READ
    MPU6050 mpu;

    // uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
    // quaternion components in a [w, x, y, z] format (not best for parsing
    // on a remote host such as Processing or something though)
    //#define OUTPUT_READABLE_QUATERNION

    // uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
    // (in degrees) calculated from the quaternions coming from the FIFO.
    // Note that Euler angles suffer from gimbal lock (for more info, see
    // http://en.wikipedia.org/wiki/Gimbal_lock)
    #define OUTPUT_READABLE_EULER

    // uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
    // pitch/roll angles (in degrees) calculated from the quaternions coming
    // from the FIFO. Note this also requires gravity vector calculations.
    // Also note that yaw/pitch/roll angles suffer from gimbal lock (for
    // more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
    //#define OUTPUT_READABLE_YAWPITCHROLL

    // uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
    // components with gravity removed. This acceleration reference frame is
    // not compensated for orientation, so +X is always +X according to the
    // sensor, just without the effects of gravity. If you want acceleration
    // compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
    //#define OUTPUT_READABLE_REALACCEL

    // uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
    // components with gravity removed and adjusted for the world frame of
    // reference (yaw is relative to initial orientation, since no magnetometer
    // is present in this case). Could be quite handy in some cases.
    //#define OUTPUT_READABLE_WORLDACCEL

    // MPU control/status vars
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


    // ================================================================
    // ===               INTERRUPT DETECTION ROUTINE                ===
    // ================================================================

    volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
    void dmpDataReady() {
        mpuInterrupt = true;
    }
#endif

#ifdef BMP280
    Adafruit_BMP280 bmp; // use I2C interface
    Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
    Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

    sensors_event_t temp_event, pressure_event;
#endif

#ifdef HMC5883
    DFRobot_QMC5883 compass(&Wire, /*I2C addr*/0x0D);
    /*
      Depending on where you buy your "HMC5883" module you can either get a genuin HMC5883L chip of a cheaper QMC5883 chip.
      They each need a different I2C address. These are 0x0D (QMC5883) and 0x1E (HMC5883L). 
      Note that i didn't test the address for a genuin HMC5883L because I don't have one.
    */

    sVector_t mag;
#endif

#ifdef SD_Card
    // SDCARD_SS_PIN is defined for the built-in SD on some boards.
    const uint8_t SD_CS_PIN = SS;
    //const uint8_t SD_CS_PIN = SDCARD_SS_PIN;

    // Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
    #define SPI_CLOCK SD_SCK_MHZ(50)

    // Try to select the best SD card configuration.
    #if HAS_SDIO_CLASS
    #define SD_CONFIG SdioConfig(FIFO_SDIO)
    #elif  ENABLE_DEDICATED_SPI
    #define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
    #else  // HAS_SDIO_CLASS
    #define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
    #endif  // HAS_SDIO_CLASS


    SdFat SD;
    FsFile logFile;
    //FsFile logFile;

    //FsFile logFile;
    String fileName = "log_";
    String curFileName;

#endif

bool receiveData() {
  bool received = false;

  #ifdef NRF24
      radio.startListening();

      long time = millis();

      // try reciving till something is received or a time of maxReceiveTime is reached
      while(!received) {
        receiveTime = millis() - time;

        if (radio.available()) {
          radio.read(&data_receive, sizeof(data_receive));

          NRF_receive = true;
          received = true;
        }
        else if (receiveTime > maxReceiveTime) {
          NRF_receive = false;
          return false;
        }
      }
  #endif

  return received;
}

void sendData() {
  #ifdef NRF24
      radio.stopListening();

      radio.write(&data_send, sizeof(data_send));
  #endif
}

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

void readVoltage() {
  #ifdef READ_VOLTAGE
      for (int i = 0; i < int (sizeof(chVoltage) / sizeof(chVoltage[0])); i++) {
        int value = analogRead(chVoltage[i].pin);

        // convert the measured value to a voltage. 4098 is because of the 12bit read resolution
        chVoltage[i].value = value * 3.3 / 4098 / (chVoltage[i].R2 / (chVoltage[i].R1 + chVoltage[i].R2));
      }
  #endif
}

void readMPU6050() {
  #ifdef MPU6050_READ
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();

      // get current FIFO count
      fifoCount = mpu.getFIFOCount();

      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
          // reset so we can continue cleanly
          mpu.resetFIFO();
          Serial.print(F("FIFO overflow!   "));

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } 
      else if ((mpuIntStatus & 0x02) > 0) {
          // wait for correct available data length, should be a VERY short wait
          while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

          // read a packet from FIFO
          mpu.getFIFOBytes(fifoBuffer, packetSize);

          // track FIFO count here in case there is > 1 packet available
          // (this lets us immediately read more without waiting for an interrupt)
          fifoCount -= packetSize;

          #ifdef OUTPUT_READABLE_QUATERNION
              // read quaternion values in easy matrix form: w x y z
              mpu.dmpGetQuaternion(&q, fifoBuffer);
          #endif

          #ifdef OUTPUT_READABLE_EULER
              // read Euler angles in degrees
              mpu.dmpGetQuaternion(&q, fifoBuffer);
              mpu.dmpGetEuler(euler, &q);
          #endif

          #ifdef OUTPUT_READABLE_YAWPITCHROLL
              // read Euler angles in degrees
              mpu.dmpGetQuaternion(&q, fifoBuffer);
              mpu.dmpGetGravity(&gravity, &q);
              mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          #endif

          #ifdef OUTPUT_READABLE_REALACCEL
              // read real acceleration, adjusted to remove gravity
              mpu.dmpGetQuaternion(&q, fifoBuffer);
              mpu.dmpGetAccel(&aa, fifoBuffer);
              mpu.dmpGetGravity(&gravity, &q);
              mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
          #endif

          #ifdef OUTPUT_READABLE_WORLDACCEL
              // read initial world-frame acceleration, adjusted to remove gravity
              // and rotated based on known orientation from quaternion
              mpu.dmpGetQuaternion(&q, fifoBuffer);
              mpu.dmpGetAccel(&aa, fifoBuffer);
              mpu.dmpGetGravity(&gravity, &q);
              mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
              mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
          #endif
      }
  #endif
}

void readBMP280() {
  #ifdef BMP280
    bmp_temp->getEvent(&temp_event);
    bmp_pressure->getEvent(&pressure_event);
  #endif
}

void readHMC5883() {
  #ifdef HMC5883
      float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
      compass.setDeclinationAngle(declinationAngle);
      mag = compass.readRaw();
      compass.getHeadingDegrees();
  #endif
}

void writeHeader() {
  #ifdef SD_Card
      logFile.print("loopTime");
      logFile.print(";");
      logFile.print("receiveTime");
      logFile.print(";");
      logFile.print("ConPass");
      logFile.println();
      logFile.close();
  #endif
}

bool logData() {
  #ifdef SD_Card
      

      //logFile = SD.open(curFileName, FILE_WRITE);
      //logFile.print(loopTime);
      //logFile.print(";");
      //logFile.print(receiveTime);
      //logFile.print(";");
      //logFile.print("false");
  #endif

  #ifdef dataLogging
      if (!SD.open(curFileName, FILE_WRITE)) return false;

      String stringToWrite = "";

      #pragma region logLoopTime
          stringToWrite = stringToWrite + ";" + loopTime;
      #pragma endregion

      #ifdef NRF24_LOG
          // currently only receiveTime and conPass is being loged
          stringToWrite = stringToWrite + ";" + receiveTime;
          stringToWrite = stringToWrite + ";" + NRF_receive;
      #endif

      #ifdef READ_VOLTAGE_LOG
          for (int i = 0; i < int (sizeof(chVoltage) / sizeof(chVoltage[0])); i++) {
            Serial.print("v: ");
            Serial.print(i);
            Serial.print(" - ");
            Serial.print(chVoltage[i].value);
            Serial.print(" || ");
          }
      #endif

      #ifdef MPU6050_READ_LOG
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

      #ifdef BMP280_LOG
          Serial.print(F("Temperature = "));
          Serial.print(temp_event.temperature);
          Serial.print(" *C");
          Serial.print(" || ");

          Serial.print(F("Pressure = "));
          Serial.print(pressure_event.pressure);
          Serial.print(" hPa");
          Serial.print(" || ");
      #endif

      #ifdef HMC5883_LOG
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

      logFile = SD.open(curFileName, FILE_WRITE);
      Serial.println(stringToWrite);
      logFile.println(stringToWrite);
      logFile.close();
      return true;
  #endif

  return false;
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
  delay(250);
  digitalWrite(LED_PIN, LOW);
  delay(250);

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

      #ifdef READ_VOLTAGE
          for (int i = 0; i < int (sizeof(chVoltage) / sizeof(chVoltage[0])); i++) {
            Serial.print("v: ");
            Serial.print(i);
            Serial.print(" - ");
            Serial.print(chVoltage[i].value);
            Serial.print(" || ");
          }
      #endif

      #ifdef MPU6050_READ
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
      // start radio communication
      if (!radio.begin())
        criticalError(0);
      radio.openReadingPipe(0, address);
      radio.openWritingPipe(address);
      radio.setPALevel(RF24_PA_MIN);
      radio.startListening();
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

  #ifdef READ_VOLTAGE
      chVoltage[0].pin = 24;
      chVoltage[1].pin = 25;
      chVoltage[2].pin = 26;
      chVoltage[3].pin = 27;

      for (int i = 0; i < int (sizeof(chVoltage) / sizeof(chVoltage[0])); i++) {
        pinMode(chVoltage[i].pin, INPUT);
      }

      chVoltage[0].R2 = 1.00;
      chVoltage[1].R2 = 2.00;
      chVoltage[2].R2 = 2.00;
      chVoltage[3].R2 = 7.5;
  #endif

  #ifdef MPU6050_READ
      // join I2C bus (I2Cdev library doesn't do this automatically)
      #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		  #if defined (PARTICLE)
		  	Wire.setSpeed(CLOCK_SPEED_400KHZ);
		  	Wire.begin();
		  #else
		  	Wire.begin();
		  	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
		  #endif
      #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
          Fastwire::setup(400, true);
      #endif

      // initialize device
      Serial.println(F("Initializing I2C devices..."));
      mpu.initialize();

      // verify connection
      Serial.println(F("Testing device connections..."));
      Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

      // load and configure the DMP
      Serial.println(F("Initializing DMP..."));
      devStatus = mpu.dmpInitialize();

      // supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXGyroOffset(220);
      mpu.setYGyroOffset(76);
      mpu.setZGyroOffset(-85);
      mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

      // make sure it worked (returns 0 if so)
      if (devStatus == 0) {
          // turn on the DMP, now that it's ready
          Serial.println(F("Enabling DMP..."));
          mpu.setDMPEnabled(true);

          // enable Arduino interrupt detection
          Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
	    #if defined (PARTICLE)
          attachInterrupt(D2, dmpDataReady, RISING);
	    #else
          attachInterrupt(2, dmpDataReady, RISING);
	    #endif
          mpuIntStatus = mpu.getIntStatus();

          // set our DMP Ready flag so the main loop() function knows it's okay to use it
          Serial.println(F("DMP ready! Waiting for first interrupt..."));
          dmpReady = true;

          // get expected DMP packet size for later comparison
          packetSize = mpu.dmpGetFIFOPacketSize();
      } else {
          // ERROR!
          // 1 = initial memory load failed
          // 2 = DMP configuration updates failed
          // (if it's going to break, usually the code will be 1)
          Serial.print(F("DMP Initialization failed (code "));
          Serial.print(devStatus);
          Serial.println(F(")"));
          criticalError(1);
      }
  #endif

  #ifdef BMP280
      Serial.println(F("BMP280 Sensor event test"));

      unsigned status;
      status = bmp.begin(BMP280_ADDRESS_ALT, 0x60);
      //status = bmp.begin();
      if (!status) {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                          "try a different address!"));
        Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");

        criticalError(2);
      }

      /* Default settings from datasheet. */
      bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                      Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                      Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                      Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                      Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

      bmp_temp->printSensorDetails();
  #endif

  #ifdef HMC5883
      if (!compass.begin()) {
          Serial.println("Could not find a valid 5883 sensor, check wiring!");
          criticalError(3);
        }

      if(compass.isHMC())
      {
        Serial.println("Initialize HMC5883");

        //Set/get the compass signal gain range, default to be 1.3 Ga
        // compass.setRange(HMC5883L_RANGE_1_3GA);
        // Serial.print("compass range is:");
        // Serial.println(compass.getRange());

        //Set/get measurement mode
        // compass.setMeasurementMode(HMC5883L_CONTINOUS);
        // Serial.print("compass measurement mode is:");
        // Serial.println(compass.getMeasurementMode());

        //Set/get the data collection frequency of the sensor
        // compass.setDataRate(HMC5883L_DATARATE_15HZ);
        // Serial.print("compass data rate is:");
        // Serial.println(compass.getDataRate());

        //Get/get sensor status
        // compass.setSamples(HMC5883L_SAMPLES_8);
        // Serial.print("compass samples is:");
        // Serial.println(compass.getSamples());
      }
      else if(compass.isQMC())
      {
        Serial.println("Initialize QMC5883");
        // compass.setRange(QMC5883_RANGE_2GA);
        // Serial.print("compass range is:");
        // Serial.println(compass.getRange());

        // compass.setMeasurementMode(QMC5883_CONTINOUS);
        // Serial.print("compass measurement mode is:");
        // Serial.println(compass.getMeasurementMode());

        // compass.setDataRate(QMC5883_DATARATE_50HZ);
        // Serial.print("compass data rate is:");
        // Serial.println(compass.getDataRate());

        // compass.setSamples(QMC5883_SAMPLES_8);
        // Serial.print("compass samples is:");
        // Serial.println(compass.getSamples());
      }
      else if(compass.isVCM())
      {
        Serial.println("Initialize VCM5883L");
        // compass.setMeasurementMode(VCM5883L_CONTINOUS);
        // Serial.print("compass measurement mode is:");
        // Serial.println(compass.getMeasurementMode());

        // compass.setDataRate(VCM5883L_DATARATE_200HZ);
        // Serial.print("compass data rate is:");
        // Serial.println(compass.getDataRate());
      }
  #endif

  #ifdef SD_Card
      Serial.print("Initializing SD card...");
  
      // see if the card is present and can be initialized:
      if (!SD.begin(SD_CONFIG)) {
        Serial.println("Card failed, or not present");
        // don't do anything more:
        return;
      }
      Serial.println("card initialized.");


      // starts with tempFileName as fileName. If it already exists increment the last number 
      String tempFileName = fileName + "00.csv";
      int i = 0;

      while (SD.exists(tempFileName)) {
        i++;
        if (i < 10) {
          tempFileName = fileName + "0" + i + ".csv";
        }
        else {
          tempFileName = fileName +  i + ".csv";
        }
      }

      curFileName = tempFileName;
      Serial.println(curFileName);
      //if (!logFile.open(tempFileName, FILE_WRITE)) {
      //  Serial.println(F("file.open failed"));
      //}

      logFile = SD.open(curFileName, FILE_WRITE);
      //logFile.open(SD, curFileName, O_RDWR);
      //logFile.open(curFileName, O_RDWR);
      //logFile.open(tempFileName, FILE_WRITE);

      if (SD.exists(tempFileName)) {
        Serial.println("new file created");
      }

      //logFile = SD.open(curFileName);

      writeHeader();
  #endif
}


// ================================================================
// ===                        MAIN LOOP                         ===
// ================================================================
void loop() {
  int long curTime = millis();

  #ifdef NRF24
      // receive Data
      if (!receiveData()) {
        // if there is no input received -> reset data to default values
        for (int i = 0; i < int (sizeof(data_receive) / sizeof(data_receive.channel[0])); i++) {
          data_receive.channel[i] = 128;
        }
      }

      // send Data
      //sendData();
  #endif

  readVoltage();

  #ifdef MPU6050_READ
      if (!(!mpuInterrupt && fifoCount < packetSize))
        readMPU6050();
  #endif

  readBMP280();
  readHMC5883();

  
  // alloocate received data to outputs
  #ifdef SERVO
      
  #endif

  #ifdef STEPPER
      
  #endif

  
  updateOutputChannels();
  
  LED_state = !LED_state;  // blink LED
  digitalWrite(LED_PIN, LED_state);

  outputDataOverSerial();  // output all Values to Serial monitor (if defined)
  logData();

  delay(100);

  loopTime = millis() - curTime;
}