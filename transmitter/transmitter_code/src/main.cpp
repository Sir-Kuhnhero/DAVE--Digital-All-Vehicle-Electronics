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
  
#pragma region NRF24
    /*
     * See documentation at https://nRF24.github.io/RF24
     * See License information at root directory of this library
     * Author: Brendan Doherty (2bndy5)
     */
#pragma endregion

#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include "header.h"


int loopTime;

struct ch {
  int value;
  char pin;
  bool invert;
  int valueMin = 0, valueMax = 4098, zeroPosition = 2049;
  int deadzone = 0;
} ch[6];

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  Serial.begin(9600);

  analogReadRes(12);

  #ifdef DEBUG
      Debug_WaitForSerial();
  #endif

  #ifdef NRF24
      NRF_init();
  #endif

  #ifdef XBee
      XBee_init();
  #endif

  #pragma region configureChannels
      // connected pin
      ch[0].pin = A0;
      ch[1].pin = A1;
      ch[2].pin = A2;
      ch[3].pin = A3;
      ch[4].pin = A6;
      ch[5].pin = A7;
      // is direction inverted?
      ch[0].invert = true;
      ch[1].invert = false;
      ch[2].invert = false;
      ch[3].invert = true;
      ch[4].invert = true;
      ch[5].invert = true;
      // minimum value the input can reach (important for scaling from 0 to 4098)
      ch[0].valueMin = 1550;
      ch[1].valueMin = 1610;
      ch[2].valueMin = 1560;
      ch[3].valueMin = 1510;
      ch[4].valueMin = 80;
      ch[5].valueMin = 55;
      // maximum value the input can reach (important for scaling from 0 to 4098)
      ch[0].valueMax = 2500;
      ch[1].valueMax = 2555;
      ch[2].valueMax = 2565;
      ch[3].valueMax = 2494;
      ch[4].valueMax = 4010;
      ch[5].valueMax = 3990;
      // value of zero position
      ch[0].zeroPosition = 1915;
      ch[1].zeroPosition = 2090;
      ch[2].zeroPosition = 2136;
      ch[3].zeroPosition = 2000;
      ch[4].zeroPosition = 2065;
      ch[5].zeroPosition = 2055;
      // range around zero position that will not change the output
      ch[0].deadzone = 25;
      ch[1].deadzone = 25;
      ch[2].deadzone = 25;
      ch[3].deadzone = 25;
      ch[4].deadzone = 50;
      ch[5].deadzone = 50;
  #pragma endregion

  #ifdef DEBUG
      Debug_delay();
  #endif
}

// ================================================================
// ===                        MAIN LOOP                         ===
// ================================================================
void loop() {
  int long curTime = millis();

  // read analog inputs
  for (int i = 0; i < int (sizeof(ch) / sizeof(ch[0])); i++) {
    ch[i].value = analogRead(ch[i].pin);

    if (ch[i].invert)
      ch[i].value = 4098 - ch[i].value;

    ch[i].value = constrain(ch[i].value, ch[i].valueMin, ch[i].valueMax);
    ch[i].value = map(ch[i].value, ch[i].valueMin, ch[i].valueMax, 0, 4098);

    if (ch[i].value <= ch[i].zeroPosition + ch[i].deadzone && ch[i].value >= ch[i].zeroPosition - ch[i].deadzone)
      ch[i].value = 4098 / 2;
  }

  // prepare data for transmission
  for (int i = 0; i < int (sizeof(ch) / sizeof(ch[0])); i++)
  {
    #ifdef NRF24
        NRF_data_send.RF_data[i] = uint8_t(map(ch[i].value, 0, 4098, 0, 255));
    #endif
    #ifdef XBee
        XBee_data_Transmit_Request.RF_data[i] = uint8_t(map(ch[i].value, 0, 4098, 0, 255));
    #endif
  }

  #ifdef SERIAL_out
      Debug_Serial_out();
  #endif


  #ifdef NRF24
      NRF_send();
  #endif

  #ifdef XBee
      XBee_send();
  #endif

  loopTime = millis() - curTime;
}