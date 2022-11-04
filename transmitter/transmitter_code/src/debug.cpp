#include <Arduino.h>
#include "header.h"


#ifdef DEBUG
// ================================================================
// ===                          debug                           ===
// ================================================================

void Debug_Serial_out() {
    Serial.print("loopTime: ");
    Serial.print(loopTime);
             
    if (loopTime < 10) {
        Serial.print("   ");
    }
    else if (loopTime < 100) {
        Serial.print("  ");
    }
                  
    Serial.print(" || ");
    
    #ifdef NRF24
        for (int i = 0; i < sizeof(NRF_data_send.RF_data); i++) {
            Serial.print("ch: ");
            Serial.print(i);
            Serial.print(" - ");
            Serial.print(NRF_data_send.RF_data[i]);
            if (NRF_data_send.RF_data[i] < 10)
              Serial.print("  ");
            else if (NRF_data_send.RF_data[i] < 100)
              Serial.print(" ");
            Serial.print(" || ");
        }
    #endif
    #ifdef XBee
        for (int i = 0; i < sizeof(XBee_data_Transmit_Request.RF_data); i++) {
            Serial.print("ch: ");
            Serial.print(i);
            Serial.print(" - ");
            Serial.print(XBee_data_Transmit_Request.RF_data[i]);
            if (XBee_data_Transmit_Request.RF_data[i] < 10)
              Serial.print("  ");
            else if (XBee_data_Transmit_Request.RF_data[i] < 100)
              Serial.print(" ");
            Serial.print(" || ");
        }
    #endif
             
    Serial.println();
}

void Debug_WaitForSerial() {
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

    Serial.println();
    Serial.println("================================================================================");
    Serial.println();
}

void Debug_delay() {
    for (int i = 0; i < 10; i++) {
        Serial.print(".");
        delay(500);
    }
}
#endif