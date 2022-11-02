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
              
    #ifdef NRF_SERIAL_out
        Serial.print("reviveTime: ");
        Serial.print(NRF_receive_time);
        Serial.print(" || ");
                 
        for (int i = 0; i < int (sizeof(NRF_data_receive.RF_data) / sizeof(NRF_data_receive.RF_data[0])); i++) {
            Serial.print("ch: ");
            Serial.print(i);
            Serial.print(" - ");
            Serial.print(NRF_data_receive.RF_data[i]);
                 
            if (NRF_data_receive.RF_data[i] < 10) {
                Serial.print("  ");
            }
            else if (NRF_data_receive.RF_data[i] < 100) {
                Serial.print(" ");
            }
            
            Serial.print(" || ");
        }
    #endif
                     
    #ifdef VOLTAGE_SERIAL_out
        for (int i = 0; i < int (sizeof(chVoltage) / sizeof(chVoltage[0])); i++) {
            Serial.print("v: ");
            Serial.print(i);
            Serial.print(" - ");
            Serial.print(chVoltage[i].value);
            Serial.print(" || ");
        }
    #endif
                     
    #ifdef IMU_SERIAL_out
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
             
    #ifdef BMP_SERIAL_out
        Serial.print(F("Temperature = "));
        Serial.print(temp_event.temperature);
        Serial.print(" *C");
        Serial.print(" || ");
             
        Serial.print(F("Pressure = "));
        Serial.print(pressure_event.pressure);
        Serial.print(" hPa");
        Serial.print(" || ");
    #endif
             
    #ifdef HMC_SERIAL_out
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