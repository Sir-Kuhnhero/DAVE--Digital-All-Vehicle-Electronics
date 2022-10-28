#include <Arduino.h>
#include "sensor.h"

#ifdef VOLTAGE
// ================================================================
// ===                         Voltage                          ===
// ================================================================
Voltage chVoltage[4];

void Voltage_init() {
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
}

void Voltage_read()
{
    for (int i = 0; i < int (sizeof(chVoltage) / sizeof(chVoltage[0])); i++) {
        int value = analogRead(chVoltage[i].pin);

        // convert the measured value to a voltage. 4098 is because of the 12bit read resolution
        chVoltage[i].value = value * 3.3 / 4098 / (chVoltage[i].R2 / (chVoltage[i].R1 + chVoltage[i].R2));
    }
}
#endif