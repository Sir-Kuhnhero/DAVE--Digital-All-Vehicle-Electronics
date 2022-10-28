#include <Arduino.h>

// initialize data shared between .cpp files
#define VOLTAGE
#define IMU

#ifdef VOLTAGE
    struct Voltage {
        float value;
        char pin;
        float R1 = 1.00, R2 = 1.00;
    };

    extern Voltage chVoltage[4];

    void Voltage_read();
    void Voltage_init();
#endif

