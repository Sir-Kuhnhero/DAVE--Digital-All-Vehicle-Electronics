#include <Arduino.h>
#include "header.h"


#ifdef SERVO
// ================================================================
// ===                          Servo                           ===
// ================================================================
#include <Servo.h>



ServoOut chServo[11];

bool Servo_init() {
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

    return true;
}

bool Servo_update() {
    // Servo / ESC
    for (int i = 0; i < int (sizeof(chServo) / sizeof(chServo[0])); i++) {
        int value = int (chServo[i].value * chServo[i].valueScaler) + chServo[i].minValue;

        if (!chServo[i].valueAsAngle) {
            value = map(value, 0, 255, 0, 180);
        }

        chServo[i].servo.write(value);
    }

    return true;
}

#endif

#ifdef STEPPER
// ================================================================
// ===                         Stepper                          ===
// ================================================================
#include <AccelStepper.h>



StepperOut chStepper[4];

bool Stepper_init() {
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

    return true;
}

bool Stepper_update() {
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

    return true;
}
#endif