#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <AccelStepper.h>

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";

struct chServo {
  Servo servo;
  int value = 128;
  int minValue = 45;
  float valueScaler = 1;
  char pin;
  bool valueAsAngle = true;
} chServo[12];

struct chStepper {
  AccelStepper stepper;
  int value = 128;
  int minValue = 0;
  float valueScaler = 4;
  float maxSpeed, acceleration;
  int stepPin, dirPin, enablePin;
  bool valueAsAngle = false;
  bool active = true;
} chStepper[4];

struct Data_Package {
  byte channel[6];
} data;

bool reciveData() {
  bool recived = false;
  long time = millis();

  while(!recived)
  {
    if (radio.available()) {
      radio.read(&data, sizeof(data));

      recived = true;
    }

    if (millis() - time > 1000)
      return false;
  }

  Serial.print("reciveTime");
  Serial.print(millis() - time);
  Serial.print(";  ");
  return true;
}

void updateOutputChannels() {
  // Servo / ESC
  for (int i = 0; i < int (sizeof(chServo) / sizeof(chServo[0])); i++) {
    int value = constrain(int (chServo[i].value * chServo[i].valueScaler) + chServo[i].minValue, 0, 255);

    if (!chServo[i].valueAsAngle) {
      value = map(value, 0, 255, 0, 180);  
    }
    else {
      value = constrain(value, 0, 180);
    }
    
    chServo[i].servo.write(value);
  }

  // Stepper
  for (int i = 0; i < int (sizeof(chStepper) / sizeof(chStepper[0])); i++) {
    if (!chStepper[i].active) {
      digitalWrite(chStepper[i].enablePin, HIGH);
      break;
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
}


void setup() {
  Serial.begin(9600);

  // start radion communication
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  // configure channels
  chServo[0].pin = 2;
  chServo[1].pin = 3;
  chServo[2].pin = 4;
  chServo[3].pin = 5;
  chServo[4].pin = 6;
  chServo[5].pin = 9;
  chServo[6].pin = 10;
  chServo[7].pin = 14;
  chServo[8].pin = 15;
  chServo[9].pin = 33;
  chServo[10].pin = 36;
  chServo[11].pin = 37;

  for (int i = 0; i < int (sizeof(chServo) / sizeof(chServo[0])); i++) {
    chServo[i].servo.attach(chServo[i].pin, 1000, 2000);
  }

  chStepper[0].stepPin = 20;
  chStepper[0].dirPin = 21;
  chStepper[0].enablePin = 22;
  chStepper[1].stepPin = 30;
  chStepper[1].dirPin = 31;
  chStepper[1].enablePin = 32;
  chStepper[2].stepPin = 33;
  chStepper[2].dirPin = 35;
  chStepper[2].enablePin = 38;
  chStepper[3].stepPin = 39;
  chStepper[3].dirPin = 40;
  chStepper[3].enablePin = 41;
  
  chStepper[0].maxSpeed = 3000.00;
  chStepper[1].maxSpeed = 3000.00;
  chStepper[2].maxSpeed = 3000.00;
  chStepper[3].maxSpeed = 3000.00;
  chStepper[0].acceleration = 100.00;
  chStepper[1].acceleration = 100.00;
  chStepper[2].acceleration = 100.00;
  chStepper[3].acceleration = 100.00;

  for (int i = 0; i < int (sizeof(chStepper) / sizeof(chStepper[0])); i++) {
    pinMode(chStepper[i].enablePin, OUTPUT);
    AccelStepper stepperTemp(1, chStepper[i].stepPin, chStepper[i].dirPin);
    stepperTemp.setMaxSpeed(chStepper[i].maxSpeed);
    stepperTemp.setAcceleration(chStepper[i].acceleration);
    chStepper[i].stepper = stepperTemp;
  }
}

void loop() {
  if (!reciveData())
    Serial.println("no data connection!!!");
  
  for (int i = 0; i < int (sizeof(data.channel) / sizeof(data.channel[0])); i++)
  {
    Serial.print("ch");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(data.channel[i]);
    Serial.print(";  ");
  }
  Serial.println();

  chServo[0].value = data.channel[0];
  chStepper[1].value = data.channel[5];

  updateOutputChannels();
}