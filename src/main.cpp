#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <AccelStepper.h>

RF24 radio(7, 8);  // CE, CSN

const byte address[6] = "00001";

struct chServo {
  Servo servo;
  int value = 128;
  int minValue = 45;
  float valueScaler = 0.75;
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
} data_recive;

struct Data_Package_return {
  byte x = 100;
} data_send;

bool reciveData() {
  radio.startListening();

  bool recived = false;
  long time = millis();

  while(!recived)
  {
    if (radio.available())
    {
      radio.read(&data_recive, sizeof(data_recive));

      recived = true;
    }
    else if (millis() - time > 1000)
      return false;
  }

  Serial.print("reviveTime: ");
  Serial.print(millis() - time);
  Serial.print(" || ");

  return true;
}

void sendData() {
  radio.stopListening();
  
  radio.write(&data_send, sizeof(data_send));
}

void updateOutputChannels() {
  // Servo / ESC
  for (int i = 0; i < int (sizeof(chServo) / sizeof(chServo[0])); i++) {
    int value = int (chServo[i].value * chServo[i].valueScaler) + chServo[i].minValue;

    if (!chServo[i].valueAsAngle) {
      value = map(value, 0, 255, 0, 180);
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
  // start radio communication
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.openWritingPipe(address);
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
}

void loop() {
  // recive Data
  if (!reciveData())
    Serial.print("no data connection!!!    ");

  for (int i = 0; i < int (sizeof(data_recive.channel) / sizeof(data_recive.channel[0])); i++)
  {
    Serial.print("ch: ");
    Serial.print(i);
    Serial.print(" - ");
    Serial.print(data_recive.channel[i]);
    Serial.print(" || ");
  }

  // send Data
  sendData();


  // alloocate recived data to outputs
  chServo[0].value = data_recive.channel[0];
  chStepper[0].value = data_recive.channel[5];



  updateOutputChannels();

  Serial.println();
  //delay(5);
}