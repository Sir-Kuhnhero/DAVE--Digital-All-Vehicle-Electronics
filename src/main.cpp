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

#define NRF24
#define SERVO
#define STEPPER
#define READ_VOLTAGE
#define MPU6050_READ

#ifdef NRF24
RF24 radio(7, 8);  // CE, CSN

const byte address[6] = "00001";

struct Data_Package_recive {
  byte channel[6];
} data_recive;

struct Data_Package_send {
  byte x = 100;
} data_send;
#endif

#ifdef SERVO
struct chServo {
  Servo servo;
  int value = 128;
  int minValue = 0;
  float valueScaler = 1;
  char pin;
  bool valueAsAngle = false;
} chServo[11];
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
} chStepper[4];
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

bool reciveData() {
  bool recived = false;

  #ifdef NRF24
    radio.startListening();

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
  #endif

  return recived;
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
  #endif
}

void readVoltage() {
  #ifdef READ_VOLTAGE
    for (int i = 0; i < int (sizeof(chVoltage) / sizeof(chVoltage[0])); i++) {
      int value = analogRead(chVoltage[i].pin);

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
        Serial.print(F("FIFO overflow!"));

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
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
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
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
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
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
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
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
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
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld: ");
            Serial.print(aaWorld.x);
            Serial.print(", ");
            Serial.print(aaWorld.y);
            Serial.print(", ");
            Serial.println(aaWorld.z);
            Serial.print(" || ");
        #endif
    }
  #endif
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  Serial.begin(9600);

  analogReadRes(12);
  
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
    }
  #endif
  

  #ifdef NRF24
    // start radio communication
    radio.begin();
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
}


// ================================================================
// ===                        MAIN LOOP                         ===
// ================================================================
void loop() {
  
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // recive Data
  if (reciveData()) {
    for (int i = 0; i < int (sizeof(data_recive.channel) / sizeof(data_recive.channel[0])); i++)
    {
      Serial.print("ch: ");
      Serial.print(i);
      Serial.print(" - ");
      Serial.print(data_recive.channel[i]);

      if (data_recive.channel[i] < 10)
        Serial.print("  ");
      else if (data_recive.channel[i] < 100)
        Serial.print(" ");
      Serial.print(" || ");
    }
  }
  else {
    Serial.print("no data connection!!!    ");
  }

  // send Data
  sendData();


  readVoltage();

  #ifdef MPU6050_READ
    if (!(!mpuInterrupt && fifoCount < packetSize))
      readMPU6050();
  #endif

  // alloocate recived data to outputs
  chServo[0].value = data_recive.channel[4];
  chStepper[0].value = data_recive.channel[2];

  
  updateOutputChannels();
  Serial.println();
}