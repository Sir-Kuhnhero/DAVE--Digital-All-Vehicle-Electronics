#include <Arduino.h>
#include "sensor.h"

#ifdef VOLTAGE
// ================================================================
// ===                         Voltage                          ===
// ================================================================
Voltage chVoltage[4];

bool Voltage_init() {
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

    return true;
}

bool Voltage_read()
{
    for (int i = 0; i < int (sizeof(chVoltage) / sizeof(chVoltage[0])); i++) {
        int value = analogRead(chVoltage[i].pin);

        // convert the measured value to a voltage. 4098 is because of the 12bit read resolution
        chVoltage[i].value = value * 3.3 / 4098 / (chVoltage[i].R2 / (chVoltage[i].R1 + chVoltage[i].R2));
    }

    return true;
}
#endif

#ifdef IMU
// ================================================================
// ===                            IMU                           ===
// ================================================================
#include <MPU6050_6Axis_MotionApps20.h>
#include "MPU6050.h"



MPU6050 mpu;

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


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

bool IMU_init() {
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
        return false;
    }
    return true;
}


bool IMU_read() {
    if (!mpuInterrupt && fifoCount < packetSize) {
        return false;
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.print(F("FIFO overflow!   "));
        return false;
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
    return true;
}
#endif

#ifdef BMP280
// ================================================================
// ===                          BMP280                          ===
// ================================================================
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

sensors_event_t temp_event, pressure_event;

bool BMP_init() {
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

        return false;
    }

    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    bmp_temp->printSensorDetails();

    return true;
}

bool BMP_read() {
    bmp_temp->getEvent(&temp_event);
    bmp_pressure->getEvent(&pressure_event);

    return true;
}

#endif

#ifdef HMC5883
// ================================================================
// ===                          HMC5883                         ===
// ================================================================
#include <DFRobot_QMC5883.h>
DFRobot_QMC5883 compass(&Wire, /*I2C addr*/0x0D);
/*
    Depending on where you buy your "HMC5883" module you can either get a genuin HMC5883L chip of a cheaper QMC5883 chip.
    They each need a different I2C address. These are 0x0D (QMC5883) and 0x1E (HMC5883L). 
    Note that i didn't test the address for a genuin HMC5883L because I don't have one.
*/

sVector_t mag;

bool HMC_init() {
    if (!compass.begin()) {
        Serial.println("Could not find a valid 5883 sensor, check wiring!");
        return false;
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
    else if(compass.isQMC()) {
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
    else if(compass.isVCM()) {
        Serial.println("Initialize VCM5883L");
        // compass.setMeasurementMode(VCM5883L_CONTINOUS);
        // Serial.print("compass measurement mode is:");
        // Serial.println(compass.getMeasurementMode());

        // compass.setDataRate(VCM5883L_DATARATE_200HZ);
        // Serial.print("compass data rate is:");
        // Serial.println(compass.getDataRate());
    }

    return true;
}

bool HMC_read() {
    float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
    compass.setDeclinationAngle(declinationAngle);
    mag = compass.readRaw();
    compass.getHeadingDegrees();
}
#endif