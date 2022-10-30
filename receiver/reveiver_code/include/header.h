#include <Arduino.h>

// define data shared between .cpp files

// ================================================================
// ===                           main                           ===
// ================================================================

extern int loopTime;


// ================================================================
// ===                          radio                           ===
// ================================================================
#define NRF24

#ifdef NRF24
    struct Data_Package_receive {
      byte channel[6];
    };
    
    struct Data_Package_send {
      byte x = 100;
    };
    
    extern Data_Package_receive data_receive;
    extern Data_Package_send data_send;
    
    extern long receiveTime;  // time the NRF24 took to receive data
    extern bool receivePass;  // true if NRF24 is able to receive data
    
    bool NRF_init();
    bool NRF_receive();
    bool NRF_send();
    void NRF_failsave();
#endif


// ================================================================
// ===                          sensor                          ===
// ================================================================
// #define VOLTAGE
// #define IMU
// #define BMP280
// #define HMC5883

#ifdef VOLTAGE
    struct Voltage {
        float value;
        char pin;
        float R1 = 1.00, R2 = 1.00;
    };

    extern Voltage chVoltage[4];

    bool Voltage_read();
    bool Voltage_init();
#endif

#ifdef IMU
    #include "helper_3dmath.h"
    // uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
    // quaternion components in a [w, x, y, z] format (not best for parsing
    // on a remote host such as Processing or something though)
    //#define OUTPUT_READABLE_QUATERNION

    // uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
    // (in degrees) calculated from the quaternions coming from the FIFO.
    // Note that Euler angles suffer from gimbal lock (for more info, see
    // http://en.wikipedia.org/wiki/Gimbal_lock)
    //#define OUTPUT_READABLE_EULER

    // uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
    // pitch/roll angles (in degrees) calculated from the quaternions coming
    // from the FIFO. Note this also requires gravity vector calculations.
    // Also note that yaw/pitch/roll angles suffer from gimbal lock (for
    // more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
    #define OUTPUT_READABLE_YAWPITCHROLL

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


    extern Quaternion q;           // [w, x, y, z]         quaternion container
    extern VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    extern VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    extern VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    extern VectorFloat gravity;    // [x, y, z]            gravity vector
    extern float euler[3];         // [psi, theta, phi]    Euler angle container
    extern float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


    bool IMU_init();
    bool IMU_read();
#endif

#ifdef BMP280
    #include <Adafruit_BMP280.h>

    extern sensors_event_t temp_event, pressure_event;

    bool BMP_init();
    bool BMP_read();
#endif

#ifdef HMC5883
    #include <DFRobot_QMC5883.h>

    extern sVector_t mag;

    bool HMC_init();
    bool HMC_read();
#endif


// ================================================================
// ===                            SD                            ===
// ================================================================
// #define SD_Card
// #define log_ENABLE

#define NRF_log
#define VOLTAGE_log
#define IMU_log
#define BMP_log
#define HMC_log

#pragma region preventError
    // if loging is not enabled no data can be logged
    #ifndef log_ENABLE
        #undef NRF_log
        #undef VOLTAGE_log
        #undef IMU_log
        #undef BMP_log
        #undef HMC_log
    #endif
    // if a module is not implemented it also can not be logged
    #ifndef NRF24
        #undef NRF_log
    #endif
    #ifndef VOLTAGE
        #undef VOLTAGE_log
    #endif
    #ifndef IMU
        #undef IMU_log
    #endif
    #ifndef BMP280
        #undef BMP_log
    #endif
    #ifndef HMC5883
        #undef HMC_log
    #endif
#pragma endregion

#ifdef SD_Card
    #include "SdFat.h"

    extern SdFat SD;
    extern FsFile logFile;

    extern String curFileName;

    bool SD_init();
    bool SD_write_log();
    bool SD_write_logHeader();
#endif


// ================================================================
// ===                          output                          ===
// ================================================================
#define SERVO
#define STEPPER

#ifdef SERVO
    #include <Servo.h>

    struct ServoOut {
        Servo servo;
        int value = 128;
        int minValue = 0;
        float valueScaler = 1;
        char pin;
        bool valueAsAngle = false;
    };

    extern ServoOut chServo[11];

    bool Servo_init();
    bool Servo_update();
#endif

#ifdef STEPPER
    #include <AccelStepper.h>
    
    struct StepperOut {
        AccelStepper stepper;
        int value = 128;
        int minValue = 0;
        float valueScaler = 4;
        float maxSpeed, acceleration;
        int stepPin, dirPin, enablePin;
        bool valueAsAngle = false;
        bool active = true;
    };

    extern StepperOut chStepper[4];

    bool Stepper_init();
    bool Stepper_update();
#endif


// ================================================================
// ===                          debug                           ===
// ================================================================
// #define DEBUG

#define NRF_SERIAL_out
#define VOLTAGE_SERIAL_out
#define IMU_SERIAL_out
#define BMP_SERIAL_out
#define HMC_SERIAL_out

#pragma region preventError
    // if DEBUG is not enabled no data should be printed over the serial monitor
    #ifndef DEBUG
        #undef NRF_SERIAL_out
        #undef VOLTAGE_SERIAL_out
        #undef IMU_SERIAL_out
        #undef BMP_SERIAL_out
        #undef HMC_SERIAL_out
    #endif
    // if a module is not implemented it also can not be printed over the serial monitor
    #ifndef NRF24
        #undef NRF_SERIAL_out
    #endif
    #ifndef VOLTAGE
        #undef VOLTAGE_SERIAL_out
    #endif
    #ifndef IMU
        #undef IMU_SERIAL_out
    #endif
    #ifndef BMP280
        #undef BMP_SERIAL_out
    #endif
    #ifndef HMC5883
        #undef HMC_SERIAL_out
    #endif
#pragma endregion

#ifdef DEBUG
    void Debug_Serial_out();
    void Debug_WaitForSerial();
    void Debug_delay();
#endif