#include <Arduino.h>

// initialize data shared between .cpp files
#define VOLTAGE
#define IMU
#define BMP280
#define HMC5883

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
