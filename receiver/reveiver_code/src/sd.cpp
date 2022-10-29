#include <Arduino.h>
#include "sd.h"
#include "main.h"


#ifdef SD_Card
// ================================================================
// ===                            SD                            ===
// ================================================================
#include "SdFat.h"



// initialize data shared between .cpp files
// SDCARD_SS_PIN is defined for the built-in SD on some boards.
const uint8_t SD_CS_PIN = SS;
//const uint8_t SD_CS_PIN = SDCARD_SS_PIN;

// Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
#define SPI_CLOCK SD_SCK_MHZ(50)

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif  ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif  // HAS_SDIO_CLASS


SdFat SD;
FsFile logFile;

String curFileName;

long sdEntry = 0; // each time log data is written sdEntry increases by 1 -> acts as identifier

String fileName = "log_";

bool SD_init() {
    Serial.print("Initializing SD card...");
  
    // see if the card is present and can be initialized:
    if (!SD.begin(SD_CONFIG)) {
        Serial.println("Card failed, or not present");
        // don't do anything more:
        return false;
    }
    Serial.println("card initialized.");


    #ifdef log_ENABLE
        // starts with tempFileName as fileName. If it already exists increment the last number 
        String tempFileName = fileName + "00.csv";
        int i = 0;

        while (SD.exists(tempFileName)) {
            i++;
            if (i < 10) {
                tempFileName = fileName + "0" + i + ".csv";
            }
            else {
                tempFileName = fileName +  i + ".csv";
            }
        }

        curFileName = tempFileName;
        Serial.println(curFileName);

        logFile = SD.open(curFileName, FILE_WRITE);
        if (SD.exists(tempFileName)) {
            Serial.println("new file created");
        }
    #endif

    return true;
}

bool SD_write_log() {
    #ifdef log_ENABLE
        String stringToWrite = sdEntry;
        sdEntry++;

        #pragma region logLoopTime
            stringToWrite = stringToWrite + ";" + loopTime;
        #pragma endregion

        #ifdef NRF24_LOG
            // currently only receiveTime and conPass is being loged
            stringToWrite = stringToWrite + ";" + receiveTime;
            stringToWrite = stringToWrite + ";" + NRF_receive;
        #endif

        #ifdef READ_VOLTAGE_LOG
            for (int i = 0; i < int (sizeof(chVoltage) / sizeof(chVoltage[0])); i++) {
                stringToWrite = stringToWrite + ";" + chVoltage[i].value;
            }
        #endif

        #ifdef IMU_LOG
            #ifdef OUTPUT_READABLE_QUATERNION
                // log quaternion values in easy matrix form: w x y z
                stringToWrite = stringToWrite + ";" + q.w;
                stringToWrite = stringToWrite + ";" + q.x;
                stringToWrite = stringToWrite + ";" + q.y;
                stringToWrite = stringToWrite + ";" + q.z;
            #endif

            #ifdef OUTPUT_READABLE_EULER
                // log Euler angles in degrees
                stringToWrite = stringToWrite + ";" + euler[0] * 180/M_PI;
                stringToWrite = stringToWrite + ";" + euler[1] * 180/M_PI;
                stringToWrite = stringToWrite + ";" + euler[2] * 180/M_PI;
            #endif

            #ifdef OUTPUT_READABLE_YAWPITCHROLL
                // log Euler angles in degrees
                stringToWrite = stringToWrite + ";" + ypr[0] * 180/M_PI;
                stringToWrite = stringToWrite + ";" + ypr[1] * 180/M_PI;
                stringToWrite = stringToWrite + ";" + ypr[2] * 180/M_PI;
            #endif

            #ifdef OUTPUT_READABLE_REALACCEL
                // log real acceleration, adjusted to remove gravity
                stringToWrite = stringToWrite + ";" + aaReal.x;
                stringToWrite = stringToWrite + ";" + aaReal.y;
                stringToWrite = stringToWrite + ";" + aaReal.z;
            #endif

            #ifdef OUTPUT_READABLE_WORLDACCEL
                // log initial world-frame acceleration, adjusted to remove gravity
                // and rotated based on known orientation from quaternion
                stringToWrite = stringToWrite + ";" + aaWorld.x;
                stringToWrite = stringToWrite + ";" + aaWorld.y;
                stringToWrite = stringToWrite + ";" + aaWorld.z;
            #endif
        #endif

        #ifdef BMP280_LOG
            stringToWrite = stringToWrite + ";" + temp_event.temperature;
            stringToWrite = stringToWrite + ";" + pressure_event.pressure;
        #endif

        #ifdef HMC5883_LOG
            stringToWrite = stringToWrite + ";" + mag.XAxis;
            stringToWrite = stringToWrite + ";" + mag.YAxis;
            stringToWrite = stringToWrite + ";" + mag.ZAxis;
            stringToWrite = stringToWrite + ";" + mag.HeadingDegress;
        #endif

        logFile = SD.open(curFileName, FILE_WRITE);
        Serial.println(stringToWrite);
        logFile.println(stringToWrite);
        logFile.close();
    #endif

    return true;
}

bool SD_write_logHeader() {
    #ifdef log_ENABLE
        String stringToWrite = "entry";

        stringToWrite = stringToWrite + ";" + "loopTime";

        #ifdef NRF24_LOG
            stringToWrite = stringToWrite + ";" + "receiveTime";
            stringToWrite = stringToWrite + ";" + "NRF_receive";
        #endif

        #ifdef READ_VOLTAGE_LOG
            for (int i = 0; i < int (sizeof(chVoltage) / sizeof(chVoltage[0])); i++) {
                stringToWrite = stringToWrite + ";" + "v: " + i;
            }
        #endif

        #ifdef IMU_LOG
            #ifdef OUTPUT_READABLE_QUATERNION
                // display quaternion values in easy matrix form: w x y z
                stringToWrite = stringToWrite + ";" + "quat: w";
                stringToWrite = stringToWrite + ";" + "quat: x";
                stringToWrite = stringToWrite + ";" + "quat: y";
                stringToWrite = stringToWrite + ";" + "quat: z";
            #endif

            #ifdef OUTPUT_READABLE_EULER
                // display Euler angles in degrees
                stringToWrite = stringToWrite + ";" + "euler: 0";
                stringToWrite = stringToWrite + ";" + "euler: 1";
                stringToWrite = stringToWrite + ";" + "euler: 2";
            #endif

            #ifdef OUTPUT_READABLE_YAWPITCHROLL
                // display Euler angles in degrees
                stringToWrite = stringToWrite + ";" + "ypr: 0";
                stringToWrite = stringToWrite + ";" + "ypr: 1";
                stringToWrite = stringToWrite + ";" + "ypr: 2";
            #endif

            #ifdef OUTPUT_READABLE_REALACCEL
                // display real acceleration, adjusted to remove gravity
                stringToWrite = stringToWrite + ";" + "areal: x";
                stringToWrite = stringToWrite + ";" + "areal: y";
                stringToWrite = stringToWrite + ";" + "areal: z";
            #endif

            #ifdef OUTPUT_READABLE_WORLDACCEL
                // display initial world-frame acceleration, adjusted to remove gravity
                // and rotated based on known orientation from quaternion
                stringToWrite = stringToWrite + ";" + "aworld: x";
                stringToWrite = stringToWrite + ";" + "aworld: y";
                stringToWrite = stringToWrite + ";" + "aworld: z";
            #endif
        #endif

        #ifdef BMP280_LOG
            stringToWrite = stringToWrite + ";" + "Temperature (BMP, C*)";
            stringToWrite = stringToWrite + ";" + "Pressure (hPa)";
        #endif

        #ifdef HMC5883_LOG
            stringToWrite = stringToWrite + ";" + "mag.XAxis";
            stringToWrite = stringToWrite + ";" + "mag.YAxis";
            stringToWrite = stringToWrite + ";" + "mag.ZAxis";
            stringToWrite = stringToWrite + ";" + "mag.HeadingDegress";
        #endif

        logFile = SD.open(curFileName, FILE_WRITE);
        Serial.println(stringToWrite);
        logFile.println(stringToWrite);
        logFile.close();
    #endif

    return true;
}

#endif