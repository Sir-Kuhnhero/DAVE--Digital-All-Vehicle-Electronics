#include <Arduino.h>

// initialize data shared between .cpp files
#define SD_Card
#define log_ENABLE

#ifdef SD_Card
    #include "SdFat.h"

    extern SdFat SD;
    extern FsFile logFile;

    extern String curFileName;

    bool SD_init();
    bool SD_write_log();
    bool SD_write_logHeader();
#endif