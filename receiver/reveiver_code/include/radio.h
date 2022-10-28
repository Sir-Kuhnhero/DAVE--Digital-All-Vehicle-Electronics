#include <Arduino.h>

// initialize data shared between .cpp files

struct Data_Package_receive {
  byte channel[6];
};

struct Data_Package_send {
  byte x = 100;
};

extern Data_Package_receive data_receive;
extern Data_Package_send data_send;

extern long receiveTime;  // time the NRF24 took to receive data

bool NRF_init();
bool NRF_receive();
bool NRF_send();
void NRF_failsave();
