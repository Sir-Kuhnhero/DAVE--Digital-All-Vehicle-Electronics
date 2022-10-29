#include <Arduino.h>
#include "header.h"

#ifdef NRF24
// ================================================================
// ===                         NRF24L01                         ===
// ================================================================
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8);  // CE, CSN

const byte address[6] = "00001";

Data_Package_receive data_receive;
Data_Package_send data_send;

long receiveTime;  // time the NRF24 took to receive data
bool receivePass;  // true if NRF24 is able to receive data
const int maxReceiveTime = 250;  // max time the NRF24 will try receiving data

// return true if successful
bool NRF_init() {
    // start radio communication
    Serial.println("Initializing NRF module...");
    if (!radio.begin()) {
        Serial.print("NRF initialization failed");
        return false;
    }
    
    radio.openReadingPipe(0, address);
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();

    Serial.println("NRF initialization successful");

    Serial.println();
    Serial.println("================================================================================");
    Serial.println();

    return true;
}

// return true if successful
bool NRF_receive() {
    receivePass = false;

    radio.startListening();

    long time = millis();

    // try receiving till something is received or a time of maxReceiveTime is reached
    while(!receivePass) {
    receiveTime = millis() - time;

    if (radio.available()) {
      radio.read(&data_receive, sizeof(data_receive));

        receivePass = true;
      }
      else if (receiveTime >= maxReceiveTime) {
        return false;
      }
    }

  return receivePass;
}

// return true if successful
bool NRF_send() {
    radio.stopListening();

    radio.write(&data_send, sizeof(data_send));
    return true;
}

// if nothing is received, the channels should be set to a neutral value
void NRF_failsave() {
    for (int i = 0; i < int (sizeof(data_receive) / sizeof(data_receive.channel[0])); i++) {
      data_receive.channel[i] = 128;
    }
}
#endif