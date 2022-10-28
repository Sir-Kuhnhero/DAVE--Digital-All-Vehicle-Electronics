#include <Arduino.h>
#include "radio.h"
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8);  // CE, CSN

const byte address[6] = "00001";

Data_Package_receive data_receive;
Data_Package_send data_send;

long receiveTime;  // time the NRF24 took to receive data
const int maxReceiveTime = 250;  // max time the NRF24 will try reciving data

// return true if succesfull
bool NRF_init() {
    // start radio communication
    Serial.println("Initializing NRF module...");
    if (!radio.begin()) {
        Serial.print("NRF Initialization failed");
        return false;
    }
    
    radio.openReadingPipe(0, address);
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();

    Serial.print("NRF Initialization complete");
    return true;
}

bool NRF_receive() {
    bool received = false;

    radio.startListening();

    long time = millis();

    // try reciving till something is received or a time of maxReceiveTime is reached
    while(!received) {
    receiveTime = millis() - time;

    if (radio.available()) {
      radio.read(&data_receive, sizeof(data_receive));

        received = true;
      }
      else if (receiveTime >= maxReceiveTime) {
        return false;
      }
    }

  return received;
}

bool NRF_send() {
    radio.stopListening();

    radio.write(&data_send, sizeof(data_send));
    return true;
}

void NRF_failsave() {
    for (int i = 0; i < int (sizeof(data_receive) / sizeof(data_receive.channel[0])); i++) {
      data_receive.channel[i] = 128;
    }
}