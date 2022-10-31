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

#ifdef XBee
// ================================================================
// ===                           XBee                           ===
// ================================================================


//byte packet[] = {0x7E, 0x00, 0x1A, 0x10, 0x01, 0x00, 0x13, 0xA2, 0x00, 0x41, 0xBB, 0xA0, 0x59, 0xFF, 0xFE, 0x00, 0x00, 0x48, 0x65, 0x6C, 0x6C, 0x6F, 0x20, 0x57, 0x6F, 0x72, 0x6C, 0x64, 0x21, 0x0A};
byte packet[] = {0x7E , 0x00 , 0x1D , 0x10 , 0x01 , 0x00 , 0x13 , 0xA2 , 0x00 , 0x41 , 0xBB , 0xA0 , 0x59 , 0xFF , 0xFE , 0x00 , 0x00 , 0x49 , 0x73 , 0x20 , 0x74 , 0x68 , 0x69 , 0x73 , 0x20 , 0x77 , 0x6F , 0x72 , 0x6B , 0x69 , 0x6E , 0x67 , 0x92};

bool XBee_init() {
  Serial1.begin(9600);

  return true;
}

bool XBee_send() {
  Serial1.write(packet, sizeof(packet));
}

bool XBee_receive() {
  if (!Serial1.available()) {
    return false;
  }

  Serial.println();
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }
  return true;
}

#endif