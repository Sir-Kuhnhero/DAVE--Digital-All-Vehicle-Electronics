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

uint8_t Start_Delimiter[] = {0x7E};
uint8_t Length[2];
uint8_t Frame_Type[] = {0x10};
uint8_t Frame_ID[] = {0x01};
uint8_t Address_64Bit[] = {0x00 , 0x13 , 0xA2 , 0x00 , 0x41 , 0xBB , 0xA0 , 0x59};
uint8_t Address_16Bit[] = {0xFF, 0xFE};
uint8_t Broadcast_radius[] = {0x00};
uint8_t options[] = {0x00};

uint8_t packet[] = {0x49 , 0x73 , 0x20 , 0x74 , 0x68 , 0x69 , 0x73 , 0x20 , 0x77 , 0x6F , 0x72 , 0x6B , 0x69 , 0x6E , 0x67};

uint8_t checksum;  // gets calculated automatically

//uint8_t packet_premade[] = {0x7E , 0x00 , 0x1D , 0x10 , 0x01 , 0x00 , 0x13 , 0xA2 , 0x00 , 0x41 , 0xBB , 0xA0 , 0x59 , 0xFF , 0xFE , 0x00 , 0x00 , 0x49 , 0x73 , 0x20 , 0x74 , 0x68 , 0x69 , 0x73 , 0x20 , 0x77 , 0x6F , 0x72 , 0x6B , 0x69 , 0x6E , 0x67 , 0x92};
//byte packet[] =7E 00 1D 10 01 00 13 A2 00 41 BB A0 59 FF FE 00 00 49 73 20 74 68 69 73 20 77 6F 72 6B 69 6E 67 92};

bool XBee_init() {
  Serial1.begin(9600);

  return true;
}

bool XBee_send() {
  /*-------------- Calculate required values --------------*/
  //  3 -> number of bytes til packet (Start_Deliter, Length, ...); int (sizeof(packet)) -> number of bytes in packet; 1 -> byte for checksum
  int numbOfBytes = 17 + int (sizeof(packet)) + 1;
  byte packet_full[numbOfBytes];
  
  // calculate Length bytes
  uint16_t length = numbOfBytes - 4;
  uint16_t MASK  = 0xFF00; //1111 1111 0000 0000

  // seperate 16bit length value into two 8bit values
  Length[0] = (length & MASK) >> 8;
  Length[1] = length >> 8;

  // calculate checksum
  int decimalSum = 0;

  for (int i = 3; i < numbOfBytes - 1; i++) {
    decimalSum += packet_full[i];
  }

  checksum = 0xFF - uint8_t(decimalSum);
  
  /*---------- Start building byte array to send ----------*/
  // Start
  packet_full[0] = Start_Delimiter[0];
  // Length
  packet_full[1] = Length[0];
  packet_full[2] = Length[1];
  // Frame_Type
  packet_full[3] = Frame_Type[0];
  // Frame_ID
  packet_full[4] = Frame_ID[0];
  // 64Bit address
  packet_full[5] = Address_64Bit[0];
  packet_full[6] = Address_64Bit[1];
  packet_full[7] = Address_64Bit[2];
  packet_full[8] = Address_64Bit[3];
  packet_full[9] = Address_64Bit[4];
  packet_full[10] = Address_64Bit[5];
  packet_full[11] = Address_64Bit[6];
  packet_full[12] = Address_64Bit[7];
  //16Bit address
  packet_full[13] = Address_16Bit[0];
  packet_full[14] = Address_16Bit[1];
  // Broadcast_radius
  packet_full[15] = Broadcast_radius[0];
  // options
  packet_full[16] = options[0];

  // packet
  for (int i = 0; i < int (sizeof(packet) / sizeof(packet[0])); i++) {
    packet_full[i + 17] = packet[i];
  }

  // checksum
  packet_full[numbOfBytes - 1] = checksum;

  // print packet_full for debugging
  for (int i = 0; i < int (sizeof(packet_full) / sizeof(packet_full[0])); i++) {
    Serial.print(packet_full[i], HEX);
  }


  /*------------------- Send byte array -------------------*/
  Serial1.write(packet_full, sizeof(packet_full));

  return true;
}


bool XBee_receive() {
  if (!Serial1.available()) {
    return false;
  }


  return true;

  Serial.println();
  while (Serial1.available()) {
    Serial.print(Serial1.read(),HEX);
    Serial.print("--");
  }


  return true;
}

#endif