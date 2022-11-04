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

NRF_Data_Packet_receive NRF_data_receive;
NRF_Data_Packet_send NRF_data_send;

long NRF_receive_time;  // time the NRF24 took to receive data
bool NRF_receive_pass;  // true if NRF24 is able to receive data
const int NRF_MAX_receive_time = 250;  // max time the NRF24 will try receiving data

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
    NRF_receive_pass = false;

    radio.startListening();

    long time = millis();

    // try receiving till something is received or a time of NRF_MAX_receive_time is reached
    while(!NRF_receive_pass) {
    NRF_receive_time = millis() - time;

    if (radio.available()) {
      radio.read(&NRF_data_receive, sizeof(NRF_data_receive));

        NRF_receive_pass = true;
      }
      else if (NRF_receive_time >= NRF_MAX_receive_time) {
        return false;
      }
    }

  return NRF_receive_pass;
}

// return true if successful
bool NRF_send() {
    radio.stopListening();

    radio.write(&NRF_data_send, sizeof(NRF_data_send));
    return true;
}

// if nothing is received, the channels should be set to a neutral value
void NRF_failsave() {
    for (int i = 0; i < int (sizeof(NRF_data_receive) / sizeof(NRF_data_receive.RF_data[0])); i++) {
      NRF_data_receive.RF_data[i] = 128;
    }
}
#endif

#ifdef XBee
// ================================================================
// ===                           XBee                           ===
// ================================================================

uint8_t Start_Delimiter = 0x7E;

uint8_t Delivery_status_Success = 0x00;
uint8_t Delivery_status_Route_not_found = 0x25;


XBee_Data_Receive_Packet XBee_data_Receive_Packet;
XBee_Data_Transmit_Status XBee_data_Transmit_Status;
XBee_Data_Transmit_Request XBee_data_Transmit_Request;

enum packet_Type {NONE, RECEIVE_PACKET, TRANSMIT_STATUS};
int byte_ID_Read = 0;
packet_Type cur_packet_type;
uint8_t unknown_packet_length[2];



bool XBee_init() {
  Serial1.begin(9600);

  if (true) {
    Serial.println("XBee initialization successful");

    Serial.println();
    Serial.println("================================================================================");
    Serial.println();
  }
  
  return true;
}

bool XBee_send() {
  //----- Start_Delimiter ----- Length ----- Frame_Type_Transmit_Request ----- Frame_ID ----- Address_64Bit ----- Address_16Bit ----- Broadcast_radius ----- options ----- RF_data ----- checksum_Transmit_Request -----//
  //-------------- Calculate required values --------------//
  //  3 -> number of bytes til RF_data (Start_Deliter, Length, ...); int (sizeof(RF_data)) -> number of bytes in RF_data; 1 -> byte for checksum_Transmit_Request
  int numbOfBytes = 17 + int (sizeof(XBee_data_Transmit_Request.RF_data)) + 1;


  uint8_t packet[numbOfBytes];
  
  // calculate Length bytes
  uint16_t length = numbOfBytes - 4;

  // seperate 16bit length value into two 8bit values
  XBee_data_Transmit_Request.Length[0] = length >> 8;
  XBee_data_Transmit_Request.Length[1] = length;

  //---------- Start building byte array to send ----------//
  // Start
  packet[0] = Start_Delimiter;
  // Length
  packet[1] = XBee_data_Transmit_Request.Length[0];
  packet[2] = XBee_data_Transmit_Request.Length[1];
  // Frame_Type
  packet[3] = XBee_data_Transmit_Request.Frame_Type;
  // Frame_ID
  packet[4] = XBee_data_Transmit_Request.Frame_ID;
  // 64Bit address
  packet[5] = XBee_data_Transmit_Request.Address_64Bit[0];
  packet[6] = XBee_data_Transmit_Request.Address_64Bit[1];
  packet[7] = XBee_data_Transmit_Request.Address_64Bit[2];
  packet[8] = XBee_data_Transmit_Request.Address_64Bit[3];
  packet[9] = XBee_data_Transmit_Request.Address_64Bit[4];
  packet[10] = XBee_data_Transmit_Request.Address_64Bit[5];
  packet[11] = XBee_data_Transmit_Request.Address_64Bit[6];
  packet[12] = XBee_data_Transmit_Request.Address_64Bit[7];
  //16Bit address
  packet[13] = XBee_data_Transmit_Request.Address_16Bit[0];
  packet[14] = XBee_data_Transmit_Request.Address_16Bit[1];
  // Broadcast_radius
  packet[15] = XBee_data_Transmit_Request.Broadcast_radius;
  // options
  packet[16] = XBee_data_Transmit_Request.options;

  // RF_data
  for (int i = 0; i < int (sizeof(XBee_data_Transmit_Request.RF_data) / sizeof(XBee_data_Transmit_Request.RF_data[0])); i++) {
    packet[i + 17] = XBee_data_Transmit_Request.RF_data[i];
  }

  // calculate checksum
  int decimalSum = 0;

  for (int i = 3; i < numbOfBytes - 1; i++) {
    decimalSum += packet[i];
  }

  XBee_data_Transmit_Request.checksum = 0xFF - uint8_t(decimalSum);

  // checksum
  packet[numbOfBytes - 1] = XBee_data_Transmit_Request.checksum;


  //------------------- Send byte array -------------------//
  Serial1.write(packet, sizeof(packet));
  XBee_data_Transmit_Request.time_stamp = millis();

  return true;
}


bool XBee_receive() {
  if (!Serial1.available()) {
    return false;
  }


  while (Serial1.available()) {
    uint8_t b = Serial1.read();

    // Serial.print(b, HEX);
    // Serial.print("--");

    int caseNumb = byte_ID_Read;
    if (byte_ID_Read > 3)
      caseNumb = 4;
    
    switch (caseNumb)
    {
    case 0:  // Start_Delimiter
      if (b != 0x7E) {
        // This is neither a receive packet or transmit status -> clear serial data
        Serial1.clear();

        return false;
      }
      // Serial.println("Start_Delimiter");
      byte_ID_Read++;

      break;
    case 1:  // Length[0]
      unknown_packet_length[0] = b;
      // Serial.println("Length[0]");
      byte_ID_Read++;

      break;
    case 2:  // Length[1]
      unknown_packet_length[1] = b;
      // Serial.println("Length[1]");
      byte_ID_Read++;

      break;
    case 3:  // Frame_Type
      if (b == XBee_data_Receive_Packet.Frame_Type) {
        // Serial.println("Frame_Type (Receive_Packet)");
        cur_packet_type = RECEIVE_PACKET;

        XBee_data_Receive_Packet.Length[0] = unknown_packet_length[0];
        XBee_data_Receive_Packet.Length[1] = unknown_packet_length[1];

        // calc some stuff
        int lengthAsInt = unknown_packet_length[0] * 256 + unknown_packet_length[1];

        // expected packet is longer than defined packet length (wrong bytes read or too many / few bytes send)
        if (lengthAsInt != 12 + sizeof(XBee_data_Receive_Packet.RF_data)) {
          // Serial.println("error");
          cur_packet_type = NONE;
          byte_ID_Read = 0;

          return false;
        }
      }
      else if (b == XBee_data_Transmit_Status.Frame_Type) {
        // Serial.println("Frame_Type (Transmit_Status)");
        cur_packet_type = TRANSMIT_STATUS;

        
        XBee_data_Transmit_Status.Length[0] = unknown_packet_length[0];
        XBee_data_Transmit_Status.Length[1] = unknown_packet_length[1];

        // calc some stuff
        int lengthAsInt = unknown_packet_length[0] * 256 + unknown_packet_length[1];

        // expected packet is longer than defined packet length (wrong bytes read or too many / few bytes send)
        if (lengthAsInt != 7) {
          // Serial.println("error");
          cur_packet_type = NONE;
          byte_ID_Read = 0;

          return false;
        }
      }
      // Frame_Type unknown
      else {
        // Serial.println("Frame_Type unknown");
        cur_packet_type = NONE;
        byte_ID_Read = 0;

        Serial1.clear();
        return false;
      }
      byte_ID_Read++;

      break;
    case 4:
      switch (cur_packet_type)
      {
      case RECEIVE_PACKET:
          // Address_64Bit
          if (byte_ID_Read >= 4 && byte_ID_Read <= 11) {
            // Serial.println("Address_64Bit");
            XBee_data_Receive_Packet.Address_64Bit[byte_ID_Read - 4] = b;
          }
          // Address_16Bit
          else if (byte_ID_Read >= 12 && byte_ID_Read <= 13) {
            // Serial.println("Address_16Bit");
            XBee_data_Receive_Packet.Address_16Bit[byte_ID_Read - 12];
          }
          // options
          else if (byte_ID_Read == 14) {
            // Serial.println("options");
            XBee_data_Receive_Packet.options = b;
          }
          // RF_data
          else if (byte_ID_Read - 15 < sizeof(XBee_data_Receive_Packet.RF_data)) {
            // Serial.println("RF_data");
            XBee_data_Receive_Packet.RF_data[byte_ID_Read - 15] = b;
          }
          // checksum
          else {
            // Serial.println("checksum");
            XBee_data_Receive_Packet.checksum = b;

            XBee_data_Receive_Packet.time_stamp = millis();

            byte_ID_Read = 0;
            cur_packet_type = NONE;
            return true;
          }

          byte_ID_Read++;
        break;
      case TRANSMIT_STATUS:
          // Frame_ID
          if (byte_ID_Read == 4) {
            // Serial.println("Frame_ID");
            XBee_data_Transmit_Status.Frame_ID = b;
          }
          // Address_16Bit
          else if (byte_ID_Read >= 5 && byte_ID_Read <= 6) {
            // Serial.println("Address_16Bit");
            XBee_data_Transmit_Status.Address_16Bit[byte_ID_Read - 5];
          }
          // retry_count
          else if (byte_ID_Read == 7) {
            // Serial.println("retry_count");
            XBee_data_Transmit_Status.retry_count = b;
          }
          // Delivery_status
          else if (byte_ID_Read == 8) {
            // Serial.println("Delivery_status");
            XBee_data_Transmit_Status.Delivery_status = b;
          }
          // Discovery_status
          else if (byte_ID_Read == 9) {
            // Serial.println("Discovery_status");
            XBee_data_Transmit_Status.Discovery_status = b;
          }
          // checksum
          else {
            // Serial.println("checksum");
            XBee_data_Transmit_Status.checksum = b;

            XBee_data_Transmit_Status.time_stamp = millis();

            byte_ID_Read = 0;
            cur_packet_type = NONE;
            return true;
          }

          byte_ID_Read++;
        break;
      default:
        byte_ID_Read = 0;

        Serial1.clear();
        return false;
        break;
      }
      break;
    }
  }

  return true;
}


// if nothing is received, the channels should be set to a neutral value
void XBee_failsave() {
    for (int i = 0; i < sizeof(XBee_data_Receive_Packet.RF_data); i++) {
      ;
    }
}
#endif