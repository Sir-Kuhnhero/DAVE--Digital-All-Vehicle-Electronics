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
uint8_t Frame_Type_Receive_Packet = 0x90;
uint8_t Frame_Type_Transmit_Status = 0x8B;
uint8_t Frame_Type_Transmit_Request = 0x10;

uint8_t Delivery_status_Success = 0x00;
uint8_t Delivery_status_Route_not_found = 0x25;


XBee_Data_Packet_send XBee_data_send;
XBee_Data_Packet_receive XBee_data_receive;


bool XBee_init() {
  Serial1.begin(9600);

  return true;
}

bool XBee_send() {
  /*----- Start_Delimiter ----- Length ----- Frame_Type_Transmit_Request ----- Frame_ID ----- Address_64Bit ----- Address_16Bit ----- Broadcast_radius ----- options ----- RF_data ----- checksum_Transmit_Request -----*/
  /*-------------- Calculate required values --------------*/
  //  3 -> number of bytes til RF_data (Start_Deliter, Length, ...); int (sizeof(RF_data)) -> number of bytes in RF_data; 1 -> byte for checksum_Transmit_Request
  int numbOfBytes = 17 + int (sizeof(XBee_data_send.RF_data)) + 1;


  uint8_t packet_full[numbOfBytes];
  
  // calculate Length bytes
  uint16_t length = numbOfBytes - 4;

  // seperate 16bit length value into two 8bit values
  XBee_data_send.Length[0] = length >> 8;
  XBee_data_send.Length[1] = length;

  /*---------- Start building byte array to send ----------*/
  // Start
  packet_full[0] = Start_Delimiter;
  // Length
  packet_full[1] = XBee_data_send.Length[0];
  packet_full[2] = XBee_data_send.Length[1];
  // Frame_Type
  packet_full[3] = Frame_Type_Transmit_Request;
  // Frame_ID
  packet_full[4] = XBee_data_send.Frame_ID;
  // 64Bit address
  packet_full[5] = XBee_data_send.Address_64Bit[0];
  packet_full[6] = XBee_data_send.Address_64Bit[1];
  packet_full[7] = XBee_data_send.Address_64Bit[2];
  packet_full[8] = XBee_data_send.Address_64Bit[3];
  packet_full[9] = XBee_data_send.Address_64Bit[4];
  packet_full[10] = XBee_data_send.Address_64Bit[5];
  packet_full[11] = XBee_data_send.Address_64Bit[6];
  packet_full[12] = XBee_data_send.Address_64Bit[7];
  //16Bit address
  packet_full[13] = XBee_data_send.Address_16Bit[0];
  packet_full[14] = XBee_data_send.Address_16Bit[1];
  // Broadcast_radius
  packet_full[15] = XBee_data_send.Broadcast_radius;
  // options
  packet_full[16] = XBee_data_send.options;

  // RF_data
  for (int i = 0; i < int (sizeof(XBee_data_send.RF_data) / sizeof(XBee_data_send.RF_data[0])); i++) {
    packet_full[i + 17] = XBee_data_send.RF_data[i];
  }

  // calculate checksum
  int decimalSum = 0;

  for (int i = 3; i < numbOfBytes - 1; i++) {
    decimalSum += packet_full[i];
  }

  XBee_data_send.checksum_Transmit_Request = 0xFF - uint8_t(decimalSum);

  // checksum
  packet_full[numbOfBytes - 1] = XBee_data_send.checksum_Transmit_Request;


  /*------------------- Send byte array -------------------*/
  Serial1.write(packet_full, sizeof(packet_full));


  /*-------- check if last byte array was received --------*/
  if (XBee_data_send.Delivery_status == Delivery_status_Success) {
    // reset delivery status to falure
    XBee_data_send.Delivery_status = Delivery_status_Route_not_found;
    return true;
  }
  else {
    return false;
  }
}

void XBee_receive_debug(uint8_t someByte, uint8_t firstByte) {
  // debug
  Serial.println();
  Serial.print("| someByte | firstByte | Length | Frame_Type | Address_64Bit | Address_16Bit | options | RF_data | checksum_receive");
  Serial.println();

  Serial.print(someByte, HEX);
  Serial.print("----");

  Serial.print(firstByte, HEX);
  Serial.print("----");

  Serial.print(XBee_data_receive.Length[0], HEX);
  Serial.print("--");
  Serial.print(XBee_data_receive.Length[1], HEX);
  Serial.print("----");

  Serial.print(Frame_Type_Receive_Packet, HEX);
  Serial.print("----");

  for (int i = 0; i < sizeof(XBee_data_receive.Address_64Bit); i++) {
    Serial.print(XBee_data_receive.Address_64Bit[i], HEX);
    Serial.print("--");
  }
  Serial.print("--");

  Serial.print(XBee_data_receive.Address_16Bit[0], HEX);
  Serial.print("--");
  Serial.print(XBee_data_receive.Address_16Bit[1], HEX);
  Serial.print("----");

  Serial.print(XBee_data_receive.options, HEX);
  Serial.print("----");

  for (int i = 0; i < sizeof(XBee_data_receive.RF_data); i++) {
    Serial.print(XBee_data_receive.RF_data[i], HEX);
    Serial.print("--");
  }
  Serial.print("--");

  Serial.print(XBee_data_receive.checksum_receive, HEX);
  Serial.println();


  if (Serial1.available()) {
    Serial.println("There is still serial: ");
  }


  while (Serial1.available()) {
    Serial.print(Serial1.read(), HEX);
    Serial.print("--");
  }
}

bool XBee_receive() {

  bool Receive_Packet_received = false;

  // try reading as long as serial data is available
  while (Serial1.available()) {
    // read first byte
    uint8_t firstByte = Serial1.read();

    if (firstByte != 0x7E) {
      // This is neither a receive packet or transmit status -> clear serial data
      Serial1.clear();
      return Receive_Packet_received;
    }


    // read length of packet
    uint8_t length[2];
    length[0] = Serial1.read();
    length[1] = Serial1.read();

    int LengthAsInt = length[0] * 256 + length[1];

    // frame type
    uint8_t b = Serial1.read();
    uint8_t frame_type = Serial.read();

    uint8_t someByte = frame_type;
    frame_type = b;


    // receive packet and transmit request have a different structure
    /*Receive_Packet -> ----- Start_Delimiter ----- Length ----- Frame_Type_Receive_Packet ----- Address_64Bit ----- Address_16Bit ----- options ----- RF_data ----- checksum -----*/
    if (frame_type == Frame_Type_Receive_Packet) {
      /*-------------------- Receive_Packet -------------------*/
      // save length
      XBee_data_receive.Length[0] = length[0];
      XBee_data_receive.Length[1] = length[1];

      // 64Bit Address
      for (int i = 0; i < 8; i++) {
        XBee_data_receive.Address_64Bit[i] = Serial1.read();
      }

      // 16Bit Address
      XBee_data_receive.Address_16Bit[0] = Serial1.read();
      XBee_data_receive.Address_16Bit[1] = Serial1.read();

      XBee_data_receive.options = Serial1.read();

      // read RF_data
      for (int i = 0; i < LengthAsInt - 12; i++) {
        XBee_data_receive.RF_data[i] = Serial1.read();
      }

      // read checksum
      XBee_data_receive.checksum_receive = Serial1.read();

      XBee_receive_debug(someByte, firstByte);

      Receive_Packet_received = true;
    }
    /*Transmit_Status -> ----- Start_Delimiter ----- Length ----- Frame_ID ----- Frame_Type_Transmit_Status ----- Address_16Bit ----- retry_count ----- Delivery_status ----- Discovery_status ----- checksum -----*/
    else if (frame_type == Frame_Type_Transmit_Status) {
      /*------------------- Transmit_Status -------------------*/
      // 16Bit Address
      uint8_t address_16Bit[2];

      //Frame_ID
      uint8_t frame_id = Serial1.read();

      address_16Bit[0] = Serial1.read();
      address_16Bit[1] = Serial1.read();


      // retry_count
      XBee_data_send.retry_count = Serial1.read();

      // Delivery_status
      XBee_data_send.Delivery_status = Serial1.read();

      // Discovery_status
      XBee_data_send.Discovery_status = Serial1.read();

      // read checksum
      XBee_data_send.checksum_Transmit_Status = Serial1.read();
    }
    else {
      /*------ neither Receive_Packet or Transmit_Status ------*/
      // Serial data is neither a receive packet nor a transmit status
      Serial.print("Clear Serial");
      Serial1.clear();
      return false;
    }
  }
  
  return Receive_Packet_received;
}

#endif