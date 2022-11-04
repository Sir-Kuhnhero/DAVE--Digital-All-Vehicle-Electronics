#include <Arduino.h>

// define data shared between .cpp files

// ================================================================
// ===                           main                           ===
// ================================================================

extern int loopTime;


// ================================================================
// ===                          radio                           ===
// ================================================================
#define NRF24
#define XBee

const int numbOfBytes_send = 6;
const int numbOfBytes_received = 0;

#ifdef NRF24
    struct NRF_Data_Packet_receive {
      uint8_t RF_data[numbOfBytes_received];
    };
    
    struct NRF_Data_Packet_send {
      uint8_t RF_data[numbOfBytes_send];
    };
    
    extern NRF_Data_Packet_receive NRF_data_receive;
    extern NRF_Data_Packet_send NRF_data_send;
    
    extern long NRF_receive_time;  // time the NRF24 took to receive data
    extern bool NRF_receive_pass;  // true if NRF24 is able to receive data
    
    bool NRF_init();
    bool NRF_receive();
    bool NRF_send();
    void NRF_failsave();
#endif

#ifdef XBee
    /*----- Start_Delimiter ----- Length ----- Frame_Type(Receive_Packet) ----- Address_64Bit ----- Address_16Bit ----- options ----- RF_data ----- checksum -----*/
    struct XBee_Data_Receive_Packet {
        uint8_t Length[2];
        const uint8_t Frame_Type = 0x90;
        uint8_t Address_64Bit[8];
        uint8_t Address_16Bit[2];
        uint8_t options;

        uint8_t RF_data[numbOfBytes_received];

        uint8_t checksum;

        long time_stamp;
    };

    /*----- Start_Delimiter ----- Length ----- Frame_Type(Transmit_Status) ----- Frame_ID ----- Address_16Bit ----- retry_count ----- Delivery_status ----- Discovery_status ----- checksum -----*/
    struct XBee_Data_Transmit_Status {
        uint8_t Length[2];
        const uint8_t Frame_Type = 0x8B;
        uint8_t Frame_ID;
        uint8_t Address_16Bit[2];
        uint8_t retry_count;
        uint8_t Delivery_status;
        uint8_t Discovery_status;

        uint8_t checksum;

        long time_stamp;
    };
    
    /*----- Start_Delimiter ----- Length ----- Frame_Type(Transmit_Request) ----- Frame_ID ----- Address_64Bit ----- Address_16Bit ----- Broadcast_radius ----- options ----- RF_data ----- checksum -----*/
    struct XBee_Data_Transmit_Request {
        uint8_t Length[2];
        const uint8_t Frame_Type = 0x10;
        uint8_t Frame_ID = 0x01;
        uint8_t Address_64Bit[8] = {0x00 , 0x13 , 0xA2 , 0x00 , 0x41 , 0xBB , 0xA0 , 0x59};
        uint8_t Address_16Bit[2] = {0xFF, 0xFE};
        uint8_t Broadcast_radius = 0x00;
        uint8_t options = 0x00;

        uint8_t RF_data[numbOfBytes_send];

        uint8_t checksum;  // gets calculated automatically

        long time_stamp;
    };

    extern XBee_Data_Receive_Packet XBee_data_Receive_Packet;
    extern XBee_Data_Transmit_Status XBee_data_Transmit_Status;
    extern XBee_Data_Transmit_Request XBee_data_Transmit_Request;

    bool XBee_init();
    bool XBee_send();
    bool XBee_receive();
    void XBee_failsave();
#endif


// ================================================================
// ===                          debug                           ===
// ================================================================
#define DEBUG
#define SERIAL_out


#ifdef DEBUG
    void Debug_Serial_out();
    void Debug_WaitForSerial();
    void Debug_delay();
#endif