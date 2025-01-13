#include "AtSerial.h"
#include "Arduino.h"

/*
Author:
    Lachlan Mares, lachlan.mares@adelaide.edu.au

License:
    ??    

Description:
    Class for implementing AT style headers and acknowledgement to serial messages
*/

AtSerial::AtSerial() {

}

void AtSerial::setInitial(long baudrate, int timeout) {
    /*
    param: baudrate: Serial port baudrate
    param: timeout: Serial port timeout period in milliseconds    
    */
    Serial.begin(baudrate);

    if(timeout > 0) {
        Serial.setTimeout(timeout);
    }
}

int AtSerial::update(unsigned char* ext_buffer) {
    /*
    param: ext_buffer: Pointer to a uint8_t buffer where the valid messages will go  
    return: message payload length
    */
    bool read_flag = false;
    int data_length = 0;
    unsigned int bytes_read = 0;

    // Try a couple of times to find start of message token
    for(int i=0; i<2; i++) {

        if(Serial.available() > HEADER_LENGTH) {
            unsigned char in_byte = Serial.read();
            // Check for start of message token
            if(in_byte == STX) {
                in_byte = Serial.read();
                
                // Second byte should be message length
                if(in_byte > 0 & in_byte < ARDUINO_BUFFER_LENGTH) {
                    data_length = in_byte-HEADER_LENGTH;
                    read_flag = true;
                    break;
                }
            }
        }
    }

    // If a message was found
    if(read_flag) {
        unsigned char read_buffer[data_length];
        bytes_read = Serial.readBytes(read_buffer, data_length);

        // Check for the end of message token
        if(bytes_read == data_length & read_buffer[data_length-1] == ETX) {
            // Copy into external buffer
            memcpy(ext_buffer, &read_buffer, data_length-1);

        } else {
            read_flag = false;
        }
    }
  
    return data_length-1;
}

void AtSerial::sendMessage(unsigned char* buffer_start, int msg_length) {
    /*
    param: buffer_start: pointer to where the message payload is stored
    param: msg_length: payload length
    */
    unsigned char packet[msg_length+3];
    // Add length + header and footer tokens
    packet[0] = STX;
    packet[1] = (unsigned char) msg_length + 3;
    packet[msg_length+2] = ETX;

    memcpy(&packet[2], buffer_start, msg_length);

    // Sned message
    Serial.write(packet, msg_length + 3);
}


