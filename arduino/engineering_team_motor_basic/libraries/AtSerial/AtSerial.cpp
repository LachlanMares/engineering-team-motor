#include "AtSerial.h"
#include "Arduino.h"

AtSerial::AtSerial() {

}

void AtSerial::setInitial(long baudrate, int timeout) {
    Serial.begin(baudrate);
    if(timeout > 0) {
        Serial.setTimeout(timeout);
    }
}

int AtSerial::update(unsigned char* ext_buffer) {
    bool read_flag = false;
    int data_length = 0;
    unsigned int bytes_read = 0;

    for(int i=0; i<10; i++) {

        if(Serial.available() > HEADER_LENGTH) {
            unsigned char in_byte = Serial.read();
            
            if(in_byte == STX) {
                in_byte = Serial.read();
                
                if(in_byte > 0 & in_byte < ARDUINO_BUFFER_LENGTH) {
                    data_length = in_byte-HEADER_LENGTH;
                    read_flag = true;
                    break;
                }
            }
        }
    }

    if(read_flag) {
        unsigned char read_buffer[data_length];
        bytes_read = Serial.readBytes(read_buffer, data_length);

        if(bytes_read == data_length & read_buffer[data_length-1] == ETX) {
            memcpy(ext_buffer, &read_buffer, data_length-1);
        } else {
            read_flag = false;
        }
    }
  
    return data_length-1;
}

void AtSerial::sendMessage(unsigned char* buffer_start, int msg_length) {
    unsigned char packet[msg_length+3];
    packet[0] = STX;
    packet[1] = (unsigned char) msg_length+3;
    packet[msg_length+2] = ETX;
    memcpy(&packet[2], buffer_start, msg_length);
    Serial.write(packet, msg_length+3);
}


