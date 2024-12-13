#ifndef AtSerial_h
#define AtSerial_h

#include "Arduino.h"

#define STX 0x02
#define ETX 0x03
#define ACK 0x06
#define NAK 0x15
#define ARDUINO_BUFFER_LENGTH 64
#define HEADER_LENGTH 2
#define FOOTER_LENGTH 1

class AtSerial {
	public:
		AtSerial();
		void setInitial(long baudrate, int timeout);
    	int update(unsigned char* ext_buffer);
    	void sendMessage(unsigned char* buffer_start, int msg_length);

	private:

};

#endif
