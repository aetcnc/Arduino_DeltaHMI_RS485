#ifndef SIMPLE_MODBUS_MASTER_AET_H
#define SIMPLE_MODBUS_MASTER_AET_H

// SimpleModbusMasterAET.h

// Edited by Graeme Hulme-Jones of Affordable Engineering Technologies on 20/12/2013 for use with
// the AET RS485 arduino shield. graeme@affordengtech.com or info@affordengtech.com

// This is an edited version of the original "SimpleModbusMasterV10" library from arduino.cc.
// Only minor changes have been made such as the removal of the pointer allocation of Serial ports.
// The serial port number is now explicitly defined in "SimpleModbusMasterAET.cpp" as Serial3.
// Should you wish to use a different serial port such as Serial1 or Serial2, then you must change
// all occurrences of "Serial3" to "Serial1" or "Serial2" in the *.cpp file.

/* 
	 SimpleModbusMaster allows you to communicate
	 to any slave using the Modbus RTU protocol.
	
	 To communicate with a slave you need to create a packet that will contain 
   all the information required to communicate to the slave.
   Information counters are implemented for further diagnostic.
   These are variables already implemented in a packet. 
   You can set and clear these variables as needed.
   
   The following modbus information counters are implemented:    
   
   requests - contains the total requests to a slave
   successful_requests - contains the total successful requests
	 failed_requests - general frame errors, checksum failures and buffer failures 
   retries - contains the number of retries
   exception_errors - contains the specific modbus exception response count
	 These are normally illegal function, illegal address, illegal data value
	 or a miscellaneous error response.
	
   And finally there is a variable called "connection" that 
   at any given moment contains the current connection 
   status of the packet. If true then the connection is 
   active. If false then communication will be stopped
   on this packet until the programmer sets the connection
   variable to true explicitly. The reason for this is 
   because of the time out involved in modbus communication.
   Each faulty slave that's not communicating will slow down
   communication on the line with the time out value. E.g.
   Using a time out of 1500ms, if you have 10 slaves and 9 of them
   stops communicating the latency burden placed on communication
   will be 1500ms * 9 = 13,5 seconds!  
   Communication will automatically be stopped after the retry count expires
   on each specific packet.
	
   All the error checking, updating and communication multitasking
   takes place in the background.
  
   In general to communicate with to a slave using modbus
   RTU you will request information using the specific
   slave id, the function request, the starting address
   and lastly the data to request.
   Function 1, 2, 3, 4, 15 & 16 are supported. In addition to
   this broadcasting (id = 0) is supported for function 15 & 16.
	 
   Constants are provided for:
	 Function 1  - READ_COIL_STATUS
	 Function 2  - READ_INPUT_STATUS
   Function 3  - READ_HOLDING_REGISTERS 
	 Function 4  - READ_INPUT_REGISTERS
	 Function 15 - FORCE_MULTIPLE_COILS
   Function 16 - PRESET_MULTIPLE_REGISTERS 
	 
	 Note:  
   The Arduino serial ring buffer is 128 bytes or 64 registers.
   Most of the time you will connect the Arduino using a MAX485 or similar.
 
   In a function 3 or 4 request the master will attempt to read from a
   slave and since 5 bytes is already used for ID, FUNCTION, NO OF BYTES
   and two BYTES CRC the master can only request 122 bytes or 61 registers.
 
   In a function 16 request the master will attempt to write to a 
   slave and since 9 bytes is already used for ID, FUNCTION, ADDRESS, 
   NO OF REGISTERS, NO OF BYTES and two BYTES CRC the master can only write
   118 bytes or 59 registers.
    
   Note:
   Using a USB to Serial converter the maximum bytes you can send is 
   limited to its internal buffer which differs between manufactures. 
 
   Since it is assumed that you will mostly use the Arduino to connect without
   using a USB to Serial converter the internal buffer is set the same as the
   Arduino Serial ring buffer which is 128 bytes.
*/

#include "Arduino.h"

#define READ_COIL_STATUS 1 // Reads the ON/OFF status of discrete outputs (0X references, coils) in the slave.
#define READ_INPUT_STATUS 2 // Reads the ON/OFF status of discrete inputs (1X references) in the slave.
#define READ_HOLDING_REGISTERS 3 // Reads the binary contents of holding registers (4X references) in the slave.
#define READ_INPUT_REGISTERS 4 // Reads the binary contents of input registers (3X references) in the slave. Not writable.
#define FORCE_SINGLE_COIL 5
#define FORCE_MULTIPLE_COILS 15 // Forces each coil (0X reference) in a sequence of coils to either ON or OFF.
#define PRESET_MULTIPLE_REGISTERS 16 // Presets values into a sequence of holding registers (4X references).

typedef struct
{
  // specific packet info
  unsigned char id;
  unsigned char function;
  unsigned int address;
	// For functions 1 & 2 data is the number of points
  // For functions 3, 4 & 16 data is the number of registers
  // For function 15 data is the number of coils
  unsigned int data; 
  unsigned int* register_array;
  
  // modbus information counters
  unsigned int requests;
  unsigned int successful_requests;
	unsigned int failed_requests;
	unsigned int exception_errors;
  unsigned int retries;
  	
  // connection status of packet
  unsigned char connection; 
  
}Packet;

typedef Packet* packetPointer;

// function definitions
void modbus_update();

void modbus_construct(Packet *_packet, 
											unsigned char id, 
											unsigned char function, 
											unsigned int address, 
											unsigned int data, 
											unsigned int* register_array);
											
void modbus_configure(						long baud, 
											unsigned char byteFormat,
											unsigned int _timeout, 
											unsigned int _polling, 
											unsigned char _retry_count, 
											unsigned char _TxEnablePin,
											Packet* _packets, 
											unsigned int _total_no_of_packets);

#endif