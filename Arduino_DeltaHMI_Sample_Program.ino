#include <SimpleModbusMaster.h>
#include <TimerOne.h>

// AET edit of the original "SimpleModbusMasterV10" example from www.arduino.cc
// This edited program implements the HMI end ONLY (For now... see later tutorials)
// of an Arduino communicating with a DELTA HMI to control a Schneider Variable Speed Drive (VSD)
// over MODBUS RTU, using the AET RS485 shield.

// 27/12/2013 - Made available as sample program under GPL-3.0.

// AET shall not be liable for any damages, injury, or legal claims sustained through use of this code.
// It is intended for learning purposes only!!!
// Your safety and that of others is your own responsibility!!!

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
   
   The example will use packet1 to read a register from address 0 (the adc ch0 value)
   from the arduino slave. It will then use this value to adjust the brightness
   of an led on pin 9 using PWM.
   It will then use packet2 to write a register (its own adc ch0 value) to address 1 
   on the arduino slave adjusting the brightness of an led on pin 9 using PWM.
*/

//////////////////// Port information ///////////////////
#define baud 19200
#define timeout 1000
#define polling 10 // the scan rate

// If the packets internal retry register matches
// the set retry count then communication is stopped
// on that packet. To re-enable the packet you must
// set the "connection" variable to true.
#define retry_count 10

// used to toggle the receive/transmit pin on the driver
#define TxEnablePin 20 

#define LED 9



// This is the easiest way to create new packets
// Add as many as you want. TOTAL_NO_OF_PACKETS
// is automatically updated.
enum
{
  PACKET1,
  PACKET2,
  PACKET3,
  PACKET4,
  PACKET5,
  PACKET6,
  PACKET7,
  PACKET8,
  PACKET9,
  PACKET10,
  PACKET11,
  PACKET12,
  PACKET13,
  PACKET14,
  PACKET15,
  PACKET16,
  PACKET17,
  TOTAL_NO_OF_PACKETS // leave this last entry
};

// Create an array of Packets to be configured
Packet packets[TOTAL_NO_OF_PACKETS];

// Create a packetPointer to access each packet
// individually. This is not required you can access
// the array explicitly. E.g. packets[PACKET1].id = 2;
// This does become tedious though...
//////////////////////////HMI READS/////////////////////////////
packetPointer enableDisableHMI = &packets[PACKET1];
packetPointer runFwdHMI = &packets[PACKET2];
packetPointer runRevHMI = &packets[PACKET3];
packetPointer resetHMI = &packets[PACKET4];
packetPointer eStopHMI = &packets[PACKET5];
packetPointer userSetSpeedHMI = &packets[PACKET6];

//////////////////////////HMI WRITES////////////////////////////
packetPointer runLedHMI = &packets[PACKET7];
packetPointer stopLedHMI = &packets[PACKET8];
packetPointer errorLedHMI = &packets[PACKET9];
packetPointer actualSpeedHMI = &packets[PACKET10];
packetPointer motorCurrentHMI = &packets[PACKET11];

/////////////////////////VSD READS//////////////////////////////
packetPointer statusWordVSD = &packets[PACKET12];
packetPointer actualSpeedVSD = &packets[PACKET13];
packetPointer motorCurrentVSD = &packets[PACKET14];

/////////////////////////VSD WRITES/////////////////////////////
packetPointer commandWordVSD = &packets[PACKET15];
packetPointer userSetSpeedVSD = &packets[PACKET16];
packetPointer clearFaultsVSD = &packets[PACKET17];

////////HMI READ VARIABLES////////////
unsigned int readEnableDisableHMI[1];
unsigned int readRunFwdHMI[1];
unsigned int readRunRevHMI[1];
unsigned int readResetHMI[1];
unsigned int readEstopHMI[1];
unsigned int readUserSetSpeedHMI[1];

////////HMI WRITE VARIABLES//////////
unsigned int writeRunLedHMI[1];
unsigned int writeStopLedHMI[1];
unsigned int writeErrorLedHMI[1];
unsigned int writeActualSpeedHMI[1];
unsigned int writeMotorCurrentHMI[1];

////////VSD READ VARIABLES///////////
unsigned int readStatusWordVSD[1];
unsigned int readActualSpeedVSD[1];
unsigned int readMotorCurrentVSD[1];

////////VSD WRITE VARIABLES//////////
unsigned int writeControlWordVSD[1];
unsigned int writeUserSetSpeedVSD[1]={0};
unsigned int writeClearFaultsVSD[1];


// High or Low variables as arrays
unsigned int writeHigh[1]={1};
unsigned int writeLow[1]={0};

const int vccPin =  17;
const int gndPin =  21;

unsigned int BUTTONSTATE;
unsigned int PREVBUTTONSTATE;

void setup()
{
   Serial.begin(19200);
   
  // Set modes of some pins for LED outputs 
  pinMode(vccPin, OUTPUT);
  pinMode(gndPin, OUTPUT);
  digitalWrite(vccPin, HIGH);
  digitalWrite(gndPin, LOW);
   
  // Read all values from HMI  
  modbus_construct(enableDisableHMI, 3, READ_HOLDING_REGISTERS, 50, 1, readEnableDisableHMI);
  modbus_construct(runFwdHMI, 3, READ_HOLDING_REGISTERS, 60, 1, readRunFwdHMI);
  modbus_construct(runRevHMI, 3, READ_HOLDING_REGISTERS, 70, 1, readRunRevHMI);
  modbus_construct(resetHMI, 3, READ_HOLDING_REGISTERS, 80, 1, readResetHMI);
  modbus_construct(eStopHMI, 3, READ_HOLDING_REGISTERS, 90, 1, readEstopHMI);
  modbus_construct(userSetSpeedHMI, 3, READ_HOLDING_REGISTERS, 10, 1, readUserSetSpeedHMI);
  
  // Write required values to HMI
  modbus_construct(runLedHMI, 3, PRESET_MULTIPLE_REGISTERS, 100, 1, writeRunLedHMI);
  modbus_construct(stopLedHMI, 3, PRESET_MULTIPLE_REGISTERS, 110, 1, writeStopLedHMI);
  modbus_construct(errorLedHMI, 3, PRESET_MULTIPLE_REGISTERS, 120, 1, writeErrorLedHMI);
  modbus_construct(actualSpeedHMI, 3, PRESET_MULTIPLE_REGISTERS, 0, 1, readActualSpeedVSD);
  modbus_construct(motorCurrentHMI, 3, PRESET_MULTIPLE_REGISTERS, 20, 1, readMotorCurrentVSD);
  
  // Read all values from VSD
  modbus_construct(statusWordVSD, 2, READ_HOLDING_REGISTERS, 8603, 1, readStatusWordVSD);
  modbus_construct(actualSpeedVSD, 2, READ_HOLDING_REGISTERS, 8604, 1, readActualSpeedVSD);
  modbus_construct(motorCurrentVSD, 2, READ_HOLDING_REGISTERS, 3204, 1, readMotorCurrentVSD);
  
  // Write required values to VSD
  modbus_construct(commandWordVSD, 2, PRESET_MULTIPLE_REGISTERS, 8601, 1, writeControlWordVSD);
  modbus_construct(userSetSpeedVSD, 2, PRESET_MULTIPLE_REGISTERS, 8602, 1, readUserSetSpeedHMI);
  modbus_construct(clearFaultsVSD, 2, PRESET_MULTIPLE_REGISTERS, 8501, 1, writeClearFaultsVSD);
  
  // Configure the MODBUS connection
  modbus_configure(baud, SERIAL_8E1, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS);
  
  pinMode(LED, OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(5,OUTPUT);
  digitalWrite(5,LOW);
  pinMode(3,OUTPUT);
  pinMode(2,OUTPUT);
  digitalWrite(2,LOW);
  pinMode(11,OUTPUT);
  pinMode(10,OUTPUT);
  digitalWrite(10,LOW);
}

void loop()
{
  modbus_update();  // Get the latest register values for the buttons and screen elements...
  checkState();     // Convert the button register states into a single integer value for use in switch|case statement...
  
  switch(BUTTONSTATE){
   
   case 0:
        
        while(BUTTONSTATE==0)
        {
          modbus_update();
          checkState();
          Serial.println(BUTTONSTATE);
          // Your Code Here...
        }
          
   break;
   
   case 1:
                          
        while(BUTTONSTATE==1)
        {
         modbus_update();
         checkState();
         Serial.println(BUTTONSTATE);
         // Your Code Here...
        }
                
   break;
   
   case 2:
        
        while(BUTTONSTATE==2)
        {
          modbus_update();
          checkState();
          Serial.println(BUTTONSTATE);
          // Your Code Here...
        }
                
   break;
   
   case 3:
   
       while(BUTTONSTATE==3)
        {
          modbus_update();
          checkState(); 
          Serial.println(BUTTONSTATE); 
          // Your Code Here...
        }
                
   break;
   
   case 4:
        
        while(BUTTONSTATE==4)
        {
          modbus_update();
          checkState();
          Serial.println(BUTTONSTATE); 
          // Your Code Here...
        }
        
   break;
   
   case 5:
       
       while(BUTTONSTATE==5)
        {
          modbus_update();
          checkState();
          Serial.println(BUTTONSTATE);
          // Your Code Here...
        }
         
   break;
   
   case 6:
   
        while(BUTTONSTATE==6)
        {
          modbus_update();
          checkState();
          Serial.println(BUTTONSTATE);
        // Your Code Here...
        }
        
   break;
   
   case 7:
        
        while(BUTTONSTATE==7)
        {
          modbus_update();
          checkState();
          Serial.println(BUTTONSTATE);
        // Your Code Here...
        }
        
   break;
   
   case 8:
        
        while(BUTTONSTATE==8)
        {
          modbus_update();
          checkState();
          Serial.println(BUTTONSTATE);
        // Your Code Here...
        }
        
   break;
   
   case 9:
        
        while(BUTTONSTATE==9)
        {
          modbus_update();
          checkState();
          Serial.println(BUTTONSTATE);
        // Your Code Here...
        }
        
   break;
   
   case 10:
        
        while(BUTTONSTATE==10)
        {
          modbus_update();
          checkState();
          Serial.println(BUTTONSTATE);
        // Your Code Here...
        }
        
   break;
   
   case 16:
   
        while(BUTTONSTATE==16)
        {
          modbus_update();
          checkState();
          Serial.println(BUTTONSTATE);
        // Your Code Here...
        }   
   break;
   
   default:
   
   break;
    
  }
   
}

void checkState() // This function simply reads whether each button is active("1") or inactive ("0") and bitshifts each button to create a unique BUTTONSTATE value for each combination of button presses...
{
  PREVBUTTONSTATE=BUTTONSTATE;
      
  BUTTONSTATE = (readEnableDisableHMI[0])+(readRunFwdHMI[0]<<1)+(readRunRevHMI[0]<<2)+(readResetHMI[0]<<3)+(readEstopHMI[0]<<4); // Example: Drive enabled, and run forward... 0b00011 = dec "3"... so BUTTONSTATE = 3.
  
  if(BUTTONSTATE==11||BUTTONSTATE==12||BUTTONSTATE==13||BUTTONSTATE==14||BUTTONSTATE==15) // For our implementation these button press combinations are not valid... so just make them = to state 10...
  {
    BUTTONSTATE=10;
  }
  // As above... These are all the possible BUTTONSTATE values when the E-stop button is pressed... but if E-Stop is pressed we want to kill all buttons anyway...
  if(BUTTONSTATE==17||BUTTONSTATE==18||BUTTONSTATE==19||BUTTONSTATE==20||BUTTONSTATE==21||BUTTONSTATE==22||BUTTONSTATE==23||BUTTONSTATE==24||BUTTONSTATE==25||BUTTONSTATE==26||BUTTONSTATE==27||BUTTONSTATE==28||BUTTONSTATE==29||BUTTONSTATE==30||BUTTONSTATE==31)
  {
    BUTTONSTATE=16;
  }
      
}


