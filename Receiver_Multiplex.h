/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
  
  Multiplex Receiver Serial 
  Copyright (c) 2012 Mattias Welponer <mattias@welponer.net>.  All rights reserved.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef _AEROQUAD_RECEIVER_MULTIPLEX_H_
#define _AEROQUAD_RECEIVER_MULTIPLEX_H_

//#include <Receiver.h>
#include <Receiver_base_MEGA.h>

#ifndef RECEIVER_MULTIPLEX_SERIAL
  #define RECEIVER_MULTIPLEX_SERIAL Serial1
#endif

// Serial Multiplex protocol 
// StartByte (0xA1), 24 Servo high&low byte, 2 byte CRC16

#define MULTIPLEX_RX_STARTBYTE 0xa1
#define MULTIPLEX_RX_LENGHT 24
#define MULTIPLEX_RX_BAUD 115200

char ServoData[24];  
unsigned short CheckSum;
unsigned short ServoData_Count;  
bool CheckSum_OK;
unsigned short CheckSum_CRC16;

// CRC16 
static unsigned short TableCRC16[256];
void initializeCRC16_CCITT( void);
unsigned short calcCRC16_CCITT( unsigned short crc, char c);
#define CRC16_CCITT_POLY 0x1021

//void initializeReceiver( int nbChannel) {
void initializeReceiverMPX() {
//  initializeReceiverParam(nbChannel);
  initializeCRC16_CCITT();
  RECEIVER_MULTIPLEX_SERIAL.begin(MULTIPLEX_RX_BAUD);
  CheckSum_OK = false;
}

/* void ReceiverMPX(void) {

}  */
  
void readMultiplexSerialReceiver( void) {    
  while (RECEIVER_MULTIPLEX_SERIAL.available()) {
    char inByte = RECEIVER_MULTIPLEX_SERIAL.read();

    if ((inByte == MULTIPLEX_RX_STARTBYTE) && (ServoData_Count >= MULTIPLEX_RX_LENGHT + 2)) {
      ServoData_Count = 0;
      CheckSum_CRC16 = calcCRC16_CCITT(0x0000, MULTIPLEX_RX_STARTBYTE);   // start byte also part of the CRC 
    } else if (ServoData_Count <= MULTIPLEX_RX_LENGHT - 1) {
        ServoData[ServoData_Count] = inByte;
        CheckSum_CRC16 = calcCRC16_CCITT(CheckSum_CRC16, inByte);
        ServoData_Count++;
        //SerialUSB.print(inByte, DEC); SerialUSB.print(", ");
    } else if (ServoData_Count == MULTIPLEX_RX_LENGHT) {
        CheckSum = inByte; 
        ServoData_Count++;
        //SerialUSB.println("");
    } else if (ServoData_Count == MULTIPLEX_RX_LENGHT + 1) {
        CheckSum = ((CheckSum << 8) + inByte);  
        CheckSum_OK = (CheckSum == CheckSum_CRC16);
        ServoData_Count++;
    }
  }    
}

void setChannelValue(byte channel,int value) {
    receiverCommand[channel] = value;
}

//int getRawChannelValue(byte channel) {
int getRawChannelValueMPX(const byte channel) {
  if (channel == XAXIS) {
    readMultiplexSerialReceiver();
  }
  if (CheckSum_OK) {
    receiverCommand[channel] = ((ServoData[channel*2] << 8) + ServoData[channel*2+1]);
  }
  return receiverCommand[channel];
}

unsigned short calcCRC16_CCITT( unsigned short crc, char c) {
  unsigned short tmp, short_c;
  
  short_c  = 0x00ff & (unsigned short) c;
  tmp = (crc >> 8) ^ short_c;
  crc = (crc << 8) ^ TableCRC16[tmp];
  return crc;
} 

void initializeCRC16_CCITT( void) {
  int i, j;
  unsigned short crc, c;
  
  for (i=0; i<256; i++) {
    crc = 0;
    c   = ((unsigned short) i) << 8;
    for (j=0; j<8; j++) {
      if ( (crc ^ c) & 0x8000 ) crc = ( crc << 1 ) ^ CRC16_CCITT_POLY;
      else  crc =   crc << 1;
      c = c << 1;
    }
    TableCRC16[i] = crc;
  }
} 


void initializeReceiverPWM() {

}


int getRawChannelValuePWM(const byte channel) {
	return 0;
}

void initializeReceiverPPM() {

}


int getRawChannelValuePPM(const byte channel) {
	return 0;
}

void initializeReceiverSBUS() {

}


int getRawChannelValueSBUS(const byte channel) {
	return 0;
}



#endif


