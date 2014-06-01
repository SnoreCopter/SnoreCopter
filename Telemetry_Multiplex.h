#ifndef _AEROQUAD_TELEMETRY_MULTIPLEX_H_
#define _AEROQUAD_TELEMETRY_MULTIPLEX_H_

#ifndef TELEMETRY_MULTIPLEX_SERIAL
  #define TELEMETRY_MULTIPLEX_SERIAL Serial2
#endif

#define MULTIPLEX_NONE		0x00
#define MULTIPLEX_VOLTAGE	0x01    // 0,1 V
#define MULTIPLEX_CURRENT	0x02	// 0,1 A	
#define MULTIPLEX_VARIO		0x03	// 0,1 m/s
#define MULTIPLEX_SPEED		0x04	// 0,1 km/h
#define MULTIPLEX_RPM		0x05	// 100 rpm bzw. 10 rpm
#define MULTIPLEX_TEMP		0x06	// 0,1 °C
#define MULTIPLEX_DIR		0x07	// 0,1 Grad
#define MULTIPLEX_ALT		0x08	// 1m
#define MULTIPLEX_TANK		0x09	// 1% Tank
#define MULTIPLEX_LQI		0x0a	// 1% LQI
#define MULTIPLEX_CAP		0x0b	// 1 mAh
#define MULTIPLEX_FLUID		0x0c	// 1 mL
#define MULTIPLEX_LDIST		0x0d	// 0,1 km
 
#define MULTIPLEX_NOALERT		0
#define MULTIPLEX_ALERT			1

#define MULTIPLEX_RESET		0x5a
#define MULTIPLEX_UNKNOWN	0x80
  
#define MULTIPLEX_ADDR00  	0x00
#define MULTIPLEX_ADDR01  	0x01
#define MULTIPLEX_ADDR02  	0x02
#define MULTIPLEX_ADDR03  	0x03
#define MULTIPLEX_ADDR04  	0x04
#define MULTIPLEX_ADDR05  	0x05
#define MULTIPLEX_ADDR06  	0x06
#define MULTIPLEX_ADDR07  	0x07
#define MULTIPLEX_ADDR08  	0x08
#define MULTIPLEX_ADDR09  	0x09
#define MULTIPLEX_ADDR0A  	0x0a
#define MULTIPLEX_ADDR0B  	0x0b
#define MULTIPLEX_ADDR0C  	0x0c
#define MULTIPLEX_ADDR0D  	0x0d
#define MULTIPLEX_ADDR0E  	0x0e
#define MULTIPLEX_ADDR0F  	0x0f

/*
0 (1 64 0)        1 (1A C8 0) 2 3 4 5 6 7 8 9 A B C D E F 
0 (1 64 0) (80 0) 1 (1A C8 0) 2 3 4 5 6 7 8 9 A B C D E F
*/  



int Telemetry_Data[16];

byte Multiplex_FrameIndex = 0;
byte Telemetry_SensorIndex = 0;
byte Telemetry_ResetIndex = 0;

    
byte Multiplex_SerialData = 0xff;
byte Multiplex_SerialDataLast = 0xff;
byte Multiplex_DataID = 0;
byte Multiplex_DataType = 0;
byte Multiplex_Data1 = 0;
byte Multiplex_Data0 = 0;

void inititializeTelemetry(void) {
    TELEMETRY_MULTIPLEX_SERIAL.begin(38400);
	
}

void writeMultiplexTelemetry(byte SensorAddress, int SensorData, byte SensorType, byte SensorAlert = 0) {
  if (Multiplex_FrameIndex == SensorAddress) {
	delay(1);
    TELEMETRY_MULTIPLEX_SERIAL.write((SensorAddress << 4) + SensorType);  
    TELEMETRY_MULTIPLEX_SERIAL.write((SensorData << 1) + SensorAlert);
    TELEMETRY_MULTIPLEX_SERIAL.write(SensorData >> 8);  
  }
}

void updateMultiplexTelemetryData() {
  if (Telemetry_Data[MULTIPLEX_ADDR01] > 50)
    writeMultiplexTelemetry(MULTIPLEX_ADDR02, Telemetry_Data[MULTIPLEX_ADDR01], MULTIPLEX_LQI, MULTIPLEX_NOALERT);
  else
    writeMultiplexTelemetry(MULTIPLEX_ADDR02, Telemetry_Data[MULTIPLEX_ADDR01], MULTIPLEX_LQI, MULTIPLEX_ALERT);

#ifdef xxxxxBattMonitor
  writeMultiplexTelemetry(MULTIPLEX_ADDR03, batteryData[0].voltage, MULTIPLEX_VOLTAGE, MULTIPLEX_NOALERT);
  writeMultiplexTelemetry(MULTIPLEX_ADDR04, batteryData[0].current/100.0, MULTIPLEX_CURRENT, MULTIPLEX_NOALERT);
  writeMultiplexTelemetry(MULTIPLEX_ADDR05, batteryData[0].usedCapacity/1000.0, MULTIPLEX_CAP, MULTIPLEX_NOALERT);
#endif
} 

void updateTelemtry(void) {    
  while (TELEMETRY_MULTIPLEX_SERIAL.available()) {
    Multiplex_SerialData = TELEMETRY_MULTIPLEX_SERIAL.read();
	
  if (Telemetry_SensorIndex > 0) { 
	// 3 Byte sensor data 
	if (Telemetry_SensorIndex == 2)   
      Multiplex_Data0 = Multiplex_SerialData;
	if (Telemetry_SensorIndex == 1) {
	  Multiplex_Data1 = Multiplex_SerialData; 
	  Telemetry_Data[Multiplex_DataID] = Multiplex_Data1 + (Multiplex_Data0 >> 1);
	  //#define TELEMETRY_DEBUG test
#ifdef TELEMETRY_DEBUG    
	    SerialUSB.print("<s:");		
		SerialUSB.print(Multiplex_DataID, HEX);
		SerialUSB.print(":");
		SerialUSB.print(Multiplex_DataType, HEX);
		SerialUSB.print(":");
		SerialUSB.print(Telemetry_Data[Multiplex_DataID]);
		SerialUSB.print(">");
#endif
	  }
	Telemetry_SensorIndex--; 
	Multiplex_SerialData = 0xff;							
  } else if (Telemetry_ResetIndex > 0) {
	// 2 Byte unkown command 
	Telemetry_ResetIndex = 0;
	Multiplex_SerialData = 0xff;							
  } else {
	// Frame decoder  
    if (Multiplex_SerialData >> 4 == Multiplex_SerialDataLast) {
 	  Multiplex_DataID = Multiplex_SerialData >> 4;
	  Multiplex_DataType = Multiplex_SerialData & 0x0f;
	  Telemetry_SensorIndex = 2;
    } else if (Multiplex_SerialData == MULTIPLEX_UNKNOWN) {
	  Telemetry_ResetIndex = 1;
    } else if (Multiplex_SerialData == Multiplex_FrameIndex) {
      Multiplex_FrameIndex++;
      updateMultiplexTelemetryData();
    } else {
	//	    for(byte i=0; i<sizeof(Telemetry_Data)/2; i++) 
	//				  Telemetry_Data[i] = 0;
		
    }
		
  } 
  // Check for end of frame 
  if (Multiplex_FrameIndex > 0x0f) { 
	Multiplex_FrameIndex = 0;	
  }
  Multiplex_SerialDataLast = Multiplex_SerialData;
  } // while  
}




#endif // _AEROQUAD_TELEMETRY_MULTIPLEX_H_
