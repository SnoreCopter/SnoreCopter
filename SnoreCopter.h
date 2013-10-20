


#undef SERIAL_BEGIN
#define SERIAL_BEGIN(...) ""

#define DEBUG_PRINT(str) SerialUSB.print(str); SerialUSB.print(" ");
#define DEBUG_PRINTLN2(str, str2) SerialUSB.print(str); SerialUSB.println(str2); 
#define DEBUG_PRINTLN(str)  SerialUSB.println(str); 
 
  
/*
  Serial1: Multiplex Receiver
  Serial2: Mavlink or MPXsensor
  Serial3: GPS
*/  

#if defined(SnorCopter_MapleCSG) || defined(SnorCopter_MapleDroTek) || defined(SnorCopter_MapleDroTek2)  
  #define LED_Green 13
  #define LED_Red 35
  #define LED_Yellow 36
  #define BUZZER_PIN 20

  // EEPROM emulation
  #define EEPROM_USES_16BIT_WORDS
  
  // Serial
  #define SERIAL_USES_USB
  
//  #define I2C_HARDWARE
//  #define I2C_HARDWARE_PORT 1
  #include <Device_I2C.h>

  // Gyroscope declaration
  #ifdef SnorCopter_MapleDroTek2
    #define MPU6000_I2C
    #define MPU6000_I2C_ADDRESS 0x69
    #include <Gyroscope_MPU6000.h>			// DroTek MPU6050
  #else
    #ifdef SnorCopter_MapleCSG
      #define ITG3200_ADDRESS_ALTERNATE     // for CSG
    #endif
    #include <Gyroscope_ITG3200.h>			// DroTek 
  #endif
  
  // Accelerometer declaration
  #ifdef SnorCopter_MapleDroTek2
    #include <Accelerometer_MPU6000.h>		// DroTek MPU6050
  #else
    #ifdef SnorCopter_MapleCSG
      #define BMA180_ADDRESS_ALTERNATE      // for CSG
    #endif
    #include <Accelerometer_BMA180.h>		// DroTek 
  #endif 
  
  // Receiver Declaration
  #define RECEIVER_MULTIPLEX
  #define RECEIVER_MULTIPLEX_SERIAL Serial1
  #include <Receiver_Multiplex.h>

  // Motor Declaration
  #include <Motors_MapleR5.h>

  // Heading mag hold declaration
  #ifdef HeadingMagHold
    #define HMC5883L
    #include <Magnetometer_HMC5883L.h>
  #endif
  
  #ifdef AltitudeHoldRangeFinder
    #define XLMAXSONAR 
  #endif

  // Altitude declaration
  #ifdef AltitudeHoldBaro
    #if defined(SnorCopter_MapleDroTek) || defined(SnorCopter_MapleDroTek2)
      #define MS5611_ADDRESS_ALTERNATE
      #define MS5611			// for DroTek sensor board
    #else
      #define BMP085      		// for CSG sensor board
    #endif
  #endif

  // Battery monitor declaration
  #ifdef BattMonitor
    //#define BattDefaultConfig DEFINE_BATTERY(0, 15, 13.3, 0.0, 16, 25.0, 0.0)
    // AQ32 attopilot 45A voltage=A1 current=A2
    #define BattDefaultConfig DEFINE_BATTERY(0, 15, 13.6, 0, 16, 45.1, 0)
  #endif

  #ifdef OSD
    #define MAX7456_OSD
  #endif


//#define MULTIPLEX_TELEMETRY
#ifdef MULTIPLEX_TELEMETRY
  #include <SoftwareSerial_NB.h>
  SoftwareSerial_NB mySerial(33, 34); // RX, TX
#endif

  /**
   * Put SnorCopter specific intialization need here
   */
  void initPlatform() {
    pinMode(LED_Red, OUTPUT);
    pinMode(LED_Yellow, OUTPUT);
    pinMode(LED_Green, OUTPUT);
    #ifdef BUZZER_PIN 
      pinMode(BUZZER_PIN, OUTPUT);
      digitalWrite(BUZZER_PIN, 0);
    #endif

    initializeI2C();

    //Serial2.begin(38400);

    #ifdef MavLink
      SerialMavlink.begin(57600);
    #endif
    

  }


  void initPlatformEEPROM(void) {
    flightMode = ATTITUDE_FLIGHT_MODE;
    headingHoldConfig = ON;  
    minArmedThrottle = 1100;

    
    for (byte channel = XAXIS; channel < LASTCHANNEL; channel++) {
      receiverSlope[channel] = 0.3106;
      receiverOffset[channel] = 863.77;
      receiverSmoothFactor[channel] = 1.0;
    }


    PID[RATE_XAXIS_PID_IDX].P = 60.0;
    PID[RATE_XAXIS_PID_IDX].I = 0.0;
    PID[RATE_XAXIS_PID_IDX].D = -5.0*PID[RATE_XAXIS_PID_IDX].P;
    PID[RATE_YAXIS_PID_IDX].P = PID[RATE_XAXIS_PID_IDX].P;
    PID[RATE_YAXIS_PID_IDX].I = PID[RATE_XAXIS_PID_IDX].I;
    PID[RATE_YAXIS_PID_IDX].D = PID[RATE_XAXIS_PID_IDX].D;
    PID[ZAXIS_PID_IDX].P = 1.0*PID[RATE_XAXIS_PID_IDX].P;
    PID[ZAXIS_PID_IDX].I = 5.0;
    PID[ZAXIS_PID_IDX].D = -4.0*PID[ZAXIS_PID_IDX].P;
    PID[ATTITUDE_XAXIS_PID_IDX].P = PID[RATE_XAXIS_PID_IDX].P/20;
    PID[ATTITUDE_XAXIS_PID_IDX].I = 0.0;
    PID[ATTITUDE_XAXIS_PID_IDX].D = 0.0;
    PID[ATTITUDE_YAXIS_PID_IDX].P = PID[ATTITUDE_XAXIS_PID_IDX].P;
    PID[ATTITUDE_YAXIS_PID_IDX].I = PID[ATTITUDE_XAXIS_PID_IDX].I;
    PID[ATTITUDE_YAXIS_PID_IDX].D = PID[ATTITUDE_XAXIS_PID_IDX].D;
    PID[HEADING_HOLD_PID_IDX].P = PID[ATTITUDE_XAXIS_PID_IDX].P;
    PID[HEADING_HOLD_PID_IDX].I = 0.1;
    PID[HEADING_HOLD_PID_IDX].D = 0.0;  
    PID[ATTITUDE_GYRO_XAXIS_PID_IDX].P = PID[RATE_XAXIS_PID_IDX].P - (0.1*PID[RATE_XAXIS_PID_IDX].P);
    PID[ATTITUDE_GYRO_XAXIS_PID_IDX].I = PID[RATE_XAXIS_PID_IDX].I ;
    PID[ATTITUDE_GYRO_XAXIS_PID_IDX].D = PID[RATE_XAXIS_PID_IDX].D;
    PID[ATTITUDE_GYRO_YAXIS_PID_IDX].P = PID[RATE_YAXIS_PID_IDX].P - (0.1*PID[RATE_YAXIS_PID_IDX].P);
    PID[ATTITUDE_GYRO_YAXIS_PID_IDX].I = PID[RATE_YAXIS_PID_IDX].I;
    PID[ATTITUDE_GYRO_YAXIS_PID_IDX].D = PID[RATE_YAXIS_PID_IDX].D;   
  }

  // Called when eeprom is initialized
  void initializePlatformSpecificAccelCalibration() {  
    // Accel Cal
    runTimeAccelBias[XAXIS] = 0.0;
    runTimeAccelBias[YAXIS] = 0.0;
    runTimeAccelBias[ZAXIS] = 0.0;
    #ifdef SnorCopter_MapleDroTek2
      accelScaleFactor[XAXIS] = G_2_MPS2(1.0/8192.0);
    #else
      accelScaleFactor[XAXIS] = G_2_MPS2(1.0/2048.0);  //  g per LSB @ +/- 4g range
    #endif
    accelScaleFactor[YAXIS] = -accelScaleFactor[XAXIS];
    accelScaleFactor[ZAXIS] = -accelScaleFactor[XAXIS];
    storeSensorsZeroToEEPROM();
    #ifdef HeadingMagHold
      magBias[XAXIS]  = 60.000000;
      magBias[YAXIS]  = -39.000000;
      magBias[ZAXIS]  = -7.500000;
    #endif
  }

  /**
   * Measure critical sensors
   */
  unsigned long previousMeasureCriticalSensorsTime = 0;
  void measureCriticalSensors() {
    // read sensors not faster than every 1 ms
    if (currentTime - previousMeasureCriticalSensorsTime >= 1000) {
	  measureGyroSum();
	  measureAccelSum();
	  previousMeasureCriticalSensorsTime = currentTime;
	}
  }
  
  int data = 0;
  
  void updateSnoreCopter10HZ() {
    #if defined(BUZZER_PIN) && defined(BattMonitor) 
    digitalWrite(BUZZER_PIN, batteryAlarm && motorArmed);
    #endif

//#define MULTIPLEX_TELEMETRY
#ifdef MULTIPLEX_TELEMETRY
    byte tele = 0;
    while (Serial2.available()) {
       tele = Serial2.read();
       
        if( tele == 0x02) {
/*    delay(1);
    data = batteryData[0].voltage;
    byte msb = data >> 8;
    byte lsb = data & 0xff;
*/
    Serial2.print(0x22);   // symbol (V, m/s, ....), addr    
    Serial2.print(0);
    Serial2.print(11);
    SerialUSB.print("s ");
}    
       
       if (tele == 0x1a)  SerialUSB.println("");
       if( tele < 0x10) SerialUSB.print("0");
       SerialUSB.print(tele, HEX);
       SerialUSB.print(" ");
    }
    
#endif

#ifdef CONFIG_PIDCH8
    float factor = receiverCommand[7];
    factor -= 1256.0;
    factor /= 125.6;
    factor /= 3;

    PID[RATE_XAXIS_PID_IDX].P = 75.0 + 75.0 * factor;
    PID[RATE_XAXIS_PID_IDX].I = 0.0;
    PID[RATE_XAXIS_PID_IDX].D = -4.0*PID[RATE_XAXIS_PID_IDX].P;
    PID[RATE_YAXIS_PID_IDX].P = PID[RATE_XAXIS_PID_IDX].P;
    PID[RATE_YAXIS_PID_IDX].I = PID[RATE_XAXIS_PID_IDX].I;
    PID[RATE_YAXIS_PID_IDX].D = PID[RATE_XAXIS_PID_IDX].D;
#ifdef triConfig
    PID[ZAXIS_PID_IDX].P = 1.0*PID[RATE_XAXIS_PID_IDX].P;
#else
    PID[ZAXIS_PID_IDX].P = 2.0*PID[RATE_XAXIS_PID_IDX].P;
#endif
    PID[ZAXIS_PID_IDX].I = 5.0;
    PID[ZAXIS_PID_IDX].D = -4.0*PID[ZAXIS_PID_IDX].P;
    PID[ATTITUDE_XAXIS_PID_IDX].P = PID[RATE_XAXIS_PID_IDX].P/20;
    PID[ATTITUDE_XAXIS_PID_IDX].I = 0.0;
    PID[ATTITUDE_XAXIS_PID_IDX].D = 0.0;
    PID[ATTITUDE_YAXIS_PID_IDX].P = PID[ATTITUDE_XAXIS_PID_IDX].P;
    PID[ATTITUDE_YAXIS_PID_IDX].I = PID[ATTITUDE_XAXIS_PID_IDX].I;
    PID[ATTITUDE_YAXIS_PID_IDX].D = PID[ATTITUDE_XAXIS_PID_IDX].D;
    PID[HEADING_HOLD_PID_IDX].P = 3.0;
    PID[HEADING_HOLD_PID_IDX].I = 0.1;
    PID[HEADING_HOLD_PID_IDX].D = 0.0;  
    PID[ATTITUDE_GYRO_XAXIS_PID_IDX].P = PID[RATE_XAXIS_PID_IDX].P - (0.2*PID[RATE_XAXIS_PID_IDX].P);
    PID[ATTITUDE_GYRO_XAXIS_PID_IDX].I = PID[RATE_XAXIS_PID_IDX].I ;
    PID[ATTITUDE_GYRO_XAXIS_PID_IDX].D = PID[RATE_XAXIS_PID_IDX].D;
    PID[ATTITUDE_GYRO_YAXIS_PID_IDX].P = PID[RATE_YAXIS_PID_IDX].P - (0.2*PID[RATE_YAXIS_PID_IDX].P);
    PID[ATTITUDE_GYRO_YAXIS_PID_IDX].I = PID[RATE_YAXIS_PID_IDX].I;
    PID[ATTITUDE_GYRO_YAXIS_PID_IDX].D = PID[RATE_YAXIS_PID_IDX].D; 
#endif
  }

#endif

