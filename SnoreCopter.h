


#undef SERIAL_BEGIN
#define SERIAL_BEGIN(...) ""

#define DEBUG_PRINT2(str, str2) SerialUSB.print(str); SerialUSB.print(str2); 
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

  #include <FlightConfigMEGA.h>

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
    #define BattDefaultConfig DEFINE_BATTERY(0, 15, 13.6, 0, 16, 45.1, 0)
  #endif

  #ifdef OSD
    #define MAX7456_OSD
  #endif

  #define MULTIPLEX_TELEMETRY
  #ifdef MULTIPLEX_TELEMETRY
    #include <Telemetry_Multiplex.h>
  #endif

class Device {
public:

};

class DeviceI2C: public Device {
  boolean _status;
public:
  DeviceI2C() { _status = false; }
  void initialize(void) { Wire.begin(); _status = true; }
  byte requestI2C(int deviceAddress, int numBytes) { return Wire.requestFrom(deviceAddress, numBytes); }
 
};

class DeviceSPI: public Device {
public:
  DeviceSPI() {}
};

class Sensor {
protected:
  boolean _status; 
public:
  Device _device;
  Sensor() { _status = false; }
  Sensor(Device &d) { _device = d; _status = false; }
  
  boolean status(void) { 
    SerialUSB.print("Sensor "); 
    if (_status) SerialUSB.println("OK"); else SerialUSB.println("NOK"); 
    return _status; 
  }
  void intitialize(void) { SerialUSB.println("Error "); _status = false; }
  void deactivate(void) {}
  void measure(void) {}
  int read(byte channel) { return 0; }
};

class SensorITG3200: public Sensor {
public:  
  SensorITG3200(Device &d): Sensor(d) {}
  
  #include <Gyroscope_ITG3200.h>	
   
  void intitialize(void) { SerialUSB.println("ITG3200: init "); _status = true; }
  boolean status(void) { SerialUSB.print("ITG3200: "); 
    if (_status) SerialUSB.println("OK"); else SerialUSB.println("NOK"); 
    return _status; }
};

class SensorBMA180: public Sensor {
public:  
  SensorBMA180(Device &d): Sensor(d) {}
  
  #include <Accelerometer_BMA180.h>	 
  void intitialize(void) { SerialUSB.println("BMA180: init "); _status = true; }
  boolean status(void) { SerialUSB.print("BMA180"); 
    if (_status) SerialUSB.println("OK"); else SerialUSB.println("NOK"); 
    return _status; }
};

class SensorMPU6000: public Sensor {
public:
  SensorMPU6000(Device &d): Sensor(d) {}
  SensorMPU6000(byte addr): Sensor() {}
  
  #define MPU6000_I2C
  #define MPU6000_I2C_ADDRESS 0x69
  #include <Gyroscope_MPU6000.h>
  #include <Accelerometer_MPU6000.h>	 
  
  void intitialize(void) { SerialUSB.println("MPU6000: init "); _status = true; }
  boolean status(void) { SerialUSB.println("MPU6000"); 
    if (_status) SerialUSB.println("OK"); else SerialUSB.println("NOK"); 
    return _status;}
};


class SensorHMC5883L: public Sensor {
public:
  SensorHMC5883L(Device &d): Sensor(d) {}
  #define HMC5883L
  #include <Magnetometer_HMC5883L.h>
  void intitialize(void) { SerialUSB.println("HMC5883L: init "); _status = true; }
  boolean status(void) { SerialUSB.println("HMC5883L"); if (_status) SerialUSB.println("OK"); else SerialUSB.println("NOK"); 
    return _status;}
};


  /**
   * Put SnoreCopter specific intialization need here
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

    DeviceI2C DevI2C;

    Sensor SensNone;
    SensorMPU6000 SensMPU6000(DevI2C);
    SensorITG3200 SensITG3200(DevI2C);
    SensorBMA180 SensBMA180(DevI2C);
    SensorHMC5883L SensHMC5883L(DevI2C);
    
    boolean mpu6000 = true;
    boolean hmc5883l = false;
    
    Sensor SensorGyro;
    Sensor SensorAccel;
    Sensor SensorMag;
    
    SensorGyro = SensMPU6000;
    SensorAccel = SensMPU6000;
    SensorMag = SensHMC5883L;
    
    
    SensorGyro.status();
    SensorAccel.status();
    SensorMag.status();

    

    inititializeTelemetry();
	
    #ifdef MavLink
      SerialMavlink.begin(57600);
    #endif
	
      switch (flightConfigType) 
      {
        case OCTO_X :
        case OCTO_PLUS :
        case OCTO_X8 :
          LASTMOTOR = 8;
          break;
        case HEX_Y6 :
        case HEX_PLUS :
        case HEX_X :
          LASTMOTOR = 6;
          break;
        default:
          LASTMOTOR = 4;
      }
  }

  /**
   * Put SnoreCopter specific intialization need here
   */
  void initPlatformEEPROM(void) {
    flightMode = ATTITUDE_FLIGHT_MODE;
    //headingHoldConfig = ON;  
	headingHoldState    = ON;
    minArmedThrottle = 1100;
#ifdef SnorCopter_MapleDroTek		// DroTek ITG3200/BMA180
    flightConfigType = HEX_Y6;	 
#else //SnorCopter_MapleDroTek2    	// DroTek MPU6050
    flightConfigType = QUAD_X;	 
#endif
	LASTMOTOR = 6; 
	receiverTypeUsed = 1;
	LAST_CHANNEL = 8;
    yawDirection = 1;
	
    for (byte channel = XAXIS; channel < LASTCHANNEL; channel++) {
      receiverMinValue[channel] = 439;
	  receiverMaxValue[channel] = 3655;	 
      //receiverSmoothFactor[channel] = 1.0;
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
	/*    PID[ATTITUDE_GYRO_XAXIS_PID_IDX].P = PID[RATE_XAXIS_PID_IDX].P - (0.1*PID[RATE_XAXIS_PID_IDX].P);
    PID[ATTITUDE_GYRO_XAXIS_PID_IDX].I = PID[RATE_XAXIS_PID_IDX].I ;
    PID[ATTITUDE_GYRO_XAXIS_PID_IDX].D = PID[RATE_XAXIS_PID_IDX].D;
    PID[ATTITUDE_GYRO_YAXIS_PID_IDX].P = PID[RATE_YAXIS_PID_IDX].P - (0.1*PID[RATE_YAXIS_PID_IDX].P);
    PID[ATTITUDE_GYRO_YAXIS_PID_IDX].I = PID[RATE_YAXIS_PID_IDX].I;
    PID[ATTITUDE_GYRO_YAXIS_PID_IDX].D = PID[RATE_YAXIS_PID_IDX].D;   */
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
  //unsigned long previousMeasureCriticalSensorsTime = 0;
  void measureCriticalSensors() {
    #ifdef SnorCopter_MapleDroTek2
	  readMPU6000Sensors();
    #endif
	// read sensors not faster than every 1 ms
    //if (currentTime - previousMeasureCriticalSensorsTime >= 1000) {
	  measureGyroSum();
	  measureAccelSum();
	//  previousMeasureCriticalSensorsTime = currentTime;
	//}
  }  
  
#define MAPLE_ADC_TEST
#ifdef MAPLE_ADC_TEST

#endif



    
    
  void processSnoreCopter50Hz() {
  
/*  
  SerialUSB.println(">");
  SerialUSB.print(batteryData[0].voltage/1000.0);   // x 1 mV
  SerialUSB.print(", ");
  SerialUSB.print(batteryData[0].current/100.0);
  SerialUSB.print(", ");
  SerialUSB.print(batteryData[0].usedCapacity/1000.0);
*/
  
    #if defined(BUZZER_PIN) && defined(BattMonitor) 
      digitalWrite(BUZZER_PIN, batteryAlarm && motorArmed);
    #endif
    #ifdef MULTIPLEX_TELEMETRY
      updateTelemtry();
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
/*    PID[ATTITUDE_GYRO_XAXIS_PID_IDX].P = PID[RATE_XAXIS_PID_IDX].P - (0.2*PID[RATE_XAXIS_PID_IDX].P);
    PID[ATTITUDE_GYRO_XAXIS_PID_IDX].I = PID[RATE_XAXIS_PID_IDX].I ;
    PID[ATTITUDE_GYRO_XAXIS_PID_IDX].D = PID[RATE_XAXIS_PID_IDX].D;
    PID[ATTITUDE_GYRO_YAXIS_PID_IDX].P = PID[RATE_YAXIS_PID_IDX].P - (0.2*PID[RATE_YAXIS_PID_IDX].P);
    PID[ATTITUDE_GYRO_YAXIS_PID_IDX].I = PID[RATE_YAXIS_PID_IDX].I;
    PID[ATTITUDE_GYRO_YAXIS_PID_IDX].D = PID[RATE_YAXIS_PID_IDX].D; */
#endif
  }

#endif

