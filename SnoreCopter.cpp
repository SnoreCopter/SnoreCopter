/* 
  SnoreCopter => AeroQuad for Maple
  Mattias@Welponer.net
*/

#ifndef __STM32__
  #define __STM32__
#endif

#include <stdlib.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


#define SnorCopter_MapleDroTek		// DroTek ITG3200/BMA180
//#define SnorCopter_MapleDroTek2    	// DroTek MPU6050
//#define quadXConfig
//#define triConfig
#define hexY6Config
//#define CHANGE_YAW_DIRECTION	// only needed if you want to reverse the yaw correction direction
#define USE_400HZ_ESC			// For ESC that support 400Hz update rate, ESC OR PLATFORM MAY NOT SUPPORT IT
#define HeadingMagHold				// Enables Magnetometer, gets automatically selected if CHR6DM is defined
#define BattMonitor
#define NormalReceiver
#define LASTCHANNEL 8  
#define CONFIG_PIDCH8


#include <WProgram.h>
#include <GlobalDefined.h>
#include <AeroQuad.h>
#include "PID.h"
#include <AQMath.h>
#include <FourtOrderFilter.h>
#include <BatteryMonitorTypes.h>

#include <SnoreCopter.h>
#include "AeroQuad/AeroQuad.ino"

__attribute__((constructor)) void premain() {
    init();
}

int main(void) {   
    delay(2000);
    #ifdef SnorCopter_MapleDroTek
    SerialUSB.println("SnoreCopter MapleDroTek: (" __DATE__ " - "__TIME__")");
    #endif
    #ifdef SnorCopter_MapleDroTek2
    SerialUSB.println("SnoreCopter MapleDroTek2: (" __DATE__ " - "__TIME__")");
    #endif
    delay(3000);
    SerialUSB.println("SnoreCopter: init");
    setup();
    commandAllMotors(MINCOMMAND);
    SerialUSB.println("SnoreCopter: loop");
    while (true) {
      loop();
    }

    return 0;
}

