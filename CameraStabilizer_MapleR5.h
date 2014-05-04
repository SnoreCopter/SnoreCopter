/*
  AeroQuad v3.0 - April 2011
  www.AeroQuad.com 
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
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

#ifndef _SNORQUAD_CAMERA_STABILIZER_MAPLER5_H_
#define _SNORQUAD_CAMERA_STABILIZER_MAPLER5_H_

#include <WProgram.h>
#include <CameraStabilizer.h>

#define SERVO_FREQUENCY 50 // 50 Hz for analog servo
//#define SERVO_FREQUENCY 400 // 300 Hz for digital high speed servo

#define SERVO_PERIOD (1000000/SERVO_FREQUENCY)
#define PRESCALER CPU_FREQ/1000000 - 1

#define SERVOPIN1 14
#define SERVOPIN2 24
//#define CAMERAPIN3 24


void initializeCameraControl() {
    
  Timer4.setPrescaleFactor(PRESCALER);
  Timer4.setOverflow(SERVO_PERIOD);
  
  pinMode(SERVOPIN1, PWM);
  pinMode(SERVOPIN2, PWM);
//  pinMode(CAMERAPIN3, PWM);

  cameraControlMove(1500, 1500, 1500);
}

void cameraControlMove(int servoPitch, int servoRoll, int servoYaw) {

  Timer4.setCompare(TIMER_CH3, servoPitch);
  Timer4.setCompare(TIMER_CH4, servoRoll);
//  Timer4.setCompare(TIMER_CH2, servoYaw);
}


#endif


