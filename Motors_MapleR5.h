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

#ifndef _AEROQUAD_MOTORS_MAPLER5_H_
#define _AEROQUAD_MOTORS_MAPLER5_H_

#include <WProgram.h>
#include "Motors.h"

#ifdef triConfig

#define MOTORPIN1 2   // Timer2		// servo

#define MOTORPIN2 12  // Timer3   	// left
#define MOTORPIN3 11				// right
#define MOTORPIN4 27				// rear
#define MOTORPIN5 -1				
#define MOTORPIN6 -1   
#define MOTORPIN7 -1				
#define MOTORPIN8 -1 

#else

#define MOTORPIN1 12  // Timer3   	// left
#define MOTORPIN2 11				// right
#define MOTORPIN3 27				// rear
#define MOTORPIN4 28				// left under
#define MOTORPIN5 2   // Timer2		// right under
#define MOTORPIN6 3					// rear under
#define MOTORPIN7 -1
#define MOTORPIN8 -1 

#endif

#ifdef USE_400HZ_ESC
  #define MOTOR_FREQUENCY 400 // 300 Hz for digital high speed servo
#else
  #define MOTOR_FREQUENCY 50 // 50 Hz for analog servo
#endif
#define SERVO_FREQUENCY 50

#define MOTOR_PERIOD (1000000/MOTOR_FREQUENCY)
#define MOTOR_PRESCALER (CPU_FREQ/1000000 - 1)

#define SERVO_PERIOD (1000000/SERVO_FREQUENCY)
#define SERVO_PRESCALER (CPU_FREQ/1000000 - 1)


void initializeMotors(byte numbers) {

}

void initializeMotors(NB_Motors numbers) {
  numberOfMotors = numbers;
#ifdef triConfig

  Timer3.setPrescaleFactor(MOTOR_PRESCALER);
  Timer3.setOverflow(MOTOR_PERIOD);
  
  pinMode(MOTORPIN2, PWM);
  pinMode(MOTORPIN3, PWM);
  pinMode(MOTORPIN4, PWM);
  
  Timer2.setPrescaleFactor(SERVO_PRESCALER);
  Timer2.setOverflow(SERVO_PERIOD);
    
  pinMode(MOTORPIN1, PWM);

#else    
  Timer3.setPrescaleFactor(MOTOR_PRESCALER);
  Timer3.setOverflow(MOTOR_PERIOD);
  
  pinMode(MOTORPIN1, PWM);
  pinMode(MOTORPIN2, PWM);
  pinMode(MOTORPIN3, PWM);
  pinMode(MOTORPIN4, PWM);

  if (numberOfMotors == SIX_Motors) {
    Timer2.setPrescaleFactor(MOTOR_PRESCALER);
    Timer2.setOverflow(MOTOR_PERIOD);
    
    pinMode(MOTORPIN5, PWM);
    pinMode(MOTORPIN6, PWM);
  }   
#endif  
  
  commandAllMotors(1000);
}

void writeMotors() {
#ifdef triConfig
  Timer2.setCompare(TIMER_CH1, motorCommand[MOTOR1]);
  
  Timer3.setCompare(TIMER_CH1, motorCommand[MOTOR2]);
  Timer3.setCompare(TIMER_CH2, motorCommand[MOTOR3]);
  Timer3.setCompare(TIMER_CH3, motorCommand[MOTOR4]);
#else
  Timer3.setCompare(TIMER_CH1, motorCommand[MOTOR1]);
  Timer3.setCompare(TIMER_CH2, motorCommand[MOTOR2]);
  Timer3.setCompare(TIMER_CH3, motorCommand[MOTOR3]);
  Timer3.setCompare(TIMER_CH4, motorCommand[MOTOR4]);
  
  if (numberOfMotors == SIX_Motors) {  
    Timer2.setCompare(TIMER_CH1, motorCommand[MOTOR5]);
    Timer2.setCompare(TIMER_CH2, motorCommand[MOTOR6]);
  }   
#endif  

}
  
void commandMotor( byte motor, int command) {
  motorCommand[motor] = command;
  writeMotors();
}
  
void commandAllMotors(int command) {
  for( int i = 0; i < 8; i++)    // Todo: LASTMOTOR not know here
    motorCommand[i] = command;
#ifdef triConfig
  if (command < 1200 ) motorCommand[0] = MIDCOMMAND;
#endif    
  writeMotors();
}  

#endif


