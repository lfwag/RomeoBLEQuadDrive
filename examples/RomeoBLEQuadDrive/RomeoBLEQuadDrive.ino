/*!
* @file RemeoBLEQuadDrive.ino
* @brief RemeoBLEQuadDrive.ino PID control system of DC motor
*
*  RemeoBLEQuadDrive.ino Use PID control 4 way DC motor direction and speed
* 
* @author linfeng(490289303@qq.com)
* @version  V1.0
* @date  2016-4-14
*/
#include "PID_v1.h"
#include "Motor.h"

Motor motor[4];
int motorSpeed[4] = {-200,200,400,-400};/*Set 4 speed motor*/
/* Speed=motorSpeed/(32*(setSampleTime/1000))(r/s) */
const int motorDirPin[4][2] = { //Forward, Backward
/*Motor-driven IO ports*/
  {8,23},
  {7,9},	
  {24,14},
  {4,25}
};


//const double motorPidParam[3]={0.6,1,0.03};/*DC MOTOR,Yellow??180degree*/
//const double motorPidParam[3]={1.5,1,0.05};/*DC MOTOR,Yellow??90 degree*/
const double motorPidParam[3]={1.2,0.8,0.05};/*Encoder V1.0,160rd/min ;19500/min; 32:1,Kr=3.5*/
void setup( void )
{
  Serial1.begin(115200);
     for(int i=0;i<4;i++){
		motor[i].setPid(motorPidParam[0],motorPidParam[1],motorPidParam[2]);/*Tuning PID parameters*/
		motor[i].setPin(motorDirPin[i][0],motorDirPin[i][1]);/*Configure IO ports*/
		motor[i].setSampleTime(100);/*Sets the sampling period*/
                motor[i].setChannel(i);/*Sets the motor channel */
		motor[i].ready();/*Motor enable*/
                motor[i].setSpeed(motorSpeed[i]);/*Set motor speed*/
	}
}

void loop( void )
{
	for(int i = 0; i < 4; i++){
		motor[i].calibrate();/*motor PID calibrate*/
	}

}
/******************************************************************************
  Copyright (C) <2016>  <linfeng>
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  Contact: 490289303@qq.com
 ******************************************************************************/
 
