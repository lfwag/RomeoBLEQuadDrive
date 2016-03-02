/*!
* @file Motor.h
* @brief Motor.h detailed description for Motor.cpp 
*
*  Motor.h STM32 header file that must be,4 quadrature encoder motor configurations and corresponding PWM soft function
* 
* @author linfeng(490289303@qq.com)
* @version  V1.0
* @date  2016-3-2
*/

#ifndef __ENCODER__H
#define __ENCODER__H


#define NULL 0
#include "PID_v1.h"
#include "stdio.h"
#include "stdlib.h"

#include "../../system/libstm/include/stm32f10x_rcc.h"
#include "../../system/libstm/include/stm32f10x_gpio.h"
#include "../../system/libstm/include/stm32f10x_tim.h"
#include "../../system/libstm/include/misc.h"
#include "../../system/libstm/include/stm32f10x.h"
extern void TIM5_IRQHandler(void);


#define MAX_SPEED 1000
#define MIN_SPEED (-MAX_SPEED)

typedef struct
{
	uint32_t RCC_APBPeriph_GPIO;    ///<GPIO clock
	uint32_t RCC_APBPeriph_TIM;     ///<timer clock
	uint32_t pin;                   ///<GPIO pin
	GPIO_TypeDef *GPIO;             ///<GPIO
	TIM_TypeDef *TIM;               ///<TIMER
}EncoderIO;

/*!
* @brief Motor Quadrature motor configuration function
*
* Motor  4 quadrature encoder motor configurations and corresponding PWM soft implementation
*
*/
class Motor
{
public:

	Motor() :distance(0),pid(0)
		{	}

	//~Motor() {if(pid) free(pid);}

	void setPid(double kp,double ki,double kd);
	void setPin(int forwardPin_, int backwardPin_) { forwardPin = forwardPin_; backwardPin = backwardPin_; }
	int setSpeed(int speed_);
	int setSampleTime(int sampleTime_) { sampleTime = sampleTime_; }
	
	double getSpeed(void){ return input; }
	
	int ready(void);
	void calibrate(void);
    void init();  
    unsigned short getCounter();
	int setChannel(unsigned char channel_) {channel = channel_;}
    void setCounter(unsigned short Count);
    void start_(void);
    void stop(void);
    
	void PWMInit(unsigned char digitalPin);
    void PWMStart();
    void PWMStop();
    void PWMSet(unsigned char dutSet); 

private:
    unsigned char channel;
	long distance;
	double input, output, setpoint;
	unsigned char forwardPin, backwardPin;
	int speed;
	int sampleTime;
	bool result;
	PID *pid;
};
#endif

/******************************************************************************
  Copyright (C) <2015>  <linfeng>
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
