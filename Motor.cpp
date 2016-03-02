/*!
* @file Motor.cpp
* @brief Motor.cpp Interrupted function deployment, encoder motor configurations and corresponding PWM soft function
*
*  Motor.cpp Corresponds to a set of orthogonal codes for each motor and PWM. 
*               Implements timer 5 implementation of PWM, timer every 3.9 μs (1/256 MS) into an interruption.
* 
* @author linfeng(490289303@qq.com)
* @version  V1.0
* @date  2016-3-2
*/
#include "stdlib.h"
#include "Arduino.h"
#include "Motor.h"
unsigned char dut[4]; ///< PWM value
unsigned char conter; ///< Counter every 1ms up plus 1, range 0-255
unsigned char PWMFlag;///< PWM logo, used to determine which one you should use IO

EncoderIO encoderIO[4]=
{
  {RCC_APB2Periph_GPIOA,RCC_APB1Periph_TIM2,GPIO_Pin_0 | GPIO_Pin_1,GPIOA,TIM2},
  {RCC_APB2Periph_GPIOA,RCC_APB1Periph_TIM3,GPIO_Pin_6 | GPIO_Pin_7,GPIOB,TIM3},
  {RCC_APB2Periph_GPIOB,RCC_APB1Periph_TIM4,GPIO_Pin_6 | GPIO_Pin_7,GPIOB,TIM4},
  {RCC_APB2Periph_GPIOC,RCC_APB2Periph_TIM8,GPIO_Pin_6 | GPIO_Pin_7,GPIOC,TIM8}
};



/*!
* @brief Break function for entrance
*
* @brief  3.9 μs into break, achieving up to 4 duty for 0-255 PWM
*
* @return void
*/ 
void TIM5_IRQHandler(void)
{							
  TIM5->SR = ~0x0001;
  conter++;
    
  if(PWMFlag & 0x01){
    if(!(PWMFlag & 0x10)){
      GPIO_ResetBits(GPIOC, GPIO_Pin_11);
    }else{
      if(conter<dut[0]){
        GPIO_SetBits(GPIOC, GPIO_Pin_11);
      }else if(conter!=255){
        GPIO_ResetBits(GPIOC, GPIO_Pin_11);
      }
    }
   }else{
      if(!(PWMFlag & 0x10)){
        GPIO_ResetBits(GPIOC, GPIO_Pin_12);
      }else{
        if(conter<dut[0]){
          GPIO_SetBits(GPIOC, GPIO_Pin_12);
        }else if(conter!=255){
          GPIO_ResetBits(GPIOC, GPIO_Pin_12);
        }
      }
    }        
        
  if(PWMFlag & 0x02){
    if(!(PWMFlag & 0x20)){
      GPIO_ResetBits(GPIOA, GPIO_Pin_11);
    }else{
      if(conter<dut[1]){
        GPIO_SetBits(GPIOA, GPIO_Pin_11);
      }else if(conter!=255){
        GPIO_ResetBits(GPIOA, GPIO_Pin_11);
      }
    }
  }else{
      if(!(PWMFlag & 0x20)){
        GPIO_ResetBits(GPIOC, GPIO_Pin_10);
      }else{
        if(conter<dut[1]){
          GPIO_SetBits(GPIOC, GPIO_Pin_10);
        }else if(conter!=255){
			GPIO_ResetBits(GPIOC, GPIO_Pin_10);
        }
      }
    }
        
    if(PWMFlag & 0x04){
      if(!(PWMFlag & 0x40)){
        GPIO_ResetBits(GPIOB,GPIO_Pin_9);
      }else{
        if(conter<dut[2]){
          GPIO_SetBits(GPIOB,GPIO_Pin_9);
        }else if(conter!=255){
          GPIO_ResetBits(GPIOB,GPIO_Pin_9);
        }
      }  
    }else{
      if(!(PWMFlag & 0x40)){
        GPIO_ResetBits(GPIOB, GPIO_Pin_8);
      }else{
        if(conter<dut[2]){
            GPIO_SetBits(GPIOB, GPIO_Pin_8);
        }else if(conter!=255){
            GPIO_ResetBits(GPIOB, GPIO_Pin_8);
        }
      }
    }
        
    if(PWMFlag & 0x08)
    {
      if(!(PWMFlag & 0x80)){
        GPIO_ResetBits(GPIOB,GPIO_Pin_5);
      }else{
        if(conter<dut[3]){
          GPIO_SetBits(GPIOB,GPIO_Pin_5);
        }else if(conter!=255){
          GPIO_ResetBits(GPIOB,GPIO_Pin_5);
        }
      }
    }else{
      if(!(PWMFlag & 0x80)){
        GPIO_ResetBits(GPIOD,GPIO_Pin_2);
      }else{
        if(conter<dut[3]){
          GPIO_SetBits(GPIOD,GPIO_Pin_2);
        }else if(conter!=255){
          GPIO_ResetBits(GPIOD,GPIO_Pin_2);
        }
      }
    }        
  
}

/*!
* @brief Suspension alignment function
*
* @brief  Break function for entrance
*
* @return void
*/ 
void TIM_Configuration(void)
{ 
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		

  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;			   
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
  NVIC_Init(&NVIC_InitStructure);								

  TIM_TimeBaseStructure.TIM_Period = 9;          		
  TIM_TimeBaseStructure.TIM_Prescaler = 27;       		
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    		 
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0X0;		
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);		
   
  TIM_ClearFlag(TIM5, TIM_FLAG_Update);					
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);	
  TIM_Cmd(TIM5, ENABLE);
}
/*!
* @brief Setting coefficient
*
* @brief Tuning PID Kp, Ki and Kd coefficient
*
* @return void
*/ 
void Motor::setPid(double kp,double ki,double kd)
{
	if(pid == 0){
		pid = (PID *)malloc(sizeof(PID));
		pid->PID_(&input, &output, &setpoint, kp, ki, kd, DIRECT);
		//pid = new PID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
	}else{
		//delete pid;
		//pid = 0;
	}
}
/*!
* @brief motor enable
*
* @brief motor enable
*
* @return int
*/ 
int Motor::ready(void)
{
	if(pid == NULL)
		return -1;
	pid->SetMode(AUTOMATIC);
	pid->SetSampleTime(sampleTime);
	init();
	setCounter(30000);	
	TIM_ClearFlag(encoderIO[channel].TIM,1);
	TIM_Cmd(encoderIO[channel].TIM,ENABLE);	
	return 0;
}
/*!
* @brief set Speed
*
* @brief set Speed
*
* @return int
*/ 
int Motor::setSpeed(int speed_)
{
	if(speed_ > MAX_SPEED){
		speed = MAX_SPEED;
	}else if(speed_ < MIN_SPEED){
		speed = MIN_SPEED;
	}else{
		speed = speed_;
	}
	start_();
	return speed;
}

/*!
* @brief calibrate
*
* @brief Calibration intervals PWM output
*e
* @return void
*/
void Motor::calibrate(void)
{
	
	int wheelSpeed;
	wheelSpeed = getCounter()-30000;
	input = (double)abs(wheelSpeed);
	result=pid->Compute();  
    if(result){
		setCounter(30000);		
		distance += wheelSpeed; 		
		PWMSet(output); 		
		Serial1.println(wheelSpeed);
	}
}

/*!
* @brief Motor quadrature encoder initialization function
*
* @brief  4 timers configured for quadrature encoder mode
*
* @return void
*/ 
void Motor::init(){
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;        

        
	if(encoderIO[channel].TIM == TIM4){
		GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE); 
	}
        
	if(encoderIO[channel].TIM == TIM8){
		RCC_APB2PeriphClockCmd(encoderIO[channel].RCC_APBPeriph_TIM,ENABLE);
	}else{
		RCC_APB1PeriphClockCmd(encoderIO[channel].RCC_APBPeriph_TIM,ENABLE);        
	}
	RCC_APB2PeriphClockCmd(encoderIO[channel].RCC_APBPeriph_GPIO,ENABLE);
	GPIO_InitStructure.GPIO_Pin = encoderIO[channel].pin; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(encoderIO[channel].GPIO,&GPIO_InitStructure); 

	TIM_TimeBaseStructure.TIM_Prescaler = 0x00; 
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x00;
	TIM_TimeBaseStructure.TIM_CounterMode = 0x00;
	TIM_TimeBaseInit(encoderIO[channel].TIM, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(encoderIO[channel].TIM,3,0,0); 
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(encoderIO[channel].TIM, &TIM_ICInitStructure);
}

/*!
* @brief The motor encoder value
*
* @brief  The motor encoder value
*
* @return The motor encoder value
*/ 
unsigned short Motor::getCounter()
{
  return TIM_GetCounter(encoderIO[channel].TIM);
}


/*!
* @brief Set motor encoder initialization values
*
* @brief  Set motor encoder initialization values
*
* @return void
*/ 
void Motor::setCounter(unsigned short Count)
{
  encoderIO[channel].TIM->CNT = Count;
}

/*!
* @brief Motor encoder
*
* @brief  Motor encoder
*
* @return void
*/ 

void Motor::start_()
{
	if(speed > 0){
      PWMInit(backwardPin);
      pinMode(forwardPin, OUTPUT);
      digitalWrite(forwardPin,LOW);
	}else{
      PWMInit(forwardPin);
      pinMode(backwardPin,OUTPUT);
      digitalWrite(backwardPin,LOW);
	}
	
	setpoint=abs(speed);
	PWMStart();
	PWMSet(100);
}

/*!
* @brief Disabling motor encoders
*
* @brief  Disabling motor encoders
*
* @return void
*/ 
void Motor::stop()
{
  TIM_ClearFlag(encoderIO[channel].TIM,1);
  TIM_Cmd(encoderIO[channel].TIM,DISABLE);
}

/*!
* @brief Class motor PWM
*
* @brief  IO port that you want to configure the drive motor. Motor 1 using IO port 8/23; 
* @brief  motor 2 using IO port 7/9; motor 3 using IO port 24/14 motor 4 use of IO 4/25.
*
* @return void
*/ 

void Motor::PWMInit(unsigned char digitalPin) 
{
  TIM_Configuration();
  pinMode(digitalPin,OUTPUT);
  
  if(digitalPin==8){
    PWMFlag |= 0x01;
  }else if(digitalPin==23){
    PWMFlag &= 0xfe;
  }else if(digitalPin==7){
    PWMFlag |= 0x02;
  }else if(digitalPin==9){
    PWMFlag &= 0xfd;
  }else if(digitalPin==24){
    PWMFlag |= 0x04;
  }else if(digitalPin==14){
    PWMFlag &= 0xfb;
  }else if(digitalPin==4){
    PWMFlag |= 0x08;
  }else if(digitalPin==25){
    PWMFlag &= 0xf7;
  }
}

/*!
* @brief Enabled motor PWM
*
* @brief  Enabled motor PWM
*
* @return void
*/ 
void Motor::PWMStart()
{
  //PWMFlag |= motorIO[channel].pwmflag2;
  PWMFlag |= (0x10<<channel);
}

/*!
* @brief Disabling motor PWM
*
* @brief  Disabling motor PWM
*
* @return void
*/ 
void Motor::PWMStop()
{
  //PWMFlag &= ~(motorIO[channel].pwmflag2);
  PWMFlag &= ~(0x10<<channel);
}
/*!
* @brief Set the PWM duty cycle
*
* @brief  Set the PWM duty cycle
*
* @return void
*/ 
void Motor::PWMSet(unsigned char Dut_Set)
{
  dut[channel]=Dut_Set;
}


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
