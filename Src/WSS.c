#include "stm32f3xx_hal.h"
#include "WSS.h"

extern TIM_HandleTypeDef htim2;

extern WSS_t FR, FL;

 uint32_t FRBuffer[3],FLBuffer[3],RLBuffer[3],RRBuffer[3];	
 uint16_t FRCnt=0,FLCnt=0,RRCnt=0,RLCnt=0;


void Velocity_Calc(volatile WSS_t* Wss)
{
	if(Wss->second_value>Wss->first_value)
	{
		Wss->speed=(float)(SEC_TO_MS)/(float)(Wss->second_value-Wss->first_value);
	}
	else
	{
		Wss->speed=(float)(SEC_TO_MS)/(float)((Wss->first_value)-Wss->second_value);
	}
	if(Wss->first_value==0 || Wss->second_value==0) {Wss->speed=0;}
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2) //Se chequea si la interrupción ha sido generada por el TIM2
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)//Se chequea si la ha activado el canal 1
		{
			if(!FR.cnt)
			{
				FR.first_value=htim->Instance->CCR1; //Se guarda el valor del registro CCR1 
				FR.cnt=1;														 
			}
			else
			{
				FR.second_value=htim->Instance->CCR1; 
				FR.cnt=0;															
			}
		}
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{
			if(!FL.cnt)
			{
				FL.first_value=htim->Instance->CCR4;  //Se guarda el valor del registro CCR4
				FL.cnt=1;
			}
			else
			{
				FL.second_value=htim->Instance->CCR4;
				FL.cnt=0;
			}
		}
	}
}


void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}

void ReadAllSpeeds(void)
{
	static uint32_t FR_rep,FL_rep=0;
	Velocity_Calc(&FR);
	if(FR.last_speed==FR.speed)
	{
		FR_rep++;
		FR.speed=FR_rep>3?0:FR.speed;
	}
	else
	{
		FR.last_speed=FR.speed;
		FR_rep=0;
	}		
	Velocity_Calc(&FL);
	if(FL.last_speed==FL.speed)
	{
		FL_rep++;
		FL.speed=FL_rep>3?0:FL.speed;
	}
	else
	{
		FL.last_speed=FL.speed;
		FL_rep=0;
	}		
}