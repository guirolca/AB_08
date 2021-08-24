#ifndef WSSDAS_H
#define WSSDAS_H

#include <stdint.h>

#define TIRE_PERIM 1.6f
#define TEETH 30.0f
#define SEC_TO_MS 46666666.67f //TIRE_PERIM*1.000.000.000(x1.000.000 para pasar de us a s y x1.000 para + res.)/TEETH 

typedef struct 
{
	unsigned int speed;
	unsigned int last_speed;
	unsigned int first_value, second_value, third_value;
	unsigned char cnt, flag;
} WSS_t;

void Velocity_Calc(volatile WSS_t* Wss);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void ReadAllSpeeds(void);

#endif
