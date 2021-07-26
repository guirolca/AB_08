#include "CAN.h"

/* USER CODE BEGIN Includes */
extern CanTxMsgTypeDef TxMessage;
extern CAN_HandleTypeDef hcan;

extern volatile uint16_t pulse_1;
extern volatile uint8_t AB_activation;
extern uint16_t speedRR_p, speedRL_p;
extern uint16_t speedFR_p, speedFL_p;
extern uint32_t	velNord_car, velEast_car, velDown_car;

/* USER CODE END Includes */

void addCan8Bit(uint8_t value, CanTxMsgTypeDef *pTx, uint8_t offset)
{
		pTx->Data[0+offset] = value;
}

void SendCanData(unsigned char DLC, unsigned short id, CanTxMsgTypeDef *pTx, CAN_HandleTypeDef* can)
{
	pTx->DLC=DLC;
	pTx->StdId=id;
	can->pTxMsg=pTx;
	HAL_CAN_Transmit_IT(can);
}

void send_CAN_data(void)
{
	addCan8Bit((AB_activation&0xFF),&TxMessage,0); 
	SendCanData(1,725,&TxMessage,&hcan);
}

unsigned long ReadCanData(CanRxMsgTypeDef* Msg, unsigned char offset, unsigned char len)
{
	uint32_t value=0;
	uint8_t i;
	for(i = 0; i < len; i++)
	{
		value += (((uint64_t)(Msg->Data[offset+i])&0xFF)<<(8*(len-1-i)));
	}
	return value;
}

void ReadCANData(CanRxMsgTypeDef* Msg)
{
	switch (Msg->StdId)
	{	
		case 140:
				speedFR_p=ReadCanData(Msg,0,2);
				speedFL_p=ReadCanData(Msg,2,2);
				speedRR_p=ReadCanData(Msg,4,2);
				speedRL_p=ReadCanData(Msg,6,2);
		break;	
		case 856:
				velEast_car=ReadCanData(Msg,0,4);
				velDown_car=ReadCanData(Msg,4,4);
		break;
		case 860:
				velNord_car=ReadCanData(Msg,0,4);
		break;
		default:
		break;
	}
}