#include "CAN.h"

/* USER CODE BEGIN Includes */
extern CanTxMsgTypeDef TxMessage;
extern CanRxMsgTypeDef RxMessage;
extern CAN_HandleTypeDef hcan;

extern volatile uint8_t AB_ON;
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
	addCan8Bit((AB_ON&0xFF),&TxMessage,0); 
	SendCanData(1,725,&TxMessage,&hcan);
}

unsigned long ReadCanData(CanRxMsgTypeDef* RxMessage, unsigned char offset, unsigned char len)
{
	uint32_t value=0;
	uint8_t i;
	for(i = 0; i < len; i++)
	{
		value += (((uint64_t)(RxMessage->Data[offset+i])&0xFF)<<(8*(len-1-i)));
	}
	return value;
}

void ReadCANData(CanRxMsgTypeDef* RxMessage)
{
	switch (RxMessage->StdId)
	{	
		case 140:
				speedFR_p=ReadCanData(RxMessage,0,2);
				speedFL_p=ReadCanData(RxMessage,2,2);
		break;
		case 510:
				AB_activation=ReadCanData(RxMessage,0,1);
		break;			
		case 856:
				velEast_car=ReadCanData(RxMessage,0,4);
				velDown_car=ReadCanData(RxMessage,4,4);
		break;
		case 860:
				velNord_car=ReadCanData(RxMessage,0,4);
		break;	
		default:
		break;
	}
}