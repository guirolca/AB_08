#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"
#include "main.h"

void addCan8Bit(uint8_t value, CanTxMsgTypeDef *pTx, uint8_t offset);
void SendCanData(unsigned char DLC, unsigned short id, CanTxMsgTypeDef *pTx, CAN_HandleTypeDef* can);
void send_CAN_data(void);
void ReadCANData(CanRxMsgTypeDef* RxMessage);
unsigned long ReadCanData(CanRxMsgTypeDef* Msg, unsigned char offset, unsigned char len);
