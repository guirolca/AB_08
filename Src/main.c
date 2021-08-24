/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "CAN.h"
#include "WSS.h"
#include "math.h"

/* USER CODE BEGIN Includes */
volatile uint8_t AB_ON=0;
volatile uint8_t AB_activation=0;
volatile uint8_t AB_transmit=0;
volatile uint8_t AB_update=0;
volatile uint8_t WSS_read=0;
volatile float speed=0;
volatile float speed1=0;
volatile float speed2=0;
volatile float speed3=0;
	
volatile uint16_t CAN_TRANSMIT_FREQ = 20;
volatile uint16_t CAN_RECEIVE_FREQ = 100;
volatile uint16_t AB_UPDATE_FREQ = 100;
volatile uint16_t WSS_UPDATE_FREQ = 100;

volatile uint16_t speedRR_p, speedRL_p;
volatile uint16_t speedFR_p, speedFL_p;
volatile uint32_t	velNord_car=0, velEast_car=0, velDown_car=0;
volatile WSS_t FR, FL, RL, RR;

/*Value of the pulses generated. Needs to be checked*/
volatile uint16_t pulse_1=1000, pulse_2=2000;

/*Needed to control the button bounce*/
CanRxMsgTypeDef RxMessage;
CanTxMsgTypeDef TxMessage; 

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Update_PWM(void);                                
void AB_test(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
	HAL_CAN_Receive_IT(&hcan,CAN_FIFO0);
  MX_TIM3_Init();
	MX_TIM2_Init();
	
  /* USER CODE BEGIN 2 */
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if(AB_transmit==1)
	{
		send_CAN_data();
		AB_transmit=FLAG_RESET;
	}
	if(WSS_read==1)
	{
		HAL_NVIC_DisableIRQ(TIM2_IRQn);
		ReadAllSpeeds();
		HAL_NVIC_EnableIRQ(TIM2_IRQn);
		WSS_read=0;
		}	
	if(AB_activation)
	{
		AB_test();
		AB_activation=FLAG_RESET;
	}
	if(AB_update)
	{
		Update_PWM();
		AB_update=FLAG_RESET;
	}
  /* USER CODE END 3 */
  }
}

/*Configuration for the value for each tick*/
void ConfigureSysTick(void)
{
	if(SysTick_Config(SystemCoreClock / 1000))
		while(1);
}

void Update_PWM()
{
	speed1=FR.speed+FL.speed/2;
	speed2=speedFR_p+speedFL_p/2;
	speed3=pow(pow(velEast_car,2)+pow(velDown_car,2)+pow(velNord_car,2),0.5f);
	
	if (speed3>0){
		speed=speed3;}
	if (speed2>0){
		speed=speed2;}
	if (speed1>0){
		speed=speed1;}
	
	if (speed>12000&speed<20000){
		pulse_1=9610+(speed-12000)/3.125f;
		pulse_2=12172-(speed-12000)/3.125f;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_1);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse_2);
		AB_ON=1;
		HAL_Delay(1000);
	}
}

void AB_test()
{
	for(int i=0; i<2; i++){
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 9610);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 12172);
	HAL_Delay(500);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 10110);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 11110);
	HAL_Delay(500);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 10610);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 10610);
	HAL_Delay(500);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 11110);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 10110);
	HAL_Delay(500);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 12170);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 9610);
	HAL_Delay(500);
}
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN;
	hcan.pRxMsg=&RxMessage;
	hcan.pTxMsg=&TxMessage;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_8TQ;
  hcan.Init.BS2 = CAN_BS2_3TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 5-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 28829-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 9610; //9610 a 19219 (1 a 2ms->120º) En este caso 0º
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  sConfigOC.Pulse = 12172;  //En este caso 60-28º=32º con respecto al primero
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

	HAL_TIM_Base_Start(&htim3); //Starts the TIM Base generation. Si todo esta bien lo iniciamos
	
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)//Starts the PWM signal generation
  {
    /* PWM Generation Error */
    Error_Handler();
  }
	if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK)//Starts the PWM signal generation
  {
    /* PWM Generation Error */
    Error_Handler();
  }
}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */


void HAL_SYSTICK_Callback(void)
{
	uint32_t uwTick = HAL_GetTick();
	
	if(!AB_transmit && !(uwTick % (1000/CAN_TRANSMIT_FREQ)))
		AB_transmit = FLAG_SET;
	if(!AB_update && !(uwTick % (1000/AB_UPDATE_FREQ)))
		AB_update = FLAG_SET;
	if(!WSS_read && !(uwTick % (1000/WSS_UPDATE_FREQ)))
		WSS_read = FLAG_SET;	
	if(!(uwTick % (1000/CAN_TRANSMIT_FREQ)))	
		HAL_CAN_Receive_IT(&hcan,CAN_FIFO0);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
