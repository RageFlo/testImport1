/**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   Main program body (with modifications by ARM)
  ******************************************************************************

*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "globals.h"
#include "stdout_USART.h"
#include "I2C_Treiber.h"
#include "Daten_Filter.h"
#include "PWM.h"
#include "PID.h"
#include <stdio.h>

#ifdef _RTE_
#include "RTE_Components.h"             // Component selection
#endif


/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint32_t delayTime;
static struct pid_datastruct pidDataXObj;
static struct pid_datastruct pidDataYObj;
/* Private function prototypes -----------------------------------------------*/
//static void SystemClock_Config(void);
static void Error_Handler(void);
static void main_init(void);
static void main_loop(void);

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

   main_init();

	/* Infinite loop */
	while (1)
	{
		while(HAL_GetTick10u()-delayTime<100){}
		delayTime = HAL_GetTick10u();
		main_loop();
	}
}


static void main_init(void){	
	GPIO_InitTypeDef ledGPIOInit;
	SystemInit();
	HAL_Init();	
	
	/* Configure the system clock to 168 MHz */
	//SystemClock_Config();
	std_init();

	__GPIOD_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();
	
	ledGPIOInit.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	ledGPIOInit.Mode = GPIO_MODE_OUTPUT_PP;
	ledGPIOInit.Pull = GPIO_NOPULL;
	ledGPIOInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOD,&ledGPIOInit);
	
	ledGPIOInit.Pin = GPIO_PIN_0;
	ledGPIOInit.Mode = GPIO_MODE_INPUT;
	ledGPIOInit.Pull = GPIO_PULLDOWN;
	ledGPIOInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA,&ledGPIOInit);
	HAL_Delay(10);
	
	pwm_init();
	
	if(initMPU()){
		puts("Failed MPU");
	}
	
	Get_Gyro_Offset_Start();
	HAL_Delay(1000);
	if(Get_Gyro_Offset_Stopp()<900){
		puts("Failed Gyrooffset");
		Error_Handler();
	}
	delayTime = HAL_GetTick();
	
	pidDataX = &pidDataXObj;
	pidDataY = &pidDataYObj;
	pid_init(pidDataX,1000,0,1000,1,9000,900);
	pid_init(pidDataY,1000,0,1000,1,9000,900);
	puts("Init + Selfcheck okay!");
}
	
static void main_loop(void){
	static uint32_t loopCounter = 0;
	static uint32_t divider100ms = 100;
	
	if(!(loopCounter%divider100ms)){
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);
		kommuHandler();
	}
	
	if(!(loopCounter)){

	}

//	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_SET){
//		bldc_set_power(1000,1);
//	}else{
//		bldc_set_power(power[0],1);
//	}

	loopCounter++;
}


void callback_MPU6050(void){
	static uint32_t last = 0;
	uint32_t current = HAL_GetTick10u();
	timeDiffMPU = current - last;
	last = current;

	MPU6050_GetRawAccelGyro(acceltempgyroVals);	// GET ACCLEx3 TEMP GYROx3

	filterMain();	// FILTER MPU DATA

	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
	HAL_NVIC_ClearPendingIRQ(EXTI3_IRQn);
}

void callback_PWMOut(void){
	int i;
	pidY_X = pid_run(pidDataX,angleComple[0] / 131, 900);
	pidY_Y = pid_run(pidDataY,angleComple[1] / 131, 900);
	for( i = 0; i < 4; i++){
		bldc_set_power(power[i],i+1);
	}
}












/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
