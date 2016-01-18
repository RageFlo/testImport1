#include "PWM.h"
#include "globals.h"
#include "stm32f4xx_hal_tim.h"
#include "main.h"

static TIM_HandleTypeDef hnd_tim1;

int pwm_init(){
	TIM_OC_InitTypeDef cConfig;
	int success = -1;
	hnd_tim1.Instance = TIM1;
	hnd_tim1.Channel = HAL_TIM_ACTIVE_CHANNEL_1;
	hnd_tim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	hnd_tim1.Init.ClockDivision =  TIM_CLOCKDIVISION_DIV1;
	hnd_tim1.Init.Period = 2999;
	hnd_tim1.Init.Prescaler = 168;	
	hnd_tim1.Init.RepetitionCounter = 0;
	cConfig.OCMode =  TIM_OCMODE_PWM1;
	cConfig.Pulse = 1000;
	cConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	cConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
	cConfig.OCFastMode = TIM_OCFAST_ENABLE;
	HAL_TIM_PWM_Init(&hnd_tim1);
	HAL_TIM_PWM_ConfigChannel(&hnd_tim1,&cConfig, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&hnd_tim1,&cConfig, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&hnd_tim1,&cConfig, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&hnd_tim1,&cConfig, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start_IT(&hnd_tim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&hnd_tim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&hnd_tim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&hnd_tim1,TIM_CHANNEL_4);
	

	return success;
}

void pwm_set_pulsewidth(int newWidth, int channel){
	switch(channel){
		case(1): TIM1->CCR1 = newWidth; break;
		case(2): TIM1->CCR2 = newWidth; break;
		case(3): TIM1->CCR3 = newWidth; break;
		case(4): TIM1->CCR4 = newWidth; break;
	}
}

void bldc_set_power(int newPower, int channel){
	if(newPower > 999){
		newPower  = 999;
	}else if(newPower < 0){
		newPower  = 0;
	}
	pwm_set_pulsewidth(1000+newPower, channel);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *hnd){
	GPIO_InitTypeDef PWMPinsInit;
	  
	__TIM1_CLK_ENABLE();
	__GPIOE_CLK_ENABLE();

	PWMPinsInit.Mode = GPIO_MODE_AF_PP;
	PWMPinsInit.Pin = GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14 ;
	PWMPinsInit.Pull = GPIO_PULLDOWN;
	PWMPinsInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
	PWMPinsInit.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOE, &PWMPinsInit);
	
	HAL_NVIC_SetPriority(TIM1_CC_IRQn, 1, 1);
	
	//enable the interrupt for UPDATE EVENT
	HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
}

void TIM1_CC_IRQHandler(){
	HAL_TIM_IRQHandler(&hnd_tim1);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *hnd){
	callback_PWMOut();
}

