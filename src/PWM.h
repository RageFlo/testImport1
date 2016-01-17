/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PWM_H
#define PWM_H
#include "stm32f4xx_hal.h"

int pwm_init(void);
void pwm_set_pulsewidth(int newWidth, int channel);
void bldc_set_power(int newPower, int channel);

#endif
