/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GLOBALS_H
#define GLOBALS_H
#include "stm32f4xx_hal.h"
 struct pid_datastruct {
	int last_val;
	int int_val;
	int kp;
	int ki;
	int kd;
	int dt;
	int lim_int;
};

extern struct pid_datastruct *pidDataX;
extern struct pid_datastruct *pidDataY;
extern int pidY_X;
extern int pidY_Y;
extern int16_t acceltempgyroVals[7];
extern int16_t acceltempgyroValsFiltered[7];
extern int32_t timeDiffMPU;
extern int32_t angleGyro[3];
extern int32_t angleAccel[3];
extern int32_t angleComple[3];
extern int power[4];
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);
void HAL_IncTick(void);

uint32_t HAL_GetTick(void);
uint32_t HAL_GetTick10u(void);
void HAL_Delay(__IO uint32_t Delay);
void HAL_Delay10u(__IO uint32_t Delay);

#endif
