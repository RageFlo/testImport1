/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DATEN_FILTER_H
#define DATEN_FILTER_H
#include "stm32f4xx_hal.h"

void filterMain(void);

void Get_Gyro_Offset_Start(void);
int Get_Gyro_Offset_Stopp(void);

unsigned int getFastXYAngle(int x, int y);
int Get_angle_from_accle(void);
#endif
