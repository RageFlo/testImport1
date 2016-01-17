/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PID_H
#define PID_H
#include "stm32f4xx_hal.h"
#include "globals.h"

#define PIDDOWNSCALE 1000

void pid_init(struct pid_datastruct *to_int, int inKP, int inKI, int inKD, int inDT, int inLIM, int inLAST);
int pid_run(struct pid_datastruct* pidData, int x, int w);

#endif
