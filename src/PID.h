/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PID_H
#define PID_H
#include "stm32f4xx_hal.h"
#include "globals.h"

#define PIDDOWNSCALE 1000

void pid_init(struct pid_datastruct *to_int, float inKP, float inKI, float inKD, float inDT, float inLIM, float inLAST);
int pi_run(struct pid_datastruct* pidData, float w, float x);
int pid_run_use_gyro(struct pid_datastruct* pidData, float w, float x, float deltax);
int pid_run(struct pid_datastruct* pidData, float w, float x);

#endif
