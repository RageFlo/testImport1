/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PID_H
#define PID_H
#include "stm32f4xx_hal.h"
#include "globals.h"

#define PIDDOWNSCALE 1000

void pid_init(struct pid_datastruct *to_int, int inKP, int inKI, int inKD, int inDT, int inLIM, int inLAST);
int pi_run(struct pid_datastruct* pidData, int w, int x);
int pid_run_use_gyro(struct pid_datastruct* pidData, int w, int x, int deltax);
int pid_run(struct pid_datastruct* pidData, int w, int x);

#endif
