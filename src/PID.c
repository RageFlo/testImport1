#include "PID.h"

void pid_init(struct pid_datastruct* to_int, int inKP, int inKI, int inKD, int inDT, int inLIM, int inLAST){
	to_int->last_val = inLAST;
	to_int->int_val = 0;
	to_int->dt = inDT;
	to_int->kp = inKP;
	to_int->ki = inKI;
	to_int->kd = inKD;
	to_int->lim_int = inLIM;
}

int pi_run(struct pid_datastruct* pidData, int w, int x){
	int e, y;
	e = w - x;
	pidData->int_val += e*pidData->dt;
	pidData->last_val = e;
	if(pidData->int_val > pidData->lim_int){
		pidData->int_val = pidData->lim_int;
	}else if(pidData->int_val < -pidData->lim_int){
		pidData->int_val = -pidData->lim_int;
	}
	y = (pidData->kp * e)/PIDDOWNSCALE;
	y += (pidData->ki * pidData->int_val)/PIDDOWNSCALE;
	return y;
}

int pid_run_use_gyro(struct pid_datastruct* pidData, int w, int x, int deltax){
	int y;
	y = pi_run(pidData, w, x);
	y += (pidData->kd * deltax)/PIDDOWNSCALE;
	return y;
}

int pid_run(struct pid_datastruct* pidData, int w, int x){
	int y;
	y = pi_run(pidData, w, x);
	y += (pidData->kd * ((w - x) - pidData->last_val) / pidData->dt)/PIDDOWNSCALE; // USE GYRO INFO INSTEAD???!
	return y;
}

