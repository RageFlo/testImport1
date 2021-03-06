#include "PID.h"

//void pid_init(struct pid_datastruct* to_int, float inKP, float inKI, float inKD, float inDT, float inLIM, float inLAST){
//	to_int->last_val = inLAST;
//	to_int->int_val = 0;
//	to_int->dt = inDT;
//	to_int->kp = inKP;
//	to_int->ki = inKI;
//	to_int->kd = inKD;
//	to_int->lim_int = inLIM;
//}
//
//int pi_run(struct pid_datastruct* pidData, float w, float x){
//	int e, y;
//	e = w - x;
//	pidData->int_val += e*pidData->dt;
//	pidData->last_val = e;
//	if(pidData->int_val > pidData->lim_int){
//		pidData->int_val = pidData->lim_int;
//	}else if(pidData->int_val < -pidData->lim_int){
//		pidData->int_val = -pidData->lim_int;
//	}
//	y = (pidData->kp * e)/PIDDOWNSCALE;
//	y += (pidData->ki * pidData->int_val)/PIDDOWNSCALE;
//	return y;
//}
//
//int pid_run_use_gyro(struct pid_datastruct* pidData, float w, float x, float deltax){
//	int y;
//	y = pi_run(pidData, w, x);
//	y += (pidData->kd * deltax)/PIDDOWNSCALE;
//	return y;
//}
//
//int pid_run(struct pid_datastruct* pidData, float w, float x){
//	int y;
//	y = pi_run(pidData, w, x);
//	y += (pidData->kd * ((w - x) - pidData->last_val) / pidData->dt)/PIDDOWNSCALE; // USE GYRO INFO INSTEAD???!
//	return y;
//}

void pid_init(struct pid_datastruct* to_int, float inKP, float inKI, float inKD, float inDT, float inLIM, float inLAST){
	to_int->last_val = inLAST;
	to_int->int_val = 0;
	to_int->dt = inDT;
	to_int->kp = inKP;
	to_int->ki = inKI;
	to_int->kd = inKD;
	to_int->lim_int = inLIM;
}

int pi_run(struct pid_datastruct* pidData, float w, float x){
	float e;
	int y;
	e = w - x;
	pidData->int_val += e*pidData->dt;
	pidData->last_val = e;
	if(pidData->int_val > pidData->lim_int){
		pidData->int_val = pidData->lim_int;
	}else if(pidData->int_val < -pidData->lim_int){
		pidData->int_val = -pidData->lim_int;
	}
	y = (pidData->kp * e);
	y += (pidData->ki * pidData->int_val);
	return y;
}

int pid_run_use_gyro(struct pid_datastruct* pidData, float w, float x, float deltax){
	int y;
	y = pi_run(pidData, w, x);
	y += (pidData->kd * deltax);
	return y;
}

int pid_run(struct pid_datastruct* pidData, float w, float x){
	int y;
	y = pi_run(pidData, w, x);
	y += (pidData->kd * ((w - x) - pidData->last_val) / pidData->dt); // USE GYRO INFO INSTEAD???!
	return y;
}


