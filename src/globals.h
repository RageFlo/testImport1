/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GLOBALS_H
#define GLOBALS_H
#include "stm32f4xx_hal.h"

typedef struct kalmanData kalmanData;
struct kalmanData{
    /* Kalman filter variables */
    float kalman_Q_angle; // Process noise variance for the accelerometer
    float kalman_Q_bias; // Process noise variance for the gyro bias
    float kalman_R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float kalman_angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float kalman_bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float kalman_rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float kalman_P[2][2]; // Error covariance matrix - This is a 2x2 matrix
    };

typedef struct kalman2Data{
	float xStates[4];
}kalman2Data;

struct pid_datastruct {
	float last_val;
	float int_val;
	float kp;
	float ki;
	float kd;
	float dt;
	float lim_int;
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
extern float angleKalman[3];
extern int power[4];


#define MAXBURST 20000
extern char burst[MAXBURST];
extern uint8_t bursting_start;
extern uint8_t bursting_end;
extern int32_t currentburst;

extern kalmanData kalmanX;
extern kalmanData kalmanY;
extern kalman2Data kalman2X;
extern kalman2Data kalman2Y;

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);
void HAL_IncTick(void);

uint32_t HAL_GetTick(void);
uint32_t HAL_GetTick10u(void);
void HAL_Delay(__IO uint32_t Delay);
void HAL_Delay10u(__IO uint32_t Delay);

#endif
