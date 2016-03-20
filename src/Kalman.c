#include "Kalman.h"

 void initKalman(kalmanData* instance) {
    /* We will set the variables like so, these can also be tuned by the user */
    instance->kalman_Q_angle = 0.001f;
    instance->kalman_Q_bias = 0.003f;
    instance->kalman_R_measure = 0.03f;

    instance->kalman_angle = 0.0f; // Reset the angle
    instance->kalman_bias = 0.0f; // Reset bias

    instance->kalman_P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    instance->kalman_P[0][1] = 0.0f;
    instance->kalman_P[1][0] = 0.0f;
    instance->kalman_P[1][1] = 0.0f;
};

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float kalmanGetAngle(kalmanData* instance, float newAngle, float newRate, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    instance->kalman_rate = newRate - instance->kalman_bias;
    instance->kalman_angle += dt * instance->kalman_rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    instance->kalman_P[0][0] += dt * (dt*instance->kalman_P[1][1] - instance->kalman_P[0][1] - instance->kalman_P[1][0] + instance->kalman_Q_angle);
    instance-> kalman_P[0][1] -= dt * instance->kalman_P[1][1];
    instance->kalman_P[1][0] -= dt * instance->kalman_P[1][1];
    instance->kalman_P[1][1] += instance->kalman_Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = instance->kalman_P[0][0] + instance->kalman_R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = instance->kalman_P[0][0] / S;
    K[1] = instance->kalman_P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - instance->kalman_angle; // Angle difference
    /* Step 6 */
    instance->kalman_angle += K[0] * y;
    instance->kalman_bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = instance->kalman_P[0][0];
    float P01_temp = instance->kalman_P[0][1];

    instance->kalman_P[0][0] -= K[0] * P00_temp;
    instance->kalman_P[0][1] -= K[0] * P01_temp;
    instance->kalman_P[1][0] -= K[1] * P00_temp;
    instance->kalman_P[1][1] -= K[1] * P01_temp;

    return instance->kalman_angle;
};

void kalmanSetAngle(kalmanData* instance, float angle) { instance->kalman_angle = angle; }; // Used to set angle, this should be set as the starting angle
float kalmanGetRate(kalmanData* instance) { return instance->kalman_rate; }; // Return the unbiased rate

/* These are used to tune the Kalman filter */
void kalmanSetQangle(kalmanData* instance, float Q_angle) { instance->kalman_Q_angle = Q_angle; };
void kalmanSetQbias(kalmanData* instance, float Q_bias) { instance->kalman_Q_bias = Q_bias; };
void kalmanSetRmeasure(kalmanData* instance, float R_measure) { instance->kalman_R_measure = R_measure; };

float getQangle(kalmanData* instance) { return instance->kalman_Q_angle; };
float getQbias(kalmanData* instance) { return instance->kalman_Q_bias; };
float getRmeasure(kalmanData* instance) { return instance->kalman_R_measure; };

/*
 * QUAD KALMAN FILTER
 *
 * FLORIAN HÖFER
 * 03.2016
 */


#define KMAX_ANGLE_DELTA 10.0f
#define KMAX_X4 50.0f
#define KGAIN_1_1 (0.004305f)
#define KGAIN_1_2 (0.008511f)
#define KGAIN_1_3 (0.007854f)
#define KGAIN_1_4 (-0.006217f)

#define KGAIN_2_1 (0.000918f)
#define KGAIN_2_2 (0.013826f)
#define KGAIN_2_3 (0.099184f)
#define KGAIN_2_4 (0.000133f)

void init2Kalman(kalman2Data* instance, float startAngle) {
	instance->xStates[0] = startAngle;
};

float kalman2GetAngle(kalman2Data* instance, float newAngle, float newRate) {
	float *x = instance->xStates;
	x[0] += x[1]/1000.0f + x[2]/(2.0f*1000.0f*1000.0f);
	x[1] += x[2]/1000.0f;
	float diffAngle = newAngle - x[0];
	float diffRate = newRate - x[1] - x[3];
	if(diffAngle > KMAX_ANGLE_DELTA){
		diffAngle = KMAX_ANGLE_DELTA;
	}else if(diffAngle < -KMAX_ANGLE_DELTA){
		diffAngle = -KMAX_ANGLE_DELTA;
	}

	x[0] += KGAIN_1_1 * diffAngle + KGAIN_2_1 * diffRate;
	x[1] += KGAIN_1_2 * diffAngle + KGAIN_2_2 * diffRate;
	x[2] += KGAIN_1_3 * diffAngle + KGAIN_2_3 * diffRate;
	x[3] += KGAIN_1_4 * diffAngle + KGAIN_2_4 * diffRate;

	if(x[3] > KMAX_X4){
		x[3] = KMAX_X4;
	}else if(x[3] < -KMAX_X4){
		x[3] = -KMAX_X4;
	}
	return instance->xStates[0];
}
