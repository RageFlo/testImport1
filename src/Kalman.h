#ifndef _Kalman_h_
#define _Kalman_h_

#include "globals.h"

//struct kalmanData{
//    /* Kalman filter variables */
//    float kalman_Q_angle; // Process noise variance for the accelerometer
//    float kalman_Q_bias; // Process noise variance for the gyro bias
//    float kalman_R_measure; // Measurement noise variance - this is actually the variance of the measurement noise
//
//    float kalman_angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
//    float kalman_bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
//    float kalman_rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
//
//    float kalman_P[2][2]; // Error covariance matrix - This is a 2x2 matrix
//    };
//typedef struct kalmanData kalmanData;
    void initKalman(kalmanData* instance);
    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    float kalmanGetAngle(kalmanData* instance, float newAngle, float newRate, float dt);

    void kalmanSetAngle(kalmanData* instance, float angle); // Used to set angle, this should be set as the starting angle
    float kalmanGetRate(kalmanData* instance); // Return the unbiased rate

    /* These are used to tune the Kalman filter */
    void kalmanSetQangle(kalmanData* instance, float Q_angle);
    void kalmanSetQbias(kalmanData* instance, float Q_bias);
    void kalmanSetRmeasure(kalmanData* instance, float R_measure);

    float kalmanGetQangle(kalmanData* instance);
    float kalmanGetQbias(kalmanData* instance);
    float kalmanGetRmeasure(kalmanData* instance);



    void init2Kalman(kalman2Data* instance, float startAngle);
    float kalman2GetAngle(kalman2Data* instance, float newAngle, float newRate) ;




#endif
