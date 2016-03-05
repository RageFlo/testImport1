#include "Daten_Filter.h"
#include "globals.h"
#include "Kalman.h"

#define ACCLE_Z_OFFSET 3670
#define OFFSET_DEVIDER 10
static uint8_t gettingGyroOffset;
static int32_t gyroOffsetSamples = 0;
static int32_t gyroOffsets[3] = {0,0,0};
static int32_t gyroOffsetsReminder[3] = {0,0,0};



void filterMain(void){
	int i;
	static int period = 0;
	int32_t temp = 0;
	period++;
	
	temp = acceltempgyroVals[2] + ACCLE_Z_OFFSET; // Correct Z Axis! DYNMIC?? 
	if(temp > 32767)
				temp = 32767;
	else if(temp < -32767)
				temp = - 32767;
	acceltempgyroVals[2] = temp;
	
	for(i = 0; i < 4; i++){
		temp = acceltempgyroValsFiltered[i]*0 + acceltempgyroVals[i];
		acceltempgyroValsFiltered[i] = temp/1;
	}
	
	for(i = 4; i < 7; i++){
		temp = acceltempgyroVals[i] - gyroOffsets[i-4];
		if(temp > 32767)
			temp = 32767;
		else if(temp < -32767)
			temp = - 32767;
		acceltempgyroValsFiltered[i] = temp;
		
		if(gettingGyroOffset){
			gyroOffsets[i-4] = (acceltempgyroValsFiltered[i] + gyroOffsetsReminder[i-4]) / OFFSET_DEVIDER ;
			gyroOffsetsReminder[i-4] += acceltempgyroValsFiltered[i] % OFFSET_DEVIDER;
			gyroOffsetSamples++;
		}
		
		angleGyro[i-4] += acceltempgyroValsFiltered[i];
	}
  
	Get_angle_from_accle();
	//if(period%10)
	Get_angle_from_comple();	
	Get_angle_from_Kalman();


	if(bursting_start && !bursting_end){
		burst[currentburst++] =  (uint8_t)(acceltempgyroValsFiltered[4]>>8);
		burst[currentburst++] =  (uint8_t)(acceltempgyroValsFiltered[4]);
		burst[currentburst++] =  (uint8_t)(angleAccel[0]>>8);
		burst[currentburst++] =  (uint8_t)(angleAccel[0]);
		burst[currentburst++] =  1;
		if(currentburst==MAXBURST-10){
			bursting_start = 0;
			burst[currentburst] =  0;
			bursting_end = 1;
		}
	}
}

int Get_angle_from_accle(void){
	int state = 0;
	angleAccel[0] =  getFastXYAngle(acceltempgyroVals[2],acceltempgyroVals[1]);
	angleAccel[1] =  getFastXYAngle(acceltempgyroVals[2],acceltempgyroVals[0]);
	angleAccel[2] =  getFastXYAngle(acceltempgyroVals[0],acceltempgyroVals[1]);
	return state;
}

int Get_angle_from_comple(void){
	int state = 0;
	angleComple[0] = (angleAccel[0]*1*131 + (angleComple[0] - acceltempgyroValsFiltered[4]/100)*1999)/2000;
	angleComple[1] = (angleAccel[1]*1*131 + (angleComple[1] + acceltempgyroValsFiltered[5]/100)*1999)/2000;
	angleComple[2] = (angleAccel[2]*1*131 + (angleComple[2] - acceltempgyroValsFiltered[6]/100)*1999)/2000;
	return state;
}

int Get_angle_from_Kalman(void){
	int state = 0;
	//angleKalman[0] = kalmanGetAngle(&kalmanX,angleAccel[0],acceltempgyroValsFiltered[4]/(-131.f),0.001f);
	//angleKalman[1] = kalmanGetAngle(&kalmanY,angleAccel[1],acceltempgyroValsFiltered[5]/(131.f),0.001f);
	angleKalman[0] = kalman2GetAngle(&kalman2X,angleAccel[0],acceltempgyroValsFiltered[4]/(-13.1f));
	angleKalman[1] = kalman2GetAngle(&kalman2Y,angleAccel[1],acceltempgyroValsFiltered[5]/(13.1f));
	return state;
}

void Get_Gyro_Offset_Start(void){
	if(!gettingGyroOffset){
		gettingGyroOffset = 1;
		gyroOffsetSamples = 0;
	}
}
int Get_Gyro_Offset_Stopp(void){
	if(gettingGyroOffset){
		gettingGyroOffset = 0;
		return gyroOffsetSamples;
	}
	return 0;
}

unsigned int getFastXYAngle(int x, int y){
   // Fast XY vector to integer degree algorithm - Jan 2011 www.RomanBlack.com
   // Converts any XY values including 0 to a degree value that should be
   // within +/- 1 degree of the accurate value without needing
   // large slow trig functions like ArcTan() or ArcCos().
   // NOTE! at least one of the X or Y values must be non-zero!
   // This is the full version, for all 4 quadrants and will generate
   // the angle in integer degrees from 0-360.
   // Any values of X and Y are usable including negative values provided
   // they are between -1456 and 1456 so the 16bit multiply does not overflow.

   unsigned int negflag;
   unsigned int tempdegree;
   unsigned int comp;
   unsigned int degree;     // this will hold the result
   unsigned int ux;
   unsigned int uy;

   // Save the sign flags then remove signs and get XY as unsigned ints
   negflag = 0;
   if(x < 0)
   {
      negflag += 0x01;    // x flag bit
      x = (0 - x);        // is now +
   }
   ux = x;                // copy to unsigned var before multiply
   if(y < 0)
   {
      negflag += 0x02;    // y flag bit
      y = (0 - y);        // is now +
   }
   uy = y;                // copy to unsigned var before multiply

   // 1. Calc the scaled "degrees"
   if(ux > uy)
   {
      degree = (uy * 450) / ux;   // degree result will be 0-45 range
      negflag += 0x10;    // octant flag bit
   }
   else
   {
      degree = (ux * 450) / uy;   // degree result will be 0-45 range
   }

   // 2. Compensate for the 4 degree error curve
   comp = 0;
   tempdegree = degree;    // use an unsigned char for speed!
   if(tempdegree > 220)      // if top half of range
   {
      if(tempdegree <= 440) comp+=10;
      if(tempdegree <= 410) comp+=10;
      if(tempdegree <= 370) comp+=10;
      if(tempdegree <= 320) comp+=10;  // max is 4 degrees compensated
   }
   else    // else is lower half of range
   {
      if(tempdegree >= 20) comp+=10;
      if(tempdegree >= 60) comp+=10;
      if(tempdegree >= 100) comp+=10;
      if(tempdegree >= 150) comp+=10;  // max is 4 degrees compensated
   }
   degree += comp;   // degree is now accurate to +/- 1 degree!

   // Invert degree if it was X>Y octant, makes 0-45 into 90-45
   if(negflag & 0x10) degree = (900 - degree);

   // 3. Degree is now 0-90 range for this quadrant,
   // need to invert it for whichever quadrant it was in
   if(negflag & 0x02)   // if -Y
   {
      if(negflag & 0x01)   // if -Y -X
            degree = (1800 + degree);
      else        // else is -Y +X
            degree = (1800 - degree);
   }
   else    // else is +Y
   {
      if(negflag & 0x01)   // if +Y -X
            degree = (3600 - degree);
   }
	 //if((degree - 900) > 300)
	//	 degree = 1200 + (degree - 1200) /4;
	 //if((degree - 900) < 300)
	//	 degree = 600 + (degree - 600) / 4;
	 return degree;
 }
