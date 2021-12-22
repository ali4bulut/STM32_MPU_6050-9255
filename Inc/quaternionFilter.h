/*
 * quaternionFilter.h
 *
 *  Created on: 23 Eki 2021
 *      Author: alibl
 */

#ifndef INC_QUATERNIONFILTER_H_
#define INC_QUATERNIONFILTER_H_
#include <math.h>
//----------------------------------------------------------------------------------------------------
// Variable declaration
#define PI M_PI
extern volatile float beta;				// algorithm gain
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
extern volatile float rpy[3];
extern volatile float lin_acc[3];

//---------------------------------------------------------------------------------------------------
// Function declarations
void update_rpy(float qw, float qx, float qy, float qz, float ax, float ay, float az);
void no_filter(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
#endif /* INC_QUATERNIONFILTER_H_ */
