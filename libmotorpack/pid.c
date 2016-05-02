/*
 * pid.c
 *
 *  Created on: May 1, 2016
 *      Author: Dean Miller
 */
#include <stdint.h>
#include "pid.h"

void PIDInit(struct pid *pid){
	pid->Kp	=	2.0;
	pid->Ki	=	0.003;
	pid->Kpos	=	0;
	pid->epsilon =	0.3;
}

int32_t PIDcal(float setpoint, float actual, uint32_t p_set, uint32_t p_actual, struct pid *pid)
{

	int32_t output;
#ifdef Kd
	int32_t derivative;
#endif

	//calculate outer loop
	int32_t p_error = p_set - p_actual;

	//CaculateP,I,D
	float error = p_error * pid->Kpos + setpoint - actual;

	//In case of error too small then stop integration
	if(abs(error) > pid->epsilon){
		pid->integral = pid->integral + error;
	}
	else{
		pid->integral = 0;
	}

#ifdef Kd
	derivative= (error - pid->pre_error) / VEL_SAMPLE_PERIOD;
	output = pid->Kp*error+ pid->Ki*pid->integral+ pid->Kd*derivative;
#else
	output = pid->Kp*error+ pid->Ki*pid->integral;
#endif

	//Saturation Filter
	if(output> OUT_MAX)
	{
		output= OUT_MAX;
	}
	else if(output < OUT_MIN){
		output = OUT_MIN;
	}
#ifdef Kd
	//Update error
	pid->pre_error = error;
#endif

	return output;
}
