/*
 * pid.h
 *
 *  Created on: Jan 16, 2016
 *      Author: deanmiller
 */

#ifndef PID_H_
#define PID_H_

#include <stdint.h>

#define OUT_MAX 1500
#define OUT_MIN -1500

struct pid{
	float Kp;
	float Ki;
	float Kpos;
	float epsilon;

	int32_t integral;
#ifdef Kd
	int32_t pre_error;
#endif
};

void PIDInit(struct pid *pid);
int32_t PIDcal(float setpoint, float actual, uint32_t p_set, uint32_t p_actual, struct pid *pid);

#endif /* PID_H_ */
