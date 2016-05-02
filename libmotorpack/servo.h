/*
 * servo.h
 *
 *  Created on: Jan 23, 2016
 *      Author: deanmiller
 */
#include <stdint.h>
#include <stdbool.h>

#include "pid.h"

#ifndef SERVO_H_
#define SERVO_H_

#define PWM_PERIOD 3200
#define NUM_SPEED_SAMPLES 10
#define VEL_SAMPLE_PERIOD 10000

struct servo{
	uint32_t 	pos;
	uint32_t 	target_pos;
	float	 	speed;
	float	 	target_speed;
	int32_t 	dir;
	int32_t	 	target_dir;
	int32_t 	dc;
	uint32_t 	PWM_BASE;
	uint32_t 	PWM_OUT;
	uint32_t 	PWM_PERIOD_HIGH;
	uint32_t 	QEI_BASE;
	struct pid		pid;
	uint32_t	buf[NUM_SPEED_SAMPLES];
	uint32_t	head;
};

void servo_init_pwm(struct servo *servo, uint32_t PWM_OUT, uint32_t PERIOD_HIGH);
void servo_read(struct servo *servo);
void servo_loop(struct servo *servo);
void servo_init_qei(struct servo *servo, uint32_t QEI_BASE);
void servo_stop(struct servo *servo);
void servo_start(struct servo *servo);

#endif /* SERVO_H_ */
