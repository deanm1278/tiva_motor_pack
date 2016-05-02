/*
 * libmotorpack.h
 *
 *  Created on: May 1, 2016
 *      Author: Dean Miller
 */

#ifndef LIBMOTORPACK_H_
#define LIBMOTORPACK_H_

#include "stepper.h"
#include "servo.h"

#define MP_SERVO1	0x01
#define MP_SERVO2	0x02

struct motor_pack{
	struct servo	servo1;
	struct servo	servo2;
	struct stepper 	step;
};

void motor_pack_init();

void motor_pack_init_stepper();
void motor_pack_set_stepper_div(uint32_t div);
void motor_pack_set_stepping_rate(uint32_t stepping_rate);

void motor_pack_stepper_start();
void motor_pack_stepper_stop();

void motor_pack_init_servo(uint32_t SERVO);
void motor_pack_set_servo_speed(uint32_t SERVO, float speed);

void motor_pack_servo_start(uint32_t SERVO);
void motor_pack_servo_stop(uint32_t SERVO);

void motor_pack_enable_motors();
void motor_pack_disable_motors();

#endif /* LIBMOTORPACK_H_ */
