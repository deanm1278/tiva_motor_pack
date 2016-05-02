/*
 * libmotorpack.c
 *
 *  Created on: May 1, 2016
 *      Author: Dean Miller
 */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/qei.h"
#include "driverlib/pwm.h"

#include "libmotorpack.h"
#include "stepper.h"
#include "servo.h"
#include "microstep_values.h"

static struct motor_pack mp;

//Define the QEI timer interrupts
void QEI0IntHandler(void){
	QEIIntClear(QEI0_BASE, QEI_INTTIMER);
	servo_read(&mp.servo1);

	servo_loop(&mp.servo1);
}

void QEI1IntHandler(void){
	QEIIntClear(QEI1_BASE, QEI_INTTIMER);
	servo_read(&mp.servo2);

	servo_loop(&mp.servo2);
}

//stepper interrupt handler
void
TIMER2IntHandler(void){
	//
	// Clear the timer interrupt.
	//
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	uint32_t val = microstep_values[mp.step.t_val];

	if(val & MS_APH)	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
	else	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);

	if(val & MS_BPH)	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
	else	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x00);

	DAC082S085_set_value(&mp.step.dac, DAC082S085_DAC0, (val & MS_AVREF) >> 8);
	DAC082S085_set_value(&mp.step.dac, DAC082S085_DAC1, (val & MS_BVREF));

	mp.step.t_val += mp.step.div;
	mp.step.fires_this_move++;
	if(mp.step.t_val > 1023) mp.step.t_val = 0;
}

void motor_pack_init(){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2); //enable / disable for motors
	servo_init_pwm(&mp.servo1, PWM_OUT_0, PWM_PERIOD);
	servo_stop(&mp.servo1);

	servo_init_pwm(&mp.servo2, PWM_OUT_4, PWM_PERIOD);
	servo_stop(&mp.servo2);
}

void motor_pack_init_servo(uint32_t SERVO){
	if(SERVO & MP_SERVO1){
		servo_init_qei(&mp.servo1, QEI0_BASE);
	}
	if(SERVO & MP_SERVO2){
		servo_init_qei(&mp.servo2, QEI1_BASE);
	}
}

void motor_pack_set_servo_speed(uint32_t SERVO, float speed){
	int dir = 1;
	if(speed < 0){
		dir = -1;
		speed = fabs(speed);
	}
	if(SERVO & MP_SERVO1){
		mp.servo1.target_dir = dir;
		mp.servo1.target_speed = speed;
	}
	if(SERVO & MP_SERVO2){
		mp.servo2.target_dir = dir;
		mp.servo2.target_speed = speed;
	}
}

void motor_pack_servo_start(uint32_t SERVO){
	if(SERVO & MP_SERVO1){
		servo_start(&mp.servo1);
	}
	if(SERVO & MP_SERVO2){
		servo_start(&mp.servo2);
	}
}

void motor_pack_servo_stop(uint32_t SERVO){
	if(SERVO & MP_SERVO1){
		servo_stop(&mp.servo1);
	}
	if(SERVO & MP_SERVO2){
		servo_stop(&mp.servo2);
	}
}

void motor_pack_enable_motors(){
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
}

void motor_pack_disable_motors(){
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
}

void motor_pack_init_stepper(){
	stepper_init(&mp.step);
}

void motor_pack_set_stepper_div(uint32_t div){
	mp.step.div = div;
}

void motor_pack_set_stepping_rate(uint32_t stepping_rate){
	stepper_set_stepping_rate(&mp.step, stepping_rate);
}

void motor_pack_stepper_start(){
	stepper_start(&mp.step);
}

void motor_pack_stepper_stop(){
	stepper_stop(&mp.step);
}
