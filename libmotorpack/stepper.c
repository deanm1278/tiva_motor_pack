/*
 * stepper.c
 *
 *  Created on: May 1, 2016
 *      Author: Dean Miller
 */

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "stepper.h"
#include "DAC082S085.h"

extern void TIMER2IntHandler(void);

void stepper_init(struct stepper *s){
	DAC082S085_init(&s->dac, SSI2_BASE);
	s->stepping_rate = 0;
	s->div = 1;

	// Enable the timer
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);

	IntEnable(INT_TIMER2A);
	TimerIntRegister(TIMER2_BASE, TIMER_A, TIMER2IntHandler);
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

	//set up other necessary GPIO
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3); //AEN, BPH

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0); //BEN

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3); //APH
}

void stepper_stop(struct stepper *s){
	TimerDisable(TIMER2_BASE, TIMER_A);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0);
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0);
	s->t_val = 0;
}

void stepper_start(struct stepper *s){
	s->fires_this_move = 0;
	//
	// Enable the timer.
	//
	TimerEnable(TIMER2_BASE, TIMER_A);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0);
}

void stepper_set_stepping_rate(struct stepper *s, uint32_t stepping_rate){
	s->stepping_rate = stepping_rate;
	TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet()/(s->stepping_rate * (256 / s->div)));
}

void stepper_move_steps(struct stepper *s, uint32_t steps){
	stepper_start(s);
	while(s->fires_this_move < steps);
	stepper_stop(s);
}
