/*
 * servo.c
 *
 *  Created on: Jan 23, 2016
 *      Author: deanmiller
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"

#include "servo.h"
#include "pid.h"

extern void QEI0IntHandler(void);
extern void QEI1IntHandler(void);

void servo_init_pwm(struct servo *servo, uint32_t PWM_OUT, uint32_t PERIOD_HIGH)
{
	//TODO: make sure valid pwm base is passed in

	servo->PWM_PERIOD_HIGH = PERIOD_HIGH;
	servo->dc = PERIOD_HIGH / 2;
	servo->PWM_BASE = PWM0_BASE;
	servo->PWM_OUT = PWM_OUT;

	//set pwm clock to system clock
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

	//already enabled
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	//wait for peripheral to be ready
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)){ }

	if(PWM_OUT == PWM_OUT_0){
		//configure for count down mode with immediate updates to params
		PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

		//set period
		PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWM_PERIOD);

		GPIOPinConfigure(GPIO_PB6_M0PWM0);

		GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

		//enable outputs
		PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);

		//start timers in gen0
		PWMGenEnable(PWM0_BASE, PWM_GEN_0);
	}
	else if(PWM_OUT == PWM_OUT_4){
		//configure for count down mode with immediate updates to params
		PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

		//set period
		PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, PWM_PERIOD);

		GPIOPinConfigure(GPIO_PE4_M0PWM4);

		GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);

		//enable outputs
		PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);

		//start timers in gen2
		PWMGenEnable(PWM0_BASE, PWM_GEN_2);
	}

}

void servo_read(struct servo *servo)
{
	uint32_t i, sum = 0;
	servo->buf[servo->head] = QEIVelocityGet(servo->QEI_BASE);
	for(i=0; i<NUM_SPEED_SAMPLES; i++){
		sum += servo->buf[i];
	}
	servo->head++;
	if(servo->head == NUM_SPEED_SAMPLES){
		servo->head = 0;
	}
	servo->speed = (float)sum / NUM_SPEED_SAMPLES;
	servo->pos = QEIPositionGet(servo->QEI_BASE);
	servo->dir = QEIDirectionGet(servo->QEI_BASE);
	//TODO: handle overflow
}

void servo_loop(struct servo *servo)
{
	servo->dc += PIDcal(servo->target_speed * servo->target_dir, servo->speed * servo->dir, servo->target_pos, servo->pos, &servo->pid);
	if(servo->dc > PWM_PERIOD) servo->dc = PWM_PERIOD;
	else if(servo->dc < 0) servo->dc = 0;
	PWMPulseWidthSet(servo->PWM_BASE, servo->PWM_OUT, servo->dc);
}

void servo_init_qei(struct servo *servo, uint32_t QEI_BASE)
{
	//TODO: make sure valid qei base is passed in

	servo->QEI_BASE = QEI_BASE;

	PIDInit(&servo->pid);
	if(QEI_BASE == QEI0_BASE){
		//initialize qei 0
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);

		//Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
		HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
		HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
		HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

		//Set Pins to be PHA0 and PHB0
		GPIOPinConfigure(GPIO_PD6_PHA0);
		GPIOPinConfigure(GPIO_PD7_PHB0);

		//Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. I believe this sets the pull up and makes them inputs
		GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

		//DISable peripheral and int before configuration
		QEIDisable(QEI0_BASE);
		QEIIntDisable(QEI0_BASE, QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

		//set max to largest 32 bit int
		QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET |
		QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 4294967296 - 1);

		QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, SysCtlClockGet()/VEL_SAMPLE_PERIOD);
		QEIVelocityEnable(QEI0_BASE);

		QEIIntRegister(QEI0_BASE, QEI0IntHandler);
		// Enable the quadrature encoder.
		QEIEnable(QEI0_BASE);
		QEIPositionSet(QEI0_BASE, 0);
	}
	else if(QEI_BASE == QEI1_BASE){
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);

		//Set Pins to be PHA0 and PHB0

		GPIOPinConfigure(GPIO_PC5_PHA1);
		GPIOPinConfigure(GPIO_PC6_PHB1);

		GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);

		//set max to largest 32 bit int
		QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET |
		QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 4294967296 - 1);

		QEIFilterConfigure(QEI1_BASE, QEI_FILTCNT_2);
		QEIFilterEnable(QEI1_BASE);

		//enable velocity capture
		QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1, SysCtlClockGet()/VEL_SAMPLE_PERIOD);
		QEIVelocityEnable(QEI1_BASE);

		QEIIntRegister(QEI1_BASE, QEI1IntHandler);
		//GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6, GPIO_STRENGTH_8MA_SC, GPIO_PIN_TYPE_STD_WPU);
		//
		// Enable the quadrature encoder.
		//
		QEIEnable(QEI1_BASE);
		QEIPositionSet(QEI1_BASE, 0);
	}
}

void servo_stop(struct servo *servo){
	QEIIntDisable(servo->QEI_BASE, QEI_INTTIMER);
	servo->dc = servo->PWM_PERIOD_HIGH/2;
	PWMPulseWidthSet(servo->PWM_BASE, servo->PWM_OUT, servo->dc);
	servo->target_speed = 0;
}

void servo_start(struct servo *servo){
	QEIIntEnable(servo->QEI_BASE, QEI_INTTIMER);
}
