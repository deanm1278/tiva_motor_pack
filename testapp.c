#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"

#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"

#include "libmotorpack/libmotorpack.h"

#define STEPS_PER_REV 200
#define SEC_PER_MIN 60

int main(void) {
	//
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	//
	ROM_FPULazyStackingEnable();
	ROM_FPUEnable();

	//
	// Set the clocking to run directly from the crystal.
	// Clock to 80MHZ
	//
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
					   SYSCTL_OSC_MAIN);

	ROM_IntMasterEnable();

	//initialize the motor pack
	motor_pack_init();

	//initialize the stepper
	motor_pack_init_stepper();

	//set stepper to 20 rpm
	uint32_t rpm = 20;
	motor_pack_set_stepping_rate(rpm * STEPS_PER_REV / SEC_PER_MIN);

	//set to different speeds
	motor_pack_set_servo_speed(MP_SERVO1, -2.6);
	motor_pack_set_servo_speed(MP_SERVO2, 3);

	//initialize both servos
	motor_pack_init_servo(MP_SERVO1 | MP_SERVO2);

	//enable all drivers
	motor_pack_enable_motors();

	//start all motors
	motor_pack_stepper_start();
	motor_pack_servo_start(MP_SERVO1 | MP_SERVO2);

	//loop forever
	while(1){
	}

	return 0;
}
