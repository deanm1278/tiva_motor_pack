# tiva_motor_pack
3 axis (2 servo 1 stepper) motor booster pack for EK-TM4C123GXL launchpad board

This booster pack drives 2 servos with encoders and 1 stepper motor.

The resolution limits are not yet well tested, but it definitely runs 2 brushed dc servos at 10khz velocity sampling and 1 stepper at 1/256 microstepping very smoothly. Stepper tested is a NEMA 17 2.0A at 2.8V motor that runs very smooth when microstepped. Servos tested are Harmonic drive RH-8 motors with Tamagawa encoders and also run well.

The software libraries include PID contol for the servos. Further tuning of PID may be necessary.

Servo drivers are TI DRV8871 (datasheet here: http://www.ti.com/lit/ds/symlink/drv8871.pdf)
stepper driver is a TI DRV8881E (datasheet here: http://www.ti.com/lit/ds/symlink/drv8881.pdf)

Autotune feature on the DRV8881E seems to work quite nicely :)
