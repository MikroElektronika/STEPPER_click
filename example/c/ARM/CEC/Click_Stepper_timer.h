/*

Use mikroE Timer Calculator to generate proper timer configuration
and timer ISR.

https://www.mikroe.com/timer-calculator

*/
#include "Click_STEPPER_types.h"

#define __STEPPER_TIMER__

extern T_stepper_obj myStepper;

static void stepper_confgTimer()
{
    // Configure Timer
}

static void Timer_interrupt()
{
    stepper_tick( (T_STEPPER_OBJ_P)&myStepper );
    // Reset Flag
}