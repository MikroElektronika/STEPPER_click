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
    SIM_SCGC6 |= (1 << PIT);
    NVIC_IntEnable(IVT_INT_PIT0);
    PIT_MCR = 0x00;
    PIT_LDVAL0 = 119999;
    PIT_TCTRL0 |= 2;
    PIT_TCTRL0 |= 1;
    EnableInterrupts();
}

void Timer_interrupt() iv IVT_INT_PIT0
{
    stepper_tick( (T_STEPPER_OBJ_P)&myStepper );
    PIT_TFLG0.TIF = 1;
}
