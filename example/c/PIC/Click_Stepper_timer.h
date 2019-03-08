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
    T1CON = 0x01;
    TMR1IF_bit = 0;
    TMR1H = 0xC1;
    TMR1L = 0x80;
    TMR1IE_bit = 1;
    INTCON = 0xC0;
}

void interrupt()
{
    if (TMR1IF_bit != 0)
    { 
        stepper_tick( (T_STEPPER_OBJ_P)&myStepper );
        TMR1IF_bit = 0;
        TMR1H = 0xC1;
        TMR1L = 0x80;
    }
}