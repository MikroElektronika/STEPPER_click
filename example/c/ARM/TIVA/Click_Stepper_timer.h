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
    SYSCTL_RCGCTIMER_R0_bit = 1;
    EnableInterrupts();
    TIMER_CTL_TAEN_bit = 0;
    TIMER0_CFG   = 4;
    TIMER0_TAMR |= 2;
    TIMER0_TAPR  = 1;
    TIMER0_TAILR = 60000;
    NVIC_IntEnable(IVT_INT_TIMER0A_16_32_bit);
    TIMER_IMR_TATOIM_bit = 1;
    TIMER_CTL_TAEN_bit   = 1;
}

void Timer_interrupt() iv IVT_INT_TIMER0A_16_32_bit 
{
    stepper_tick( (T_STEPPER_OBJ_P)&myStepper );
    TIMER_ICR_TATOCINT_bit = 1;
}
