/*
Example for STEPPER Click

    Date          : Jun 2018.
    Author        : Nemanja Medakovic

Test configuration STM32 :
    
    MCU              : STM32F107VCT6
    Dev. Board       : EasyMx PRO v7 for STM32
    ARM Compiler ver : v6.0.0.0

---

Description :

The application is composed of three sections :

- System Initialization - Initializes all GPIO pins found on Stepper Click and timer to 1ms interrupt.
- Application Initialization - First segment initializes driver and stepper control. 
  Second segment setup movement limits, maximum and minimum speed, and acceleration ratio. 
  Third segment enables motor and setup new route which will be called from application task.
- Application Task - (code snippet) - Sequentialy moves motor. Executes movement until the end.
  After that stops the motor for two seconds and then continues sequence.

*/

#include "Click_STEPPER_types.h"
#include "Click_STEPPER_config.h"
#include "Click_STEPPER_timer.h"

uint32_t      tmpBuffer[ 256 ];
T_stepper_obj myStepper;

void systemInit()
{
    mikrobus_gpioInit(_MIKROBUS1, _MIKROBUS_INT_PIN, _GPIO_OUTPUT);
    mikrobus_gpioInit(_MIKROBUS1, _MIKROBUS_AN_PIN, _GPIO_OUTPUT);
    mikrobus_gpioInit(_MIKROBUS1, _MIKROBUS_RST_PIN, _GPIO_OUTPUT);
    mikrobus_gpioInit(_MIKROBUS1, _MIKROBUS_CS_PIN, _GPIO_OUTPUT);
    mikrobus_gpioInit(_MIKROBUS1, _MIKROBUS_PWM_PIN, _GPIO_OUTPUT);

    stepper_confgTimer();
    Delay_ms( 100 );
}

void applicationInit()
{
// SEG 1
    stepper_gpioDriverInit( (T_STEPPER_P)&_MIKROBUS1_GPIO );
    stepper_ctlInit( (T_STEPPER_P)&_MIKROBUS1_GPIO, (T_STEPPER_OBJ_P)&myStepper );
    stepper_cfgInit( 256, &tmpBuffer[0], 1000.0, (T_STEPPER_OBJ_P)&myStepper );

// SEG 2
    stepper_setBorder( 0x00000000, 0xFFFFFFFF, (T_STEPPER_OBJ_P)&myStepper );
    stepper_setSpeed( 10.0, 100.0, 0.1, (T_STEPPER_OBJ_P)&myStepper );

// SEG 3 
    stepper_enable( _STEPPER_MOTOR_ENABLE, (T_STEPPER_OBJ_P)&myStepper, _STEPPER_FULL_STEP );
    stepper_setRoute( _STEPPER_DIR_CCW, 200, (T_STEPPER_OBJ_P)&myStepper );
}

void applicationTask()
{
    stepper_start( (T_STEPPER_OBJ_P)&myStepper );
    while ( myStepper.status.running )
    {
        stepper_process( (T_STEPPER_OBJ_P)&myStepper );
    }
    Delay_ms( 2000 );
}

void main()
{
    systemInit();
    applicationInit();

    while (1)
    {
        applicationTask();
    }
}