![MikroE](http://www.mikroe.com/img/designs/beta/logo_small.png)

---

# STEPPER Click

---

- **CIC Prefix**  : STEPPER
- **Author**      : Nemanja Medakovic
- **Verison**     : 1.0.0
- **Date**        : Jun 2018.

---

### Software Support

We provide a library for the STEPPER Click on our [LibStock](https://libstock.mikroe.com/projects/view/2318/stepper-click) 
page, as well as a demo application (example), developed using MikroElektronika 
[compilers](http://shop.mikroe.com/compilers). The demo can run on all the main 
MikroElektronika [development boards](http://shop.mikroe.com/development-boards).

**Library Description**

Library carries everything needed for stepper motor control including speed
and acceleration setup. Library is also adjustable to working on different amount of
ticks per second, also speed and acceleration can be provided in float format.
Buffer used for movement calculation is defined by user so this library can 
be adjusted for MCUs with very limited RAM resources. Check documentation for 
more details how to use it.

Key functions :

- ``` uint8_t stepper_setSpeed( float minSpeed, float maxSpeed, float accelRatio, T_STEPPER_OBJ obj ) ``` - Setup motor speed
- ``` uint8_t stepper_setRoute( const uint8_t direction, uint32_t steps, T_STEPPER_OBJ obj ) ``` - Setup new route
- ``` void stepper_start( T_STEPPER_OBJ obj ) ``` - Start motor movement

**Examples Description**

The application is composed of three sections :

- System Initialization - Initializes all GPIO pins found on Stepper Click and timer to 1ms interrupt.
- Application Initialization - First segment initializes driver and stepper control. 
  Second segment setup movement limits, maximum and minimum speed, and acceleration ratio. 
  Third segment enables motor and setup new route which will be called from application task.
- Application Task - (code snippet) - Sequentialy moves motor. Executes movement until the end.
  After that stops the motor for two seconds and then continues sequence.


```.c
void applicationTask()
{
    stepper_start( (T_STEPPER_OBJ_P)&myStepper );
    while ( myStepper.status.running )
    {
        stepper_process( (T_STEPPER_OBJ_P)&myStepper );
    }
    Delay_ms( 2000 );
}
```

In addition to library function calls example carries necessay Timer ISR and Timer initialization. Check 
Timer initialization setings and update it according to your MCU - [Timer Calculator](https://www.mikroe.com/timer-calculator).

The full application code, and ready to use projects can be found on our 
[LibStock](https://libstock.mikroe.com/projects/view/2318/stepper-click) page.

**Additional notes and informations**

Depending on the development board you are using, you may need 
[USB UART click](http://shop.mikroe.com/usb-uart-click), 
[USB UART 2 Click](http://shop.mikroe.com/usb-uart-2-click) or 
[RS232 Click](http://shop.mikroe.com/rs232-click) to connect to your PC, for 
development systems with no UART to USB interface available on the board. The 
terminal available in all Mikroelektronika 
[compilers](http://shop.mikroe.com/compilers), or any other terminal application 
of your choice, can be used to read the message.

---
---
