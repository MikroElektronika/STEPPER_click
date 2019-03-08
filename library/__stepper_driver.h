/*
    __stepper_driver.h

-----------------------------------------------------------------------------

  This file is part of mikroSDK.
  
  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

/**
@file   __stepper_driver.h
@brief    STEPPER Driver
@mainpage STEPPER Click
@{

@image html sch.jpg

@}

@defgroup   STEPPER
@brief      STEPPER Click Driver
@{

| Global Library Prefix | **STEPPER** |
|:---------------------:|:-----------------:|
| Version               | **1.0.0**    |
| Date                  | **Jun 2018.**      |
| Developer             | **Nemanja Medakovic**     |

*/
/* -------------------------------------------------------------------------- */

#include "stdint.h"

#ifndef _STEPPER_H_
#define _STEPPER_H_

/** 
 * @macro T_STEPPER_P
 * @brief Driver Abstract type 
 */
#define T_STEPPER_P      const uint8_t*
#define T_STEPPER_OBJ_P    uint8_t*

/** @defgroup STEPPER_COMPILE Compilation Config */              /** @{ */

//  #define   __STEPPER_DRV_SPI__                            /**<     @macro __STEPPER_DRV_SPI__  @brief SPI driver selector */
//  #define   __STEPPER_DRV_I2C__                            /**<     @macro __STEPPER_DRV_I2C__  @brief I2C driver selector */                                          
// #define   __STEPPER_DRV_UART__                           /**<     @macro __STEPPER_DRV_UART__ @brief UART driver selector */ 

                                                                       /** @} */
/** @defgroup STEPPER_VAR Variables */                           /** @{ */

extern const uint8_t _STEPPER_DIR_CW;
extern const uint8_t _STEPPER_DIR_CCW;
extern const uint8_t _STEPPER_RUN;
extern const uint8_t _STEPPER_STOPED;
extern const uint8_t _STEPPER_READY;
extern const uint8_t _STEPPER_BORDER_HIT;
extern const uint8_t _STEPPER_FULL_STEP;
extern const uint8_t _STEPPER_HALF_STEP;
extern const uint8_t _STEPPER_QUARTER_STEP;
extern const uint8_t _STEPPER_8_MICROSTEP;
extern const uint8_t _STEPPER_MOTOR_ENABLE;
extern const uint8_t _STEPPER_MOTOR_DISABLE;

                                                                       /** @} */
/** @defgroup STEPPER_TYPES Types */                             /** @{ */

typedef void (*T_stepper_gpioSetFp)(uint8_t);

/**
 * @struct T_stepper_ctl
 * @brief Steper Driver Control
 *
 * Structure carries pointers to functions used for motor movement.
 */
typedef struct
{
    T_stepper_gpioSetFp  fpDir;       /**< Direction pin */
    T_stepper_gpioSetFp  fpStep;      /**< Step pin */
    T_stepper_gpioSetFp  fpEnable;    /**< Enable pin */
    T_stepper_gpioSetFp  fpStepMode1;    /**< Step mode pin 1 */
    T_stepper_gpioSetFp  fpStepMode2;    /**< Step mode pin 2 */
  
}T_stepper_ctl;

/**
 * @struct T_stepper_status
 * @brief Steper Driver Status
 *
 * Structure carries current status of stepper driver.
 */
typedef struct
{
    uint32_t  position;                       /**< Absolute position */
    uint32_t  target;                         /**< Current target position */
    uint32_t  timer;                          /**< Current timer value */

    uint8_t   direction;                      /**< Current movement direction */
    uint8_t   running;                        /**< Running flag */
    uint8_t   operation;                      /**< Current operation */

}T_stepper_status;

/**
 * @struct T_stepper_cfg
 * @brief Steper Driver Configuration
 *
 * Structure carries current configuration of stepper driver.
 */
typedef struct
{
    uint32_t  borderMin;                      /**< Absolute position minimum */
    uint32_t  borderMax;                      /**< Absolute position maximum */

    float     accelRatio;                     /**< Acceleration ratio */
 
    float     speedMin;                       /**< Minimum speed (start speed) */
    float     speedMax;                       /**< Maximum speed */

}T_stepper_cfg;

/**
 * @struct T_stepper_route
 * @brief Steper Driver Route
 *
 * Structure carries current route of stepper driver.
 */
typedef struct
{
    uint32_t  position;                       /**< Current position inside route */
    uint32_t  positionAcc;                    /**< Current acceleration position */

    uint32_t  accelStop;                      /**< Position where acceleration should stop */
    uint32_t  accelStart;                     /**< Position where deacceleration should start */

    uint32_t  routeMax;                       /**< Maximum size of acceleration deacceleration steps (timing buffer size) */
    uint32_t* routeTimer;                     /**< Timing buffer pointer */

}T_stepper_route;

/**
 * @struct T_stepper_obj
 * @brief Steper Driver Object
 *
 * Structure can be observed as an instance of stepper motor.
 */
typedef struct
{
    T_stepper_ctl       control;
    T_stepper_cfg       config;
    T_stepper_status    status;
    T_stepper_route     route;

}T_stepper_obj;

                                                                       /** @} */
#ifdef __cplusplus
extern "C"{
#endif

/** @defgroup STEPPER_INIT Driver Initialization */              /** @{ */

#ifdef   __STEPPER_DRV_SPI__
void stepper_spiDriverInit(T_STEPPER_P gpioObj, T_STEPPER_P spiObj);
#endif
#ifdef   __STEPPER_DRV_I2C__
void stepper_i2cDriverInit(T_STEPPER_P gpioObj, T_STEPPER_P i2cObj, uint8_t slave);
#endif
#ifdef   __STEPPER_DRV_UART__
void stepper_uartDriverInit(T_STEPPER_P gpioObj, T_STEPPER_P uartObj);
#endif

void stepper_gpioDriverInit(T_STEPPER_P gpioObj);
                                                                       /** @} */
/** @defgroup STEPPER_FUNC Driver Functions */                   /** @{ */

/**
 * @brief Steper Control Initialization
 *
 * @param[in] gpioObj pointer to mikroBUS GPIO structure
 * @param[in] obj pointer to stepper object
 *
 * Function initializes pointers necessary for stepper control. Stepper status is also initialized
 * to default values. 
 *
 * @note
 * This function must be called after ```stepper_gpioDriverInit``` alongside with ```stepper_cfgInit```
 * and prior to any other function from this library.
 */
uint8_t stepper_ctlInit( T_STEPPER_P gpioObj, T_STEPPER_OBJ_P stepperObj );

/**
 * @brief Steper Configuration Initialization
 *
 * @param[in] buffSize size of buffer used for timing calculation
 * @param[in] pBuff pointer to buffer for timing calculation
 * @param[in] tickPerSec number of timerISR calls per second
 * @param[in] obj pointer to stepper object
 *
 * Function initializes configuration necessary for stepper control. Also speed confugraion 
 * parameters are configured to default values.
 *
 * - Border minimum 0x00000000
 * - Border maximum 0xFFFFFFFF
 * - Accel Ratio 1.0
 * - Minimum speed 1.0
 * - Maximum speed 100.0
 *
 * @note
 * This function must be called after ```stepper_gpioDriverInit``` alongside with ```stepper_ctlInit``` 
 * and prior to any other function from this library.
 */
uint8_t stepper_cfgInit( const uint32_t buffSize, uint32_t *pBuff, float stepPerSec, T_STEPPER_OBJ_P obj );

/**
 * @brief Setup Speed Parameters
 *
 * @param[in] minSpeed minimum or start speed
 * @param[in] maxSpeed maximum speed 
 * @param[in] accelRatio acceleration ratio per step
 * @param[in] obj pointer to stepper object
 *
 * Function setup parameters related to stepper movement.
 *
 * @note
 * This function must be called after ```stepper_gpioDriverInit``` and prior to any other function
 * from this library.
 */
uint8_t stepper_setSpeed( float minSpeed, float maxSpeed, float accelRatio, T_STEPPER_OBJ_P obj );

/**
 * @brief Setup Movement Limits
 *
 * @param[in] minPosition minimum absolute position
 * @param[in] maxPosition maximum absolute position
 *
 * Function setup absolute limits for stepper movement.
 */
uint8_t stepper_setBorder( uint32_t minPosition, uint32_t maxPosition, T_STEPPER_OBJ_P obj );

/**
 * @brief Setup Absolute Position
 *
 * @param[in] newPosition absolute position
 * @param[in] obj pointer to stepper object
 *
 * Function set new absolute position.
 *
 * @note
 * Function should not be called during motor movement. Also this function requires route 
 * recalculation - calling of ```stepper_setRoute```.
 */
uint8_t stepper_setPosition( uint32_t newPosition, T_STEPPER_OBJ_P obj );

/**
 * @brief Setup New Route
 *
 * @param[in] direction movement direction
 * @param[in] steps number of steps
 * @param[in] obj pointer to stepper object
 *
 * Function setup parameters related to new route.
 *
 * @note
 * This function must be called prior to ```stepper_start```.
 */
uint8_t stepper_setRoute( const uint8_t direction, uint32_t steps, T_STEPPER_OBJ_P obj );

/**
 * @brief Single Step
 *
 * @param[in] direction movement direction
 * @param[in] obj pointer to stepper object
 *
 * Function executes single step in desired direction. Usefull in case of G code apps.
 */
void stepper_step( const uint8_t direction, T_STEPPER_OBJ_P obj );

/**
 * @brief Enable Phases
 *
 * @param[in] state motor state ( 0 - ON / 1 - OFF )
 * @param[in] obj pointer to stepper object
 * @param[in] stepper_mode determines the step mode
 *
 * Function enables motor driver and puts motor to the desired step mode.
 */
void stepper_enable( const uint8_t state, T_STEPPER_OBJ_P obj, const uint8_t stepper_mode );

/**
 * @brief Stop Movement
 *
 * @param[in] obj pointer to stepper object
 *
 * Function immidiately starts stepper movement according to last configured route.
 */
void stepper_start( T_STEPPER_OBJ_P obj );

/**
 * @brief Start Movement
 *
 * @param[in] obj pointer to stepper object
 *
 * Function instantly stop stepper movement.
 *
 * @note
 * Last route setup will be retereived so new route setup is not required in case of repetition.
 */
void stepper_stop( T_STEPPER_OBJ_P obj );

/**
 * @brief Stepper ISR
 *
 * @param[in] obj pointer to stepper object
 *
 * Function which must be called in exact amount of times per second provided during ```stepper_cfgInit```
 * function call.
 *
 * @note 
 * Usage of ```stepper_process``` and ```stepper_tick``` is forbiden when this function is used 
 * for motor driving.
 */
void stepper_isr( T_STEPPER_OBJ_P obj );

/**
 * @brief Stepper State Machine
 *
 * @param[in] obj pointer to stepper object
 *
 * Function must be called as frequently as possible.
 *
 * @note 
 * This function is used alongside with ```stepper_tick```. In case of usage of this two functions 
 * usage of ```stepper_isr``` is forbidden.
 */
void stepper_process( T_STEPPER_OBJ_P obj );

/**
 * @brief Stepper Tick
 *
 * @param[in] obj pointer to stepper object
 *
 * Function which must be called in exact amount of times per second provided during ```stepper_cfgInit```
 * function call. The best case is call from timer ISR.
 *
 * @note
 * This function is used alongside with ```stepper_process```. In case of usage of this two functions 
 * usage of ```stepper_isr``` is forbidden.
 */
void stepper_tick( T_STEPPER_OBJ_P obj );

                                                                       /** @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif

/**
    @example Click_STEPPER_STM.c
    @example Click_STEPPER_TIVA.c
    @example Click_STEPPER_CEC.c
    @example Click_STEPPER_KINETIS.c
    @example Click_STEPPER_MSP.c
    @example Click_STEPPER_PIC.c
    @example Click_STEPPER_PIC32.c
    @example Click_STEPPER_DSPIC.c
    @example Click_STEPPER_AVR.c
    @example Click_STEPPER_FT90x.c
    @example Click_STEPPER_STM.mbas
    @example Click_STEPPER_TIVA.mbas
    @example Click_STEPPER_CEC.mbas
    @example Click_STEPPER_KINETIS.mbas
    @example Click_STEPPER_MSP.mbas
    @example Click_STEPPER_PIC.mbas
    @example Click_STEPPER_PIC32.mbas
    @example Click_STEPPER_DSPIC.mbas
    @example Click_STEPPER_AVR.mbas
    @example Click_STEPPER_FT90x.mbas
    @example Click_STEPPER_STM.mpas
    @example Click_STEPPER_TIVA.mpas
    @example Click_STEPPER_CEC.mpas
    @example Click_STEPPER_KINETIS.mpas
    @example Click_STEPPER_MSP.mpas
    @example Click_STEPPER_PIC.mpas
    @example Click_STEPPER_PIC32.mpas
    @example Click_STEPPER_DSPIC.mpas
    @example Click_STEPPER_AVR.mpas
    @example Click_STEPPER_FT90x.mpas
*/                                                                     /** @} */
/* -------------------------------------------------------------------------- */
/*
  __stepper_driver.h

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the MikroElektonika.

4. Neither the name of the MikroElektonika nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY MIKROELEKTRONIKA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL MIKROELEKTRONIKA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------- */