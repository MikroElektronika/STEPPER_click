/*
    __stepper_driver.c

-----------------------------------------------------------------------------

  This file is part of mikroSDK.

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

#include "__stepper_driver.h"
#include "__stepper_hal.c"

/* ------------------------------------------------------------------- MACROS */



/* ---------------------------------------------------------------- VARIABLES */

#ifdef   __STEPPER_DRV_I2C__
static uint8_t _slaveAddress;
#endif

const uint8_t _STEPPER_DIR_CW  = 0;
const uint8_t _STEPPER_DIR_CCW = 1;

const uint8_t _STEPPER_RUN        = 0;
const uint8_t _STEPPER_STOPED     = 1;
const uint8_t _STEPPER_READY      = 2;
const uint8_t _STEPPER_BORDER_HIT = 3;

const uint8_t _STEPPER_FULL_STEP    = 0;
const uint8_t _STEPPER_HALF_STEP    = 1;
const uint8_t _STEPPER_QUARTER_STEP = 2;
const uint8_t _STEPPER_8_MICROSTEP  = 3;

const uint8_t _STEPPER_MOTOR_ENABLE  = 0;
const uint8_t _STEPPER_MOTOR_DISABLE = 1;

static float _stepper_tickPerSec;

/* -------------------------------------------- PRIVATE FUNCTION DECLARATIONS */

static uint32_t _calc_steps( float minSp, float maxSp, float accRate );

static void _calc_route( uint32_t accSt, uint32_t st, T_stepper_obj *obj );

/* --------------------------------------------- PRIVATE FUNCTION DEFINITIONS */

static uint32_t _calc_steps( float minSp, float maxSp, float accRate )
{
    uint32_t accSt = 0;

    if (accRate != 0.0f)
    {
        while (minSp < maxSp)
        {
            minSp += (minSp * accRate);
            ++accSt;
        }
    }
    else
    {
        return 1;
    }

    return accSt;
}

static void _calc_route( uint32_t accSt, uint32_t st, T_stepper_obj *obj )
{

    float    tmpSpeed;
    float    tmpTiming;
    uint32_t cnt;

    T_stepper_cfg   *cfg    = &( obj->config );
    T_stepper_route *route  = &( obj->route );

    route->position         = 0;
    route->positionAcc      = 0;
    route->accelStop        = accSt;
    route->accelStart       = st - accSt;
    tmpSpeed                = cfg->speedMin;
    tmpTiming               = _stepper_tickPerSec / tmpSpeed;

// ROUTE CALCULATION >>
    route->routeTimer[0]     = (uint32_t)tmpTiming;

    for (cnt = 1; cnt < accSt; cnt++)
    {
        tmpSpeed += ( tmpSpeed * cfg->accelRatio );
        tmpTiming = ( _stepper_tickPerSec / tmpSpeed );

        route->routeTimer[cnt] = (uint32_t)tmpTiming;
    }
    
    tmpSpeed += ( tmpSpeed * cfg->accelRatio );
    tmpTiming = ( _stepper_tickPerSec / tmpSpeed );
    
    if (tmpTiming < ( _stepper_tickPerSec / cfg->speedMax ))
        tmpTiming = ( _stepper_tickPerSec / cfg->speedMax );
        
    route->routeTimer[cnt] = (uint32_t)tmpTiming;
// <<
}

/* --------------------------------------------------------- PUBLIC FUNCTIONS */

#ifdef   __STEPPER_DRV_SPI__

void stepper_spiDriverInit(T_STEPPER_P gpioObj, T_STEPPER_P spiObj)
{
    hal_spiMap( (T_HAL_P)spiObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __STEPPER_DRV_I2C__

void stepper_i2cDriverInit(T_STEPPER_P gpioObj, T_STEPPER_P i2cObj, uint8_t slave)
{
    _slaveAddress = slave;
    hal_i2cMap( (T_HAL_P)i2cObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __STEPPER_DRV_UART__

void stepper_uartDriverInit(T_STEPPER_P gpioObj, T_STEPPER_P uartObj)
{
    hal_uartMap( (T_HAL_P)uartObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif

void stepper_gpioDriverInit(T_STEPPER_P gpioObj)
{
    hal_gpioMap( (T_HAL_P)gpioObj );
}

/* ----------------------------------------------------------- IMPLEMENTATION */

uint8_t stepper_ctlInit( T_STEPPER_P gpioObj, T_STEPPER_OBJ_P obj )
{
    T_stepper_obj    *tmp     = ( T_stepper_obj* )obj;
    T_stepper_ctl    *control = &( tmp->control );
    T_stepper_status *status  = &( tmp->status );

// ASING POINTERS    
    control->fpDir    = hal_gpio_csSet;
    control->fpStep   = hal_gpio_pwmSet;
    control->fpEnable = hal_gpio_intSet;
    control->fpStepMode1 = hal_gpio_anSet;
    control->fpStepMode2 = hal_gpio_rstSet;

    status->position  = 0;
    status->target    = 0;
    status->timer     = 0;
    status->running   = 0;
    status->direction = _STEPPER_DIR_CW;
    status->operation = _STEPPER_READY;

// ENABLE
    control->fpDir( 0 );
    
    return 0;
}

void stepper_step( const uint8_t direction, T_STEPPER_OBJ_P obj )
{
    T_stepper_obj *tmp     = ( T_stepper_obj* )obj;
    T_stepper_ctl *control = &( tmp->control );

// IMPLEMENT MOVEMENT BY ONE STEP
    if ( direction )
        control->fpDir( 1 );
    else
        control->fpDir( 0 );

    control->fpStep( 1 );
    Delay_1us();
    control->fpStep( 0 );
}

uint8_t stepper_cfgInit( const uint32_t buffSize, uint32_t *pBuff, float tickPerSec, T_STEPPER_OBJ_P obj )
{
    T_stepper_obj   *tmp   = ( T_stepper_obj* )obj;
    T_stepper_cfg   *cfg   = &( tmp->config );
    T_stepper_route *route = &( tmp->route );

    if (0 == buffSize)
        return 1;

    cfg->borderMin       = 0x00000000;
    cfg->borderMax       = 0xFFFFFFFF;
    cfg->accelRatio      = 1.0;
    cfg->speedMin        = 1.0;
    cfg->speedMax        = 100.0;

    route->routeMax      = buffSize;
    route->routeTimer    = pBuff;

    _stepper_tickPerSec = tickPerSec;

    return 0;
}

uint8_t stepper_setSpeed( float minSpeed, float maxSpeed, float accelRatio, T_STEPPER_OBJ_P obj )
{
    T_stepper_obj   *tmp   = ( T_stepper_obj* )obj;
    T_stepper_cfg   *cfg   = &( tmp->config );
    T_stepper_route *route = &( tmp->route );

    if ((0.0f == minSpeed) || (minSpeed > _stepper_tickPerSec))
        return 1;

    if ((0.0f == maxSpeed) || (maxSpeed > _stepper_tickPerSec))
        return 1;

    if ((maxSpeed < minSpeed))
        return 1;

    if ( _calc_steps(minSpeed, maxSpeed, accelRatio) > route->routeMax )
        return 2;                                                                                   // < TIMER BUFFER SMALL

    cfg->speedMin   = minSpeed;
    cfg->speedMax   = maxSpeed;
    cfg->accelRatio = accelRatio;

    return 0;
}

uint8_t stepper_setBorder( uint32_t minPosition, uint32_t maxPosition, T_STEPPER_OBJ_P obj )
{
    T_stepper_obj *tmp   = ( T_stepper_obj* )obj;
    T_stepper_cfg *cfg   = &( tmp->config );

    if (( minPosition == 0xFFFFFFFF ) || ( maxPosition == 0 ))
        return 1;

    if (( minPosition >= maxPosition ))
        return 1;

    cfg->borderMin   = minPosition;
    cfg->borderMax   = maxPosition;

    return 0;
}

uint8_t stepper_setPosition( uint32_t newPosition, T_STEPPER_OBJ_P obj )
{
    T_stepper_obj *tmp       = ( T_stepper_obj* )obj;
    T_stepper_cfg *cfg       = &( tmp->config );
    T_stepper_status *status = &( tmp->status );

    if (( newPosition < cfg->borderMin ) || ( newPosition > cfg->borderMax ))
        return 1;

    status->position = newPosition;

    return 0;
}

uint8_t stepper_setRoute( const uint8_t direction, uint32_t steps, T_STEPPER_OBJ_P obj )
{    
    uint32_t tmpMinSteps;

    T_stepper_obj    *tmp    = ( T_stepper_obj* )obj;
    T_stepper_cfg    *cfg    = &( tmp->config );
    T_stepper_status *status = &( tmp->status );

    tmpMinSteps = _calc_steps( cfg->speedMin, cfg->speedMax, cfg->accelRatio );

    if (( tmpMinSteps * 2 ) > steps)
        _calc_route( steps / 2, steps, tmp );
    else
        _calc_route( tmpMinSteps, steps, tmp );

    if (( status->direction = direction ) == _STEPPER_DIR_CW)
        status->target = status->position + steps;
    else
        status->target = status->position - steps;

    return 0;
}

void stepper_enable( const uint8_t state, T_STEPPER_OBJ_P obj, const uint8_t stepper_mode )
{
    T_stepper_obj *tmp     = ( T_stepper_obj* )obj;    
    T_stepper_ctl *control = &( tmp->control );

    control->fpEnable( state );

    switch (stepper_mode)
    {
        case 0 : 
        {
            control->fpStepMode1( 0 );
            control->fpStepMode2( 0 );
        break;
        }
        case 1 : 
        {
            control->fpStepMode1( 1 );
            control->fpStepMode2( 0 );
        break;
        }
        case 2 : 
        {
            control->fpStepMode1( 0 );
            control->fpStepMode2( 1 );
        break;
        }
        case 3 : 
        {
            control->fpStepMode1( 1 );
            control->fpStepMode2( 1 );
        break;
        }
        default : 
        {
            control->fpStepMode1( 0 );
            control->fpStepMode2( 0 );
        break;
        }
    }
}

void stepper_start( T_STEPPER_OBJ_P obj )
{
    T_stepper_obj    *tmp    = ( T_stepper_obj* )obj;
    T_stepper_status *status = &( tmp->status );
    T_stepper_route  *route  = &( tmp->route );

    route->position    = 0;
    route->positionAcc = 0;

    status->operation  = _STEPPER_RUN;
    status->timer      = 0;
    status->running    = 1;
}

void stepper_stop( T_STEPPER_OBJ_P obj )
{
    T_stepper_obj    *tmp    = ( T_stepper_obj* )obj;
    T_stepper_status *status = &( tmp->status );
    T_stepper_route  *route  = &( tmp->route );

// RETREIVE LAST ROUTE >>
    if (status->direction == _STEPPER_DIR_CW)
        status->target = status->position + ((status->target - status->position) + route->position);
    else
        status->target = status->position - ((status->position - status->target) + route->position);
// <<

    status->running    = 0;
    status->timer      = 0;
    status->operation  = _STEPPER_STOPED;

    route->position    = 0;
    route->positionAcc = 0;
}

void stepper_isr( T_STEPPER_OBJ_P obj )
{
    T_stepper_obj    *tmp    = ( T_stepper_obj* )obj;
    T_stepper_status *status = &( tmp->status );
    T_stepper_route  *route  = &( tmp->route );
    T_stepper_cfg    *cfg    = &( tmp->config );

    if (status->running)
    {
        if (status->timer++ == route->routeTimer[route->positionAcc])                               // < MAKE STEP DECISION
        {
            if (route->position < route->accelStop)                                                 // < ACCELERATION
                route->positionAcc++;

            if (route->position > route->accelStart)                                                // < DEACCELERATION
                route->positionAcc--;

            if (status->direction == _STEPPER_DIR_CW )
                status->position++;
            else
                status->position--;

            route->position++;
            stepper_step( status->direction, obj );
            status->timer = 0;

            if (status->position == status->target)                                                 // < LAST STEP
            {
            // RETREIVE LAST ROUTE >>
                if (status->direction == _STEPPER_DIR_CW)
                    status->target = status->position + route->position;
                else
                    status->target = status->position - route->position;
            // <<
                status->running    = 0;
                status->timer      = 0;
                status->operation  = _STEPPER_READY;

                route->position    = 0;
                route->positionAcc = 0;
            }
        }
    }

    if (( status->position > cfg->borderMax ) || ( status->position < cfg->borderMin ))             // BORDER HIT
    {
        // RETREIVE LAST ROUTE >>
            if (status->direction == _STEPPER_DIR_CW)
                status->target = status->position + ((status->target - status->position) + route->position);
            else
                status->target = status->position - ((status->position - status->target) + route->position);
        // <<
            status->running    = 0;
            status->timer      = 0;
            status->operation  = _STEPPER_BORDER_HIT;

            route->position    = 0;
            route->positionAcc = 0;
    }
}

void stepper_process( T_STEPPER_OBJ_P obj )
{
    T_stepper_obj    *tmp    = ( T_stepper_obj* )obj;
    T_stepper_status *status = &( tmp->status );
    T_stepper_route  *route  = &( tmp->route );
    T_stepper_cfg    *cfg    = &( tmp->config );

    if (status->running)
    {
        if (status->timer == route->routeTimer[route->positionAcc])                                 // < MAKE STEP DECISION
        {
            if (route->position < route->accelStop)                                                 // < ACCELERATION
                route->positionAcc++;

            if (route->position > route->accelStart)                                                // < DEACCELERATION
                route->positionAcc--;

            if (status->direction == _STEPPER_DIR_CW )
                status->position++;
            else
                status->position--;

            route->position++;
            stepper_step( status->direction, obj );
            status->timer = 0;

            if (status->position == status->target)                                                 // < LAST STEP
            {
            // RETREIVE LAST ROUTE >>
                if (status->direction == _STEPPER_DIR_CW)
                    status->target = status->position + route->position;
                else
                    status->target = status->position - route->position;
            // <<
                status->running    = 0;
                status->timer      = 0;
                status->operation  = _STEPPER_READY;

                route->position    = 0;
                route->positionAcc = 0;
            }
        }
    }

    if (( status->position > cfg->borderMax ) || ( status->position < cfg->borderMin ))           // BORDER HIT
    {
        // RETREIVE LAST ROUTE >>
            if (status->direction == _STEPPER_DIR_CW)
                status->target = status->position + ((status->target - status->position) + route->position);
            else
                status->target = status->position - ((status->position - status->target) + route->position);
        // <<
            status->running    = 0;
            status->timer      = 0;
            status->operation  = _STEPPER_BORDER_HIT;

            route->position    = 0;
            route->positionAcc = 0;
    }
}

void stepper_tick( T_STEPPER_OBJ_P obj )
{
    T_stepper_obj    *tmp    = ( T_stepper_obj* )obj;
    T_stepper_status *status = &( tmp->status );

    if (status->running)
        status->timer++;
}

/* -------------------------------------------------------------------------- */
/*
  __stepper_driver.c

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