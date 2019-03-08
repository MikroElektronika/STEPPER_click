#ifndef _STEPPER_T_
#define _STEPPER_T_

#include "stdint.h"

#ifndef _STEPPER_H_

#define T_STEPPER_P const uint8_t* 
#define T_STEPPER_OBJ_P     uint8_t*

typedef void (*T_stepper_gpioSetFp)(uint8_t);

typedef struct
{
    T_stepper_gpioSetFp  fpDir;
    T_stepper_gpioSetFp  fpStep;
    T_stepper_gpioSetFp  fpEnable;
    T_stepper_gpioSetFp  fpStepMode1;
    T_stepper_gpioSetFp  fpStepMode2;

}T_stepper_ctl;

typedef struct
{
    uint32_t  position;
    uint32_t  target;
    uint32_t  timer;

    uint8_t   direction;
    uint8_t   running;
    uint8_t   operation;

}T_stepper_status;

typedef struct
{
    uint32_t  borderMin;
    uint32_t  borderMax;

    float     accelRatio;

    float     speedMin;
    float     speedMax;

}T_stepper_cfg;

typedef struct
{
    uint32_t  position;
    uint32_t  positionAcc;

    uint32_t  accelStop;
    uint32_t  accelStart;

    uint32_t  routeMax;
    uint32_t* routeTimer;

}T_stepper_route;

typedef struct
{
    T_stepper_ctl       control;
    T_stepper_cfg       config;
    T_stepper_status    status;
    T_stepper_route     route;

}T_stepper_obj;

#endif
#endif