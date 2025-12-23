#pragma once

#include <Arduino.h>

#define NO_DISTANCE (st_step_t*)(1)
#define NO_POINT (st_point_t*)(1)

typedef struct
{
    uint8_t bgnSpeed;
    uint8_t dstSpeed;
    uint32_t time_ms;
    uint32_t period_ms;
    uint32_t period_us;
    uint16_t bgnNumStepsPerPeriod;
    uint16_t dstNumStepsPerPeriod;
    int16_t stepNumSteps;
    uint8_t numPeriods;
} st_accel_t;

typedef struct 
{
    st_accel_t* startAccel;
    st_accel_t* finishAccel;
} st_move_t;

typedef struct
{
    uint32_t steps = 0;
    uint32_t understeps = 0;
    uint32_t oversteps = 0;
} st_step_t;

typedef struct pnt
{
    int32_t point = 0;
    int32_t understeps = 0;
    int32_t oversteps = 0;
    bool noUndersteps = false;

    uint16_t pointNumber;
    pnt* ptrOnPrev = nullptr;
} st_point_t;

typedef struct br
{
    uint32_t time_ms = 0;
    uint32_t steps = 0;
    uint8_t speed = 0;

    br* ptrOnNext = nullptr;
    br* ptrOnPrev = nullptr;
} st_brake_t;