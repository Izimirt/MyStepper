#pragma once

#include <Arduino.h>

#define NO_DISTANCE (step_t*)(1)
#define NO_POINT (point_t*)(1)

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
} accel_t;

typedef struct 
{
    accel_t* startAccel;
    accel_t* finishAccel;
} move_t;

typedef struct
{
    uint32_t steps = 0;
    uint32_t understeps = 0;
    uint32_t oversteps = 0;
} step_t;

typedef struct pnt
{
    int32_t point = 0;
    int32_t understeps = 0;
    int32_t oversteps = 0;
    bool noUndersteps = false;

    uint16_t pointNumber;
    pnt* ptrOnPrev = nullptr;
} point_t;

typedef struct br
{
    uint32_t time_ms = 0;
    uint32_t steps = 0;
    uint8_t speed = 0;

    br* ptrOnNext = nullptr;
    br* ptrOnPrev = nullptr;
} brake_t;