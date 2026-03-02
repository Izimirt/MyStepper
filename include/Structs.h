#pragma once

#include <Arduino.h>

#define NO_DISTANCE (st_step_t*)(1)
#define NO_POINT (st_point_t*)(1)

typedef struct
{
    uint16_t bgnSpeed;
    uint16_t dstSpeed;
    uint32_t time_ms;
    uint32_t period_us;
    float bgnNumStepsPerPeriod;
    float dstNumStepsPerPeriod;
    float stepNumSteps;
    uint16_t numPeriods;
} st_accel_t;

typedef struct 
{
    st_accel_t* startAccel = nullptr;
    st_accel_t* finishAccel = nullptr;
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
    uint32_t time_us = 0;
    float steps = 0;
    uint16_t speed = 0;

    br* ptrOnNext = nullptr;
    br* ptrOnPrev = nullptr;
} st_brake_t;