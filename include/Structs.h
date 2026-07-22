#pragma once

#include <Arduino.h>

#define NO_DISTANCE (st_step_t*)(1)
#define NO_POINT (st_point_t*)(1)

typedef struct
{
    uint32_t s_accel = 0;
    uint32_t s_decel = 0;
} st_accel_t;

typedef struct 
{
    uint32_t v_start_hz =   0;
    uint32_t v_work_hz =    0;
    uint32_t v_fin_hz =     0;
    float a_accel =         0.0f;
    float a_decel =         0.0f;
} st_move_t;

typedef struct
{
    uint32_t s_total =      0;
    uint32_t s_under =      0;
    uint32_t s_over =       0;
} st_dist_t;

typedef struct pnt
{
    int32_t point = 0;
    int32_t s_under = 0;
    int32_t s_over = 0;
} st_point_t;

typedef struct br
{
    uint32_t time_us = 0;
    float steps = 0;
    uint16_t speed = 0;

    br* ptrOnNext = nullptr;
    br* ptrOnPrev = nullptr;
} st_brake_t;

typedef struct TaskMail
{
    st_move_t mv        {};
    st_dist_t dist      {};
    st_accel_t a_dist   {};
} task_mail_t;