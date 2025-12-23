#pragma once

typedef enum
{
    UNDERSTEP = 0b1,
    OVERSTEP = 0b10,
    UNEXPECTED_ENTER_IN_FUNCTION = 0b11,
    CANT_COUNT_BRAKE = 0b100,
    INCORRECT_MOVE_PARAMETERS = 0b101,
    UNKNOWN_DISTANCE = 0b110,
    UNAVAILABLE_POINT = 0b111
} st_err_t;

typedef enum
{
    START,
    ACCELERATION_AND_GO,
    DECELERATION
} st_phase_t;

typedef enum
{
    BWD,
    FWD
} st_dir_t;