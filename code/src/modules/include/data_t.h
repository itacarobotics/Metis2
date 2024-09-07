#ifndef DATA_T_H
#define DATA_T_H

#include "stdint.h"

struct joints_t
{
    float q1;
    float q2;
    float q3;
    float q4;
};

struct position_t
{
    float x;
    float y;
    float z;
    float k;    // rotation about z axis
    float t;    // time stamp
};


#endif  // DATA_T_H