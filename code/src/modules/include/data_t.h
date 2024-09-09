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
    // float i;    // rotation about x axis     --> i = 0
    // float j;    // rotation about y axis     --> j = 0
    float k;    // rotation about z axis
    float t;    // time stamp
};


enum robot_state_t
{
    
};


#endif  // DATA_T_H