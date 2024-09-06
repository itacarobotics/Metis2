/*
    @author William Bailes <williambailes@gmail.com> http://tinkersprojects.com/
*/


#ifndef INVERSE_GEOMETRY_H
#define INVERSE_GEOMETRY_H

#include "../robot_description/robot_description.h"
#include "math.h"

#define PI          3.14159

#define TAN30       0.57735
#define SIN120      0.86603
#define COS120      -0.5


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
};


bool inverse_geometry(position_t *pos, joints_t *joints);
bool compute_angle(position_t pos_i, float *q_i);



#endif  // INVERSE_GEOMETRY_H