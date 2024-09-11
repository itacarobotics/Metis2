#ifndef INVERSE_GEOMETRY_H
#define INVERSE_GEOMETRY_H

#include "math.h"
#include "data_t.h"
#include "configuration.h"


// some math constants used in this class
#define TAN30       0.57735
#define SIN120      0.86603
#define COS120      -0.5
#define SIN240      -0.86603
#define COS240      -0.5


class InverseGeometry
{
public:
    InverseGeometry();
    bool    inverse_geometry(position_t *pos, joints_t *joints);

private:
    bool    compute_angle(position_t pos_i, float *q_i);
};


#include "../inverse_geometry.cpp"

#endif  // INVERSE_GEOMETRY_H