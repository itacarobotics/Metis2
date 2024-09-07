#ifndef INVERSE_GEOMETRY_H
#define INVERSE_GEOMETRY_H

#include "math.h"
#include "data_t.h"

#define TAN30       0.57735
#define SIN120      0.86603
#define COS120      -0.5
#define SIN240      -0.86603
#define COS240      -0.5


class InverseGeometry
{
public:
    InverseGeometry(float r1, float l1, float l2, float r2);
    bool inverse_geometry(position_t *pos, joints_t *joints);

private:
    float r1, l1, l2, r2;
    bool compute_angle(position_t pos_i, float *q_i);
};


#include "../inverse_geometry.cpp"

#endif  // INVERSE_GEOMETRY_H