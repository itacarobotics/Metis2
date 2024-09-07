#include "include/inverse_geometry.h"


InverseGeometry::InverseGeometry(float r1, float l1, float l2, float r2)
{
    this->r1 = r1;
    this->l1 = l1;
    this->l2 = l2;
    this->r2 = r2;
    return;
}


bool InverseGeometry::compute_angle(position_t pos_i, float *q_i)
{
    // 
    float d, theta;
    d = r1 - pos_i.y - r2;
    theta = abs(atan2(pos_i.z, d));

    // 
    float a, c, lambda;
    a = l2*l2 - pos_i.x*pos_i.x;      // projecton of forearm onto zy plane
    c = d*d + pos_i.z*pos_i.z;                          // distance between base and ee joint
    lambda = abs((l1*l1 + c - a) /           // acos(lambda) 
        (2 * l1 * sqrt(c)));      

    if ( lambda > 1 ) {
        return false;   // no solution found
    }

    *q_i = theta + acos(lambda) - M_PI;

    return true;        // solution found
}



bool InverseGeometry::inverse_geometry(position_t *pos, joints_t *joints)
{
    position_t pos_ = *pos;     // need a copy, so the og does not get changed
    bool rc;

    // chain 1
    rc = compute_angle(pos_, &joints->q1);
    if (!rc) {
        return false;       // couldn't find a solution
    }
    
    // chain 2
    pos_.x = (pos->x * COS120) - (pos->y * SIN120);
    pos_.y = (pos->x * SIN120) + (pos->y * COS120);
    rc = compute_angle(pos_, &joints->q2);
    if (!rc) {
        return false;       // couldn't find a solution
    }

    // chain 3
    pos_.x = (pos->x * COS240) - (pos->y * SIN240);
    pos_.y = (pos->x * SIN240) + (pos->y * COS240);
    rc = compute_angle(pos_, &joints->q3);
    if (!rc) {
        return false;       // couldn't find a solution
    }

    return true;    // no errors encountered
}