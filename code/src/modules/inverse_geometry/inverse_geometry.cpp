#include "inverse_geometry.h"


bool compute_angle(position_t pos_i, float *q_i)
{
    float y_1;
    y_1 = -0.5 * TAN30 * EE_RADIUS;         // f/2 * tan(30 deg)
    pos_i.y -= 0.5 * TAN30 * BASE_RADIUS;   // shift center to edge

    // z = a + b*y
    float aV;
    float bV;
    aV = ((pos_i.x * pos_i.x) + (pos_i.y * pos_i.y) + (pos_i.z * pos_i.z) - (y_1 * y_1) +
        BICEPS_LEN * BICEPS_LEN - FOREARM_LEN * FOREARM_LEN ) / (2.0f * pos_i.z);

    bV = (y_1 - pos_i.y) / pos_i.z;

    // discriminant
    double dV;
    dV = -(aV + bV * y_1) * (aV + bV * y_1) + BICEPS_LEN * (bV * bV * BICEPS_LEN + BICEPS_LEN); 
    if ( dV < 0 ) {
        return false;   // no solution
    }

    double yj = (y_1 - aV * bV - sqrt(dV)) / ( bV * bV + 1); // choosing outer povar
    double zj = aV + bV * yj;
    *q_i = atan2(-zj, (y_1 - yj));

    return true;        // solution found
}



bool inverse_geometry(position_t *pos, joints_t *joints)
{
    position_t pos_ = *pos;     // need a copy, so the og does not get changed
    bool rc;

    // chain 1
    rc = compute_angle(pos_, &joints->q1);
    if (!rc) {
        return false;       // couldn't find a solution
    }
    
    // chain 2
    pos_.x = (pos->x * COS120) + (pos->y * SIN120);
    pos_.y = (pos->y * COS120) - (pos->x * SIN120);
    rc = compute_angle(pos_, &joints->q2);
    if (!rc) {
        return false;       // couldn't find a solution
    }

    // chain 3
    pos_.x = (pos->x * COS120) - (pos->y * SIN120);
    pos_.y = (pos->y * COS120) + (pos->x * SIN120);
    rc = compute_angle(pos_, &joints->q3);
    if (!rc) {
        return false;       // couldn't find a solution
    }

    return true;    // no errors encountered
}