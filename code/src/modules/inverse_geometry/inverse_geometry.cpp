#include "inverse_geometry.h"


bool compute_angle(position_t pos_i, float *q_i)
{
    float d, theta;
    d = BASE_RADIUS - pos_i.y - EE_RADIUS;
    theta = abs(atan2(pos_i.z, d));

    float a, c, k;
    a = FOREARM_LEN*FOREARM_LEN - pos_i.x*pos_i.x;
    c = d*d + pos_i.z*pos_i.z;
    k = abs((BICEPS_LEN*BICEPS_LEN + c - a) / (2 * BICEPS_LEN * sqrt(c)));

    if ( k > 1 ) {
        return false;   // no solution found
    }

    *q_i = theta + acos(k) - PI;

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