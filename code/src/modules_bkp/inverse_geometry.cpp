#include "include/inverse_geometry.h"


InverseGeometry::InverseGeometry()
{
    return;
}


bool InverseGeometry::compute_angle(position_t pos_i, float *q_i)
{
    // 
    float d, theta;
    d = BASE_RADIUS - pos_i.y - EE_RADIUS;
    theta = abs(atan2(pos_i.z, d));

    // 
    float a, c, lambda;
    a = FOREARM_LENGTH*FOREARM_LENGTH - pos_i.x*pos_i.x;        // projecton of forearm onto zy plane
    c = d*d + pos_i.z*pos_i.z;                                  // distance between base and ee joint
    lambda = abs((BICEPS_LENGTH*BICEPS_LENGTH + c - a) /        // acos(lambda) 
        (2 * BICEPS_LENGTH * sqrt(c)));

    if ( lambda > 1 ) {
        return false;   // no solution found
    }

    *q_i = theta + acos(lambda) - M_PI;

    // check joint limits
    if (*q_i <= JOINT_LIMIT_MIN || *q_i >= JOINT_LIMIT_MAX) {
        return false;
    }

    return true;        // solution found
}


/**
 *  @brief Inverse geometry of a 3-DOF delta robot.
 *  @param  *pos Pointer to desired end-effector position.
 *  @param  *joints Pointer to respective joints position.
 *  @return true if position is feasable, false otherwise.
 *
 * This function implements a ligth-weight inverse geometry algorithm of a delta robot
 * with equations derived from simple angle relationships and projections. Therefore, no 
 * trigonometric systems of equations are solved.
 */
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

    // chain 4
    joints->q4 = pos->k;

    // time stamp
    joints->t = pos->t;

    return true;    // no errors encountered
}