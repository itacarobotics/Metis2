/*
 * @brief Example template.
 *
 * MIT License
 * 
 * Copyright (c) 2024 Federico Osti
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */



#include "inverse_geometry.h"

////////////////////////////////////////////////////////////////////////////////
// Common macros
////////////////////////////////////////////////////////////////////////////////

// some math constants used in this class
#define M_PI		3.14159	    /* pi */
#define TAN30       0.57735
#define SIN120      0.86603
#define COS120     -0.5
#define SIN240     -0.86603
#define COS240     -0.5


////////////////////////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// Private (static) function declarations
////////////////////////////////////////////////////////////////////////////////

static int32_t ig_compute_angle(float x, float y, float z, float *q_i);

////////////////////////////////////////////////////////////////////////////////
// Private (static) variables
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// Public (global) variables and externs
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// Public (global) functions
////////////////////////////////////////////////////////////////////////////////

/**
 *  @brief Inverse geometry of a 3-DOF delta robot.
 *  @param  *pos Pointer to desired end-effector position.
 *  @return true if position is feasable, false otherwise.
 *
 * This function implements a ligth-weight inverse geometry algorithm of a delta robot
 * with equations derived from simple angle relationships and projections. Therefore, no 
 * trigonometric systems of equations are solved.
 */
int32_t ig_get_inverse_geometry(gcode_t *pos)
{
    int32_t rc;

    float x;
    float y;
    float z;

    // chain 1
    x = pos->data.x;
    y = pos->data.y;
    z = pos->data.z;

    rc = ig_compute_angle(x, y, z, &pos->data.q1);
    if (rc < 0) {
        return rc;       // couldn't find a solution
    }
    
    // chain 2
    x = (pos->data.x * COS120) - (pos->data.y * SIN120);
    y = (pos->data.x * SIN120) + (pos->data.y * COS120);
    z = pos->data.z;

    rc = ig_compute_angle(x, y, z, &pos->data.q2);
    if (rc < 0) {
        return rc;       // couldn't find a solution
    }

    // chain 3
    x = (pos->data.x * COS240) - (pos->data.y * SIN240);
    y = (pos->data.x * SIN240) + (pos->data.y * COS240);
    z = pos->data.z;

    rc = ig_compute_angle(x, y, z, &pos->data.q3);
    if (rc < 0) {
        return rc;       // couldn't find a solution
    }

    // chain 4
    pos->data.q4 = pos->data.k;

    return 0;    // no errors encountered
}


////////////////////////////////////////////////////////////////////////////////
// Private (static) functions
////////////////////////////////////////////////////////////////////////////////

static int32_t ig_compute_angle(float x, float y, float z, float *q_i)
{
    // 
    float d, theta;
    d = BASE_RADIUS - y - EE_RADIUS;
    theta = fabs(atan2(z, d));

    // 
    float a, c, lambda;
    a = FOREARM_LENGTH*FOREARM_LENGTH - x*x;        // projecton of forearm onto zy plane
    c = d*d + z*z;                                  // distance between base and ee joint
    lambda = fabs((BICEPS_LENGTH*BICEPS_LENGTH + c - a) /        // acos(lambda) 
        (2 * BICEPS_LENGTH * sqrt(c)));

    if ( lambda > 1 ) {
        return MP_ERR_WS_LIMIT;   // no solution found
    }

    *q_i = theta + acos(lambda) - M_PI;

    // check joint limits
    if (*q_i <= JOINT_LIMIT_MIN || *q_i >= JOINT_LIMIT_MAX) {
        return MP_ERR_JOINT_LIMIT;
    }

    return 0;        // solution found
}