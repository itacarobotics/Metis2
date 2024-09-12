#ifndef _ROBOT_H_
#define _ROBOT_H_


/*
 * @brief Robot's important parameters.
 *
 * See implementation file for information about this module.
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


#include "gcode.h"

////////////////////////////////////////////////////////////////////////////////
// Common macros
////////////////////////////////////////////////////////////////////////////////

// robot's kinematics properties
#define BASE_RADIUS                     60      // [ mm ]
#define BICEPS_LENGTH                   80      // [ mm ]
#define FOREARM_LENGTH                  120     // [ mm ]
#define EE_RADIUS                       20      // [ mm ]

#define JOINT_LIMIT_MIN                -999
#define JOINT_LIMIT_MAX                 999

#define POSITION_HOME_OFFSET_Z         -80

#define VIA_POINTS_TIME_STEP            0.01

// robot's dynamics properties
#define MAX_LINEAR_VEL                  800     // [ mm/s ]
#define MAX_LINEAR_ACC                  20000    // [ mm/s2 ]
#define MAX_ROTATION_VEL                20      // [ rad/s ]
#define MAX_ROTATION_ACC                50      // [ rad/s2 ]


////////////////////////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////////////////////////

enum robot_status_t
{
    RUN,
    IDLE,
    DWELL,
    ERROR,
    NOT_CALIBRATED,

    SOFT_BREAK,         // the goal position is reached and then break
    HARD_BREAK,         // it stops with motors enabled
    EMERGENCY_STOP      // it stops with motors disabled
};

struct robot_info_t
{
    enum    robot_status_t  status;
    struct  gcode_data_t    pos;
};

////////////////////////////////////////////////////////////////////////////////
// Public (global) externs
////////////////////////////////////////////////////////////////////////////////

extern struct robot_info_t  robot_info;

////////////////////////////////////////////////////////////////////////////////
// Public (global) function declarations
////////////////////////////////////////////////////////////////////////////////



#endif // _ROBOT_H_