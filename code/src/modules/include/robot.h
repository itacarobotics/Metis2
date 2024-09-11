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



////////////////////////////////////////////////////////////////////////////////
// Common macros
////////////////////////////////////////////////////////////////////////////////

// robot's kinematics properties
#define BASE_RADIUS                     40      // [ mm ]
#define BICEPS_LENGTH                   80      // [ mm ]
#define FOREARM_LENGTH                  160     // [ mm ]
#define EE_RADIUS                       25      // [ mm ]

#define JOINT_LIMIT_MIN                -999
#define JOINT_LIMIT_MAX                 999

#define POSITION_HOME_OFFSET_Z         -80

// robot's dynamics properties
#define MAX_LINEAR_VEL                  500     // [ mm/s ]
#define MAX_LINEAR_ACC                  2000    // [ mm/s2 ]
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

    SOFT_BREAK,         // the goal position is reached and then break
    HARD_BREAK,         // it stops with motors enabled
    EMERGENCY_STOP      // it stops with motors disabled
};

enum rc_status_t
{
    // buffer
    BUFFER_EMPTY,
    BUFFER_FULL,
    BUFFER_OK,

    // motion planner
    VALID_TRAJECTORY,
    NOT_VALID_TRAJECTORY,
    END_OF_TRAJECTORY,

    VALID_VIA_POINT,
    NOT_VALID_VIA_POINT,

    NOT_VALID_TRAVEL_TIME,
};

    
////////////////////////////////////////////////////////////////////////////////
// Public (global) externs
////////////////////////////////////////////////////////////////////////////////

extern enum robot_status_t robot_status;


////////////////////////////////////////////////////////////////////////////////
// Public (global) function declarations
////////////////////////////////////////////////////////////////////////////////



#endif // _ROBOT_H_