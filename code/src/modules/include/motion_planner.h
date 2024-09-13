#ifndef _MOTION_PLANNER_H_
#define _MOTION_PLANNER_H_


/*
 * @brief Example template.
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


#include "trajectory_generator.h"
#include "inverse_geometry.h"
#include "buffer.h"
#include "robot.h"


////////////////////////////////////////////////////////////////////////////////
// Common macros
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////////////////////////


/**
 * @param    
    END_OF_TRAJECTORY,
    COMPUTING_VIA_POINTS,
    NOT_CALIBRATED,

    NOT_VALID_VIA_POINT,
    NOT_VALID_TRAJECTORY,
    NOT_VALID_TRAVEL_TIME,
    
    VALID_TRAJECTORY,
    VALID_VIA_POINT,

    VALID_IG_SOLUTION,
    VIA_POINT_OUT_OF_WORKSPACE,
    JOINT_LIMIT_EXCEDED,

    BUFFER_ROBOT_CMDS_FULL
 */
enum mp_status_t 
{
    MP_IDLE,
    MP_RUN,
    MP_ERROR,
    MP_WAIT_PRODUCE,
};


////////////////////////////////////////////////////////////////////////////////
// Public (global) externs
////////////////////////////////////////////////////////////////////////////////

extern enum mp_status_t mp_status;

////////////////////////////////////////////////////////////////////////////////
// Public (global) function declarations
////////////////////////////////////////////////////////////////////////////////

int32_t mp_init();
int32_t mp_start();
int32_t mp_run();


#endif // _MOTION_PLANNER_H_