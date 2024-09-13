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


#include "motion_planner.h"


////////////////////////////////////////////////////////////////////////////////
// Common macros
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Private (static) function declarations
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Private (static) variables
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Public (global) variables and externs
////////////////////////////////////////////////////////////////////////////////

enum mp_status_t mp_status;
struct bfr_gcode_t bfr_gcode_cmds;
struct bfr_gcode_t bfr_robot_cmds;
struct robot_info_t robot_info;

////////////////////////////////////////////////////////////////////////////////
// Public (global) functions
////////////////////////////////////////////////////////////////////////////////

int32_t mp_init()
{
    return -1;
}

int32_t mp_start()
{
    return -1;
}

int32_t mp_run()
{
    struct gcode_t *pos;
    enum gcode_cmd_t gcode_next_cmd;
    int32_t rc;

    switch (mp_status)
    {
    case MP_RUN:    // get via point
        rc = tg_get_via_point(pos);
        if (rc == MP_ERR_END_OF_TRAJECTORY) {
            mp_status = MP_IDLE;    // next loop a new trajectory will be set
            return rc;
        }
        
        // compute ig
        rc = ig_inverse_geometry(pos);
        if (rc < 0) {
            return rc;
        }

        // add to bfr_robot_cmds
        rc = bfr_produce(&bfr_robot_cmds, *pos);
        if (rc < 0) {
            mp_status = MP_WAIT_PRODUCE;
            return rc;
        }
        break;
    
    case MP_IDLE:   // set new trajectory
        // ROBOT_SOFT_BREAK does not consume new gcode cmd
        if (robot_info.status == ROBOT_SOFT_BREAK) {
            return RBT_ERR_SB;
        }

        // check if incoming cmd is for motion planning
        rc = bfr_get_next_cmd(&bfr_gcode_cmds, &gcode_next_cmd);
        if (gcode_next_cmd != G01 &&
            gcode_next_cmd != G02 &&
            gcode_next_cmd != G03) {
            return MP_ERR_BAD_ARGUMENT;
        }

        // get new gcode cmd
        rc = bfr_consume(&bfr_gcode_cmds, pos);
        if (rc < 0) {
            return rc;
        }

        // set trajectory
        rc = tg_set_new_trajectory(pos);
        if (rc < 0) {
            return rc;
        }

        mp_status = MP_RUN;     // next time will compute via point
        break;
    
    case MP_WAIT_PRODUCE:
        // add to bfr_robot_cmds
        rc = bfr_produce(&bfr_robot_cmds, *pos);
        if (rc < 0) {
            mp_status = MP_WAIT_PRODUCE;
            return rc;
        }
        mp_status = MP_RUN;
        break;

    default:
        break;
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Private (static) functions
////////////////////////////////////////////////////////////////////////////////


