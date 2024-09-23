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


#include "trajectory_generator.h"


////////////////////////////////////////////////////////////////////////////////
// Common macros
////////////////////////////////////////////////////////////////////////////////

// some math constants
#define SQRT_10     3.16228
#define SQRT2_3     1.31607

////////////////////////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////////////////////////

typedef enum
{
    ABSOLUTE,
    RELATIVE
} positioning_t;

////////////////////////////////////////////////////////////////////////////////
// Private (static) function declarations
////////////////////////////////////////////////////////////////////////////////

static int32_t tg_set_next_trajectory__linear_interp(gcode_t *_pos_start, 
                                                    gcode_t *_pos_end);
static int32_t tg_set_next_trajectory__arc_cv_interp(gcode_t *_pos_start, 
                                                    gcode_t *_pos_end);
static int32_t tg_set_next_trajectory__arc_ccw_interp(gcode_t *_pos_start, 
                                                    gcode_t *_pos_end);

static int32_t tg_get_via_point__linear_interp(gcode_t *pos);
static int32_t tg_get_via_point__arc_cw_interp(gcode_t *pos);
static int32_t tg_get_via_point__arc_ccw_interp(gcode_t *pos);

static float tg_get_linear_len(gcode_t *pos_start, gcode_t *pos_end);
static float tg_get_arc_len(gcode_t *pos_start, gcode_t *pos_end);

static float tg_get_linear_constrained_time(float path_length, float max_vel, 
                                            float max_acc);
static float tg_get_arc_constrained_time(float path_length, float max_vel, 
                                         float max_acc);


////////////////////////////////////////////////////////////////////////////////
// Private (static) variables
////////////////////////////////////////////////////////////////////////////////

// parameters of quintic polynomial
static float        a3, a4, a5; // a0 = 0, a1 = 0, a2 = 0

static gcode_t      pos_start;
static gcode_t      pos_end;
static float        travel_time;

static float        via_point_idx;      // a value [0, 1]
static float        via_point_time;     // a value [0, T]

static positioning_t positioning;

////////////////////////////////////////////////////////////////////////////////
// Public (global) variables and externs
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Public (global) functions
////////////////////////////////////////////////////////////////////////////////

void tg_init(void)
{
    return;
}

void tg_start(void)
{
    return;
}

void tg_reset(void)
{
    return;
}



int32_t tg_set_next_trajectory(gcode_t pos_goal)
{
    int32_t rc;

    // update module's attributes
    switch (positioning)
    {
    case ABSOLUTE:
        pos_start      = pos_end;  // previous pos_end becomes new pos_start
        pos_end        = pos_goal;
        break;

    case RELATIVE:
        pos_start      = pos_end;  // previous pos_end becomes new pos_start
        pos_end.data.x = pos_start.data.x + pos_goal.data.x;
        pos_end.data.y = pos_start.data.y + pos_goal.data.y;
        pos_end.data.z = pos_start.data.z + pos_goal.data.z;
        pos_end.data.i = pos_start.data.i + pos_goal.data.i;
        pos_end.data.j = pos_start.data.j + pos_goal.data.j;
        pos_end.data.k = pos_start.data.k + pos_goal.data.k;
        break;
    
    default:
        break;
    }
    
    // reset attributes
    via_point_idx   = 0;
    via_point_time  = 0;


    switch (pos_end.cmd)
    {
    case G01:
        rc = tg_set_next_trajectory__linear_interp(&pos_start, &pos_end);
        break;
    
    case G02:
        rc = tg_set_next_trajectory__arc_cv_interp(&pos_start, &pos_end);
        break;
    
    case G03:
        rc = tg_set_next_trajectory__arc_ccw_interp(&pos_start, &pos_end);
        break;
    
    default:
        break;
    }

    return rc;
}


int32_t tg_get_via_point(gcode_t *pos)
{
    int32_t rc;

    switch (pos_end.cmd)
    {
    case G01:
        rc = tg_get_via_point__linear_interp(pos);
        break;
    
    case G02:
        rc = tg_get_via_point__arc_cw_interp(pos);
        break;
    
    case G03:
        rc = tg_get_via_point__arc_ccw_interp(pos);
        break;

    default:
        break;
    }
    
    return rc;
}


void tg_set_positioning(gcode_cmd_t cmd)
{
    switch (cmd)
    {
    case G90:
        positioning = ABSOLUTE;
        break;
    case G91:
        positioning = RELATIVE;
        break;
    default:
        break;
    }
    return;
}


////////////////////////////////////////////////////////////////////////////////
// Private (static) functions
////////////////////////////////////////////////////////////////////////////////

static int32_t tg_set_next_trajectory__linear_interp(gcode_t *pos_start, 
                                                    gcode_t *pos_end)
{
    int32_t rc;

    // end effector position
    float path_length       = 0;
    float path_travel_time  = 0;

    path_length = tg_get_linear_len(pos_start, pos_end);
    if (path_length != 0) {
        path_travel_time = tg_get_linear_constrained_time(path_length, 
                            MAX_LINEAR_VEL, MAX_LINEAR_ACC);
        // override if input value is greater
        path_travel_time = fmax(pos_end->data.t, path_travel_time);
    }

    // end effector rotation
    float rot_length        = 0;
    float rot_travel_time   = 0;

    rot_length = abs(pos_end->data.k - pos_start->data.k);
    if (rot_length != 0) {
        rot_travel_time = tg_get_linear_constrained_time(rot_length, 
                            MAX_ROTATION_VEL, MAX_ROTATION_ACC);
        // override if input value is greater
        rot_travel_time = fmax(pos_end->data.t, rot_travel_time);
    }


    // travel time that satisfies vel and acc constraints
    travel_time = fmax(path_travel_time, rot_travel_time);
    if (travel_time == 0) {
        return MP_ERR_BAD_TRAVEL_TIME;
    }

    // Quintic polynomial coefficents, used for velocity profile
    // a0 = 0;
    // a1 = 0;
    // a2 = 0;
    a3 =   10 / pow(travel_time, 3);
    a4 =  -15 / pow(travel_time, 4);
    a5 =    6 / pow(travel_time, 5);

    return MOD_RET_OK;
}

static int32_t tg_set_next_trajectory__arc_cv_interp(gcode_t *pos_start, 
                                          gcode_t *pos_end)
{
    return MP_ERR_BAD_TRAJECTORY;
}

static int32_t tg_set_next_trajectory__arc_ccw_interp(gcode_t *pos_start, 
                                          gcode_t *pos_end)
{
    return MP_ERR_BAD_TRAJECTORY;
}



static int32_t tg_get_via_point__linear_interp(gcode_t *pos)
{
    // mapping position index with quintic polynomial profile
    // a0 = 0; a1 = 0; a2 = 0;
    via_point_idx  = a3 * pow(via_point_time, 3);
    via_point_idx += a4 * pow(via_point_time, 4);
    via_point_idx += a5 * pow(via_point_time, 5);

    if (via_point_idx > 1) {
        return MP_ERR_END_OF_TRAJECTORY;           // end point reached
    }

    // end point is almost reached
    if ((via_point_time + VIA_POINTS_TIME_STEP) > travel_time) {
        via_point_idx    = 1;           // avoid floating point errors
        via_point_time   = travel_time;
    }

    // mapping end-effector position
    pos->data.x =   ((1 - via_point_idx) * pos_start.data.x) + 
                    (via_point_idx * pos_end.data.x);
    pos->data.y =   ((1 - via_point_idx) * pos_start.data.y) + 
                    (via_point_idx * pos_end.data.y);
    pos->data.z =   ((1 - via_point_idx) * pos_start.data.z) + 
                    (via_point_idx * pos_end.data.z);
    pos->data.k =   ((1 - via_point_idx) * pos_start.data.k) + 
                    (via_point_idx * pos_end.data.k);
    pos->data.t = via_point_time;   // via point time stamp

    // update attribute
    via_point_time += VIA_POINTS_TIME_STEP;
    
    return MOD_RET_OK;
}

static int32_t tg_get_via_point__arc_cw_interp(gcode_t *pos)
{
    return MP_ERR_BAD_VIA_POINT;
}

static int32_t tg_get_via_point__arc_ccw_interp(gcode_t *pos)
{
    return MP_ERR_BAD_VIA_POINT;
}



static float tg_get_linear_len(gcode_t *pos_start, gcode_t *pos_end)
{
    float dx, dy, dz;
    
    dx = pos_end->data.x - pos_start->data.x;
    dy = pos_end->data.y - pos_start->data.y;
    dz = pos_end->data.z - pos_start->data.z;
    
    return sqrt((dx*dx + dy*dy + dz*dz));   // pytagoras theorem
}

static float tg_get_arc_len(gcode_t *pos_start, gcode_t *pos_end)
{
    return -1;
}


static float tg_get_linear_constrained_time(float path_length, float max_vel, 
                                            float max_acc)
{
    float time_vel_constrained;
    time_vel_constrained = (15 * path_length) / (8 * max_vel);

    float time_acc_constrained;
    time_acc_constrained =  (SQRT_10 * sqrt(path_length)) / 
                            (SQRT2_3 * sqrt(max_acc));

    return fmax(time_vel_constrained, time_acc_constrained);
}

static float tg_get_arc_constrained_time(float path_length, float max_vel, 
                                         float max_acc)
{
    return -1;
}