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

#define VIA_POINTS_TIME_STEP 0.01

////////////////////////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Private (static) function declarations
////////////////////////////////////////////////////////////////////////////////

static enum rc_status_t trajectory_init(struct gcode_t _pos_start, 
                                        struct gcode_t _pos_end);
static enum rc_status_t trajectory_init__G01(struct gcode_t *_pos_start, 
                                            struct gcode_t *_pos_end);
static enum rc_status_t trajectory_init__G02(struct gcode_t *_pos_start, 
                                            struct gcode_t *_pos_end);
static enum rc_status_t trajectory_init__G03(struct gcode_t *_pos_start, 
                                            struct gcode_t *_pos_end);

static enum rc_status_t get_via_point(struct gcode_t *pos);
static enum rc_status_t get_via_point__G01(struct gcode_t *pos);
static enum rc_status_t get_via_point__G02(struct gcode_t *pos);
static enum rc_status_t get_via_point__G03(struct gcode_t *pos);


float get_path_length__G01(struct gcode_t *pos_start, struct gcode_t *pos_end);
float get_best_effort_time__G01(float path_length, float max_vel, float max_acc);



////////////////////////////////////////////////////////////////////////////////
// Private (static) variables
////////////////////////////////////////////////////////////////////////////////

// parameters of quintic polynomial
static float            a3, a4, a5; // a0 = 0, a1 = 0, a2 = 0

static struct gcode_t   pos_current;
static struct gcode_t   pos_start;
static struct gcode_t   pos_end;
static float            travel_time;

static float            via_point_idx;      // a value [0, 1]
static float            via_point_step;     // increment of via_point_idx (not linear)
static float            via_point_time;     // a value [0, T]


////////////////////////////////////////////////////////////////////////////////
// Public (global) variables and externs
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// Public (global) functions
////////////////////////////////////////////////////////////////////////////////

enum rc_status_t motion_planner_init(void)
{

}

enum rc_status_t motion_planner_run(void)
{
    enum rc_status_t rc;
    struct gcode_t pos;

    rc = buffer_is_full(&buffer_robot_cmds);
    if (rc == BUFFER_FULL) {
        return rc;
    }

    rc = get_via_point(&pos);

    if (rc == END_OF_TRAJECTORY) {

        rc = buffer_consume(&buffer_gcode_cmds, &pos);
        if (rc == BUFFER_FULL) {
            return rc;
        }
        rc = trajectory_init(pos_current, pos);
        return rc;
    }

    // compute inverse geometry
    // add to buffer_robot_cmds

}

////////////////////////////////////////////////////////////////////////////////
// Private (static) functions
////////////////////////////////////////////////////////////////////////////////

static enum rc_status_t trajectory_init( struct gcode_t _pos_start, 
                                        struct gcode_t _pos_end)
{
    enum rc_status_t rc;

    // update attributes
    pos_start       = _pos_start;
    pos_end         = _pos_end;
    via_point_idx   = 0;
    via_point_step  = 0;
    via_point_time  = 0;


    switch (pos_end.cmd)
    {
    case G01:
        rc = trajectory_init__G01(&pos_start, &pos_end);
        break;
    
    case G02:
        rc = trajectory_init__G02(&pos_start, &pos_end);
        break;
    
    case G03:
        rc = trajectory_init__G03(&pos_start, &pos_end);
        break;
    
    default:
        break;
    }

    return rc;
}

static enum rc_status_t trajectory_init__G01( struct gcode_t *pos_start, 
                                            struct gcode_t *pos_end)
{
    enum rc_status_t rc;

    // end effector position
    float path_length       = 0;
    float path_travel_time  = 0;

    path_length = get_line_length(&pos_start, &pos_end);
    if (path_length != 0) {
        path_travel_time = get_best_effort_time(path_length, MAX_LINEAR_VEL, MAX_LINEAR_ACC);
        path_travel_time = max(pos_end->data.t, path_travel_time);   // override if input value is greater
    }

    // end effector rotation
    float rot_length        = 0;
    float rot_travel_time   = 0;

    rot_length = abs(pos_end->data.k - pos_start->data.k);
    if (rot_length != 0) {
        rot_travel_time = get_best_effort_time(rot_length, MAX_ROTATION_VEL, MAX_ROTATION_ACC);
        rot_travel_time = max(pos_end->data.t, rot_travel_time);     // override if input value is greater
    }


    // travel time that satisfies vel and acc constraints
    travel_time = max(path_travel_time, rot_travel_time);
    if (travel_time == 0) {
        return NOT_VALID_TRAVEL_TIME;
    }

    // Quintic polynomial coefficents, used for velocity profile
    // a0 = 0;
    // a1 = 0;
    // a2 = 0;
    a3 =   10 / pow(travel_time, 3);
    a4 =  -15 / pow(travel_time, 4);
    a5 =    6 / pow(travel_time, 5);

    return VALID_TRAJECTORY;
}

static enum rc_status_t trajectory_init__G02( struct gcode_t *pos_start, 
                                            struct gcode_t *pos_end)
{
    return NOT_VALID_TRAJECTORY;
}

static enum rc_status_t trajectory_init__G03( struct gcode_t *pos_start, 
                                            struct gcode_t *pos_end)
{
    return NOT_VALID_TRAJECTORY;
}



static enum rc_status_t get_via_point(struct gcode_t *pos)
{
    switch (pos->cmd)
    {
    case G01:
        get_via_point__G01(pos);
        break;
    
    case G02:
        get_via_point__G02(pos);
        break;
    
    case G03:
        get_via_point__G03(pos);
        break;
    
    default:
        break;
    }
}


static enum rc_status_t get_via_point__G01(struct gcode_t *pos)
{
    // mapping position index with quintic polynomial profile
    // a0 = 0; a1 = 0; a2 = 0;
    via_point_idx  = a3 * pow(via_point_time, 3);
    via_point_idx += a4 * pow(via_point_time, 4);
    via_point_idx += a5 * pow(via_point_time, 5);

    if (via_point_idx > 1) {
        return END_OF_TRAJECTORY;           // end point reached
    }

    if ((2*via_point_idx - via_point_step) > 1) {     // end point is almost reached
        via_point_idx    = 1;
        via_point_time   = travel_time;
    }

    // mapping end-effector position
    pos->data.x = ((1 - via_point_idx) * pos_start.data.x) + (via_point_idx * pos_end.data.x);
    pos->data.y = ((1 - via_point_idx) * pos_start.data.y) + (via_point_idx * pos_end.data.y);
    pos->data.z = ((1 - via_point_idx) * pos_start.data.z) + (via_point_idx * pos_end.data.z);
    pos->data.k = ((1 - via_point_idx) * pos_start.data.k) + (via_point_idx * pos_end.data.k);
    pos->data.t = via_point_time;   // via point time stamp

    // update attribute
    via_point_time += VIA_POINTS_TIME_STEP;
    via_point_step = via_point_idx;     // save previous via_point_idx
    return VALID_VIA_POINT;
}

static enum rc_status_t get_via_point__G02(struct gcode_t *pos)
{
    return NOT_VALID_VIA_POINT;
}

static enum rc_status_t get_via_point__G03(struct gcode_t *pos)
{
    return NOT_VALID_VIA_POINT;
}












