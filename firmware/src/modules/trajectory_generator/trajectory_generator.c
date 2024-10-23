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
    RELATIVE,
    NOT_CALIBRATED,
} positioning_t;

typedef struct
{
    tg_cfg_t cfg;
    positioning_t positioning;

    // a0 = 0, a1 = 0, a2 = 0
    float a3; 
    float a4;
    float a5;

    gcode_t pos_start;
    gcode_t pos_end;
    float travel_time;

    float via_point_idx;      // a value [0, 1]
    float via_point_time;     // a value [0, T]

} tg_state_t;


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

static tg_state_t st;

////////////////////////////////////////////////////////////////////////////////
// Public (global) variables and externs
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Public (global) functions
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initialize tragectory generator instance.
 *
 * @param[in] cfg The tg configuration. (FUTURE)
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * This function initializes a tg module instance. Generally, it should not
 * access other modules as they might not have been initialized yet.  An
 * exception is the log module.
 */
int32_t tg_init(tg_cfg_t *tg_cfg)
{
    if (tg_cfg == NULL) {
        return MOD_ERR_BAD_ARGUMENT;
    }

    memset(&st, 0, sizeof(st));
    st.cfg = *tg_cfg;   // import configuration
    
    st.pos_start.data.z = st.cfg.pos_home.z; // set home position
    st.positioning = NOT_CALIBRATED;

    return MOD_RET_OK;
}

void tg_start(void)
{
    return;
}

void tg_reset(void)
{
    return;
}


int32_t tg_get_dft_cfg(tg_cfg_t *tg_cfg)
{
    memset(&tg_cfg->pos_home, 0, sizeof(tg_cfg->pos_home));
    tg_cfg->pos_home.z              = -50;  // [ mm ]
    
    tg_cfg->via_points_time_step    = 0.01; // [ s ]
    tg_cfg->max_linear_vel          = 600;  // [ mm/s ]
    tg_cfg->max_linear_acc          = 2000; // [ mm/s2 ]
    tg_cfg->max_rot_vel             = 20;   // [ rad/s ]
    tg_cfg->max_rot_acc             = 100;  // [ rad/s2 ]
    return MOD_RET_OK;
}



int32_t tg_set_next_trajectory(gcode_t pos_goal)
{
    int32_t rc;

    // update module's attributes
    switch (st.positioning)
    {
    case ABSOLUTE:
        st.pos_start      = st.pos_end;  // previous pos_end becomes new pos_start
        st.pos_end        = pos_goal;
        break;

    case RELATIVE:
        st.pos_start      = st.pos_end;  // previous pos_end becomes new pos_start
        st.pos_end.data.x = st.pos_start.data.x + pos_goal.data.x;
        st.pos_end.data.y = st.pos_start.data.y + pos_goal.data.y;
        st.pos_end.data.z = st.pos_start.data.z + pos_goal.data.z;
        st.pos_end.data.i = st.pos_start.data.i + pos_goal.data.i;
        st.pos_end.data.j = st.pos_start.data.j + pos_goal.data.j;
        st.pos_end.data.k = st.pos_start.data.k + pos_goal.data.k;
        break;

    case NOT_CALIBRATED:
        return MP_ERR_NOT_CALIBRATED;

    default:
        break;
    }
    
    // reset attributes
    st.via_point_idx   = 0;
    st.via_point_time  = 0;


    switch (st.pos_end.cmd)
    {
    case G01:
        rc = tg_set_next_trajectory__linear_interp(&st.pos_start, &st.pos_end);
        break;
    
    case G02:
        rc = tg_set_next_trajectory__arc_cv_interp(&st.pos_start, &st.pos_end);
        break;
    
    case G03:
        rc = tg_set_next_trajectory__arc_ccw_interp(&st.pos_start, &st.pos_end);
        break;
    
    default:
        break;
    }

    return rc;
}


int32_t tg_get_via_point(gcode_t *pos)
{
    int32_t rc;

    switch (st.pos_end.cmd)
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


int32_t tg_set_positioning(gcode_cmd_t cmd)
{
    if (st.positioning == NOT_CALIBRATED) {
        return MP_ERR_NOT_CALIBRATED;
    }

    switch (cmd)
    {
    case G90:
        st.positioning = ABSOLUTE;
        break;
    case G91:
        st.positioning = RELATIVE;
        break;
    default:
        break;
    }

    return MOD_RET_OK;
}

int32_t tg_set_home(void) 
{
    st.pos_start.data = st.cfg.pos_home;
    st.positioning = ABSOLUTE;
    return MOD_RET_OK;
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
                            st.cfg.max_linear_vel, st.cfg.max_linear_acc);
        // override if input value is greater
        path_travel_time = fmax(pos_end->data.t, path_travel_time);
    }

    // end effector rotation
    float rot_length        = 0;
    float rot_travel_time   = 0;

    rot_length = abs(pos_end->data.k - pos_start->data.k);
    if (rot_length != 0) {
        rot_travel_time = tg_get_linear_constrained_time(rot_length, 
                            st.cfg.max_rot_vel, st.cfg.max_rot_acc);
        // override if input value is greater
        rot_travel_time = fmax(pos_end->data.t, rot_travel_time);
    }


    // travel time that satisfies vel and acc constraints
    st.travel_time = fmax(path_travel_time, rot_travel_time);
    if (st.travel_time == 0) {
        return MP_ERR_BAD_TRAVEL_TIME;
    }

    // Quintic polynomial coefficents, used for velocity profile
    // a0 = 0;
    // a1 = 0;
    // a2 = 0;
    st.a3 =   10 / pow(st.travel_time, 3);
    st.a4 =  -15 / pow(st.travel_time, 4);
    st.a5 =    6 / pow(st.travel_time, 5);

    return MOD_RET_OK;
}

static int32_t tg_set_next_trajectory__arc_cv_interp(gcode_t *pos_start, 
                                          gcode_t *pos_end)
{
    return 0;
}

static int32_t tg_set_next_trajectory__arc_ccw_interp(gcode_t *pos_start, 
                                          gcode_t *pos_end)
{
    return 0;
}



static int32_t tg_get_via_point__linear_interp(gcode_t *pos)
{
    // mapping position index with quintic polynomial profile
    // a0 = 0; a1 = 0; a2 = 0;
    st.via_point_idx  = st.a3 * pow(st.via_point_time, 3);
    st.via_point_idx += st.a4 * pow(st.via_point_time, 4);
    st.via_point_idx += st.a5 * pow(st.via_point_time, 5);

    if (st.via_point_idx > 1) {
        return MP_END_OF_TRAJECTORY;           // end point reached
    }

    // end point is almost reached
    if ((st.via_point_time + st.cfg.via_points_time_step) > st.travel_time) {
        st.via_point_idx    = 1;           // avoid floating point errors
        st.via_point_time   = st.travel_time;
    }

    // mapping end-effector position
    pos->data.x =   ((1 - st.via_point_idx) * st.pos_start.data.x) + 
                    (st.via_point_idx * st.pos_end.data.x);
    pos->data.y =   ((1 - st.via_point_idx) * st.pos_start.data.y) + 
                    (st.via_point_idx * st.pos_end.data.y);
    pos->data.z =   ((1 - st.via_point_idx) * st.pos_start.data.z) + 
                    (st.via_point_idx * st.pos_end.data.z);
    pos->data.k =   ((1 - st.via_point_idx) * st.pos_start.data.k) + 
                    (st.via_point_idx * st.pos_end.data.k);
    pos->data.t = st.via_point_time;   // via point time stamp

    // update attribute
    st.via_point_time += st.cfg.via_points_time_step;
    
    return MOD_RET_OK;
}

static int32_t tg_get_via_point__arc_cw_interp(gcode_t *pos)
{
    return 0;
}

static int32_t tg_get_via_point__arc_ccw_interp(gcode_t *pos)
{
    return 0;
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
