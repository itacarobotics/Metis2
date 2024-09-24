#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H


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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "gcode.h"
#include "robot.h"
#include "errors.h"


////////////////////////////////////////////////////////////////////////////////
// Common macros
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////////////////////////

typedef struct
{
    gcode_data_t pos_home;

    float via_points_time_step;
    
    float max_linear_vel;
    float max_linear_acc;
    float max_rot_vel;
    float max_rot_acc;
} tg_cfg_t;

////////////////////////////////////////////////////////////////////////////////
// Public (global) externs
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Public (global) function declarations
////////////////////////////////////////////////////////////////////////////////

int32_t tg_init(tg_cfg_t *tg_cfg_);
void tg_start(void);
void tg_reset(void);

int32_t tg_get_dft_cfg(tg_cfg_t *tg_cfg);

int32_t tg_set_next_trajectory(gcode_t pos_goal);
int32_t tg_get_via_point(gcode_t *pos);

int32_t tg_set_positioning(gcode_cmd_t cmd);
int32_t tg_set_home(void);


#endif // TRAJECTORY_GENERATOR_H