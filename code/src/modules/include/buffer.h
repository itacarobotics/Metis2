#ifndef _BUFFER_H_
#define _BUFFER_H_


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

#include "gcode.h"
#include "module.h"
#include "robot.h"


////////////////////////////////////////////////////////////////////////////////
// Common macros
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////////////////////////


struct bfr_gcode_t
{
    struct gcode_t         *data;
    enum gcode_cmd_t        next_cmd;
    uint32_t                head_idx;
    uint32_t                tail_idx;
    uint32_t                size;
    uint32_t                length;
    uint16_t                mutex;      // semaphore to ensure syncronisation
};


////////////////////////////////////////////////////////////////////////////////
// Public (global) externs
////////////////////////////////////////////////////////////////////////////////

extern struct bfr_gcode_t bfr_gcode_cmds;
extern struct bfr_gcode_t bfr_robot_cmds;

////////////////////////////////////////////////////////////////////////////////
// Public (global) function declarations
////////////////////////////////////////////////////////////////////////////////

int32_t bfr_init(struct bfr_gcode_t *buffer, uint32_t buffer_size);
int32_t bfr_reset(struct bfr_gcode_t *buffer);

int32_t bfr_produce(struct bfr_gcode_t *buffer, struct gcode_t item);
int32_t bfr_consume(struct bfr_gcode_t *buffer, struct gcode_t *item);

int32_t bfr_get_next_cmd(struct bfr_gcode_t *buffer, enum gcode_cmd_t *cmd);


#endif // _BUFFER_H_