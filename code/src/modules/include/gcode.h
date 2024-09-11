#ifndef _GCODE_H_
#define _GCODE_H_


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



////////////////////////////////////////////////////////////////////////////////
// Common macros
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////////////////////////



enum gcode_cmd_t
{
    G01,
    G02,
    G03,
    G04,
    G28,
    G90,
    G91,
    M04,
    M05,
    M17,
    M18,
    CSB,
    CHB,
    CES
};

struct gcode_data_t
{
    float   x;
    float   y;
    float   z;
    float   i;
    float   j;
    float   k;
    float   t;
};

/**
 * Contains all possible gcode commands with respective datas.
 * 
 * @param G01 { X Y Z I J K T }     linear interpolation
 * @param G02 { X Y Z I J K T }     CW arc or circle
 * @param G03 { X Y Z I J K T }     CCW arc or circle
 * @param G04 { T }                 dwell [ ms ]
 * @param G28                       home all axis
 * @param G90                       absolute positioning
 * @param G91                       relative positioning
 * @param M04                       gripper CLOSE
 * @param M05                       gripper OPEN
 * @param M17                       enable steppers
 * @param M18                       disable steppers
 * @param CSB                       cmd for SOFT BREAK
 * @param CHB                       cmd for HARD BREAK
 * @param CES                       cmd for EMERGENCY STOP 
 */
struct gcode_t
{
    enum    gcode_cmd_t     cmd;
    struct  gcode_data_t    data;
};


////////////////////////////////////////////////////////////////////////////////
// Public (global) externs
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// Public (global) function declarations
////////////////////////////////////////////////////////////////////////////////



#endif // _GCODE_H_