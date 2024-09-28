#ifndef GCODE_H
#define GCODE_H


/*
 * @brief gcode data types.
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

/**
 * @brief Contains all possible gcode commands.
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
typedef enum
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
} gcode_cmd_t;


/**
 * @brief Contains datas of all possible gcode commands.
 * 
 * @param x     End-effector position, [ mm ]
 * @param y     End-effector position, [ mm ]
 * @param z     End-effector position, [ mm ]
 * @param i     End-effector orientation around x, [ rad ]
 * @param j     End-effector orientation around y, [ rad ]
 * @param k     End-effector orientation around z, [ rad ]
 * @param q1    Joint actuator 1, [ rad ]
 * @param q2    Joint actuator 2, [ rad ]
 * @param q3    Joint actuator 3, [ rad ]
 * @param q4    Joint actuator 4, [ rad ]
 * @param t     Time stamp, [ s ]
 */
typedef struct
{
    float   x;
    float   y;
    float   z;
    float   i;
    float   j;
    float   k;
    float   q1;
    float   q2;
    float   q3;
    float   q4;
    float   t;
} gcode_data_t;


/**
 * @brief Contains gcode command and data instance.
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
 * 
 * @param x     End-effector position, [ mm ]
 * @param y     End-effector position, [ mm ]
 * @param z     End-effector position, [ mm ]
 * @param i     End-effector orientation around x, [ rad ]
 * @param j     End-effector orientation around y, [ rad ]
 * @param k     End-effector orientation around z, [ rad ]
 * @param q1    Joint actuator 1, [ rad ]
 * @param q2    Joint actuator 2, [ rad ]
 * @param q3    Joint actuator 3, [ rad ]
 * @param q4    Joint actuator 4, [ rad ]
 * @param t     Time stamp, [ s ]
 */
typedef struct
{
    gcode_cmd_t     cmd;
    gcode_data_t    data;
} gcode_t;


////////////////////////////////////////////////////////////////////////////////
// Public (global) externs
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// Public (global) function declarations
////////////////////////////////////////////////////////////////////////////////



#endif // GCODE_H