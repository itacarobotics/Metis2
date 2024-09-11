#ifndef DATA_T_H
#define DATA_T_H

#include "stdint.h"

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
    gcode_cmds_t cmd;
    gcode_data_t data;
};

enum gcode_cmds_t
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

struct joints_t
{
    float q1;
    float q2;
    float q3;
    float q4;
    float t;        // time stamp
};


struct position_t
{
    float x;
    float y;
    float z;
    // float i;     // rotation about x axis     --> i = 0
    // float j;     // rotation about y axis     --> j = 0
    float k;        // rotation about z axis
    float t;        // time stamp
};


enum gripper_t
{
    OPEN,
    CLOSED
};


enum header_t
{
    MOTION,     // G0* cmds
    GRIPPER,    // M0* cmds
    HOMING      // G28
};

struct robot_cmd_t
{
    header_t    header;
    joints_t    joints;
    gripper_t   gripper;
};

struct robot_task_t
{
    header_t    header;
    position_t  position;
    gripper_t   gripper;
};


enum robot_status_t
{
    IDLE,
    ERROR,
    RUN,
    SOFT_BREAK,         // the goal position is reached and then break
    HARD_BREAK,         // it stops with motors enabled
    EMERGENCY_STOP      // it stops with motors disabled
};


#endif  // DATA_T_H