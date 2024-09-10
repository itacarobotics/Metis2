#ifndef DATA_T_H
#define DATA_T_H

#include "stdint.h"



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

struct joints_t
{
    float q1;
    float q2;
    float q3;
    float q4;
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