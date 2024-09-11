#ifndef KERNEL_H
#define KERNEL_H


#include "include/delta_robot.h"
#include "include/buffer.h"


extern Kernel kernel;


class Kernel
{
public:
    Kernel();
    bool get_robot_task(robot_task_t *task);
    bool get_robot_cmd(robot_cmd_t *cmd);
    
    bool add_robot_task(robot_task_t task);
    bool add_robot_cmd(robot_cmd_t cmd);

private:
    Buffer<robot_task_t>    bfr_robot_tasks;
    Buffer<robot_cmd_t>     bfr_robot_cmds;

};

#include "../kernel.cpp"

#endif  // KERNEL_H