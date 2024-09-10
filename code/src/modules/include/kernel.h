#ifndef KERNEL_H
#define KERNEL_H


#include "include/delta_robot.h"
#include "include/buffer.h"


class Kernel
{
public:
    Kernel();
    bool add_robot_task();

private:
    DeltaRobot robot;

    Buffer<robot_task_t>    robot_tasks;
    Buffer<robot_cmd_t>     robot_cmds;


};


#include "../kernel.cpp"

#endif  // KERNEL_H