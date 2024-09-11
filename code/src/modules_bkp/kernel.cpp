#include "include/kernel.h"


// Kernel::Kernel()
// {
//     return;
// }


bool Kernel::get_robot_task(robot_task_t *task)
{
    bool rc;

    this->bfr_robot_tasks.consume(task);
    if (rc == false) {
        return false;   // buffer is empty
    }

    return true;
}


bool Kernel::get_robot_cmd(robot_cmd_t *cmd)
{
    bool rc;

    this->bfr_robot_cmds.consume(cmd);
    if (rc == false) {
        return false;   // buffer is empty
    }

    return true;
}


bool Kernel::add_robot_task(robot_task_t task)
{
    bool rc;

    this->bfr_robot_tasks.produce(task);
    if (rc == false) {
        return false;   // buffer is full
    }

    return true;
}


bool Kernel::add_robot_cmd(robot_cmd_t cmd)
{
    bool rc;

    this->bfr_robot_cmds.produce(cmd);
    if (rc == false) {
        return false;   // buffer is full
    }

    return true;
}