#include <iostream>

#include "../../include/delta_robot.h"


int main()
{
    bool rc;
    DeltaRobot robot;

    position_t pos_goal = {.x = 100, .y = -100, .z = -100, .k = 1.2, .t = 2.3};

    rc = robot.set_motion_task(pos_goal);
    if (!rc) {
        std::cout << "[ERROR!] robot not calibrated.\n";
    }

    robot.set_calibration_status(true);

    rc = robot.set_motion_task(pos_goal);
    if (!rc) {
        std::cout << "[ERROR!] robot not calibrated.\n";
    }

    position_t pos;
    joints_t q;

    while (true) {

        rc = robot.get_motion_cmd(&pos, &q);
        if (rc == false) {
            std::cout << "[ERROR!] invalid trajectory.\n";
            break;
        }

        std::cout << "q1: " << q.q1 << " q2: " << q.q2; 
        std::cout << " q3: " << q.q3 << " q4: " << q.q4;
        std::cout << " t: " << q.t << "\n";
    }
    
}