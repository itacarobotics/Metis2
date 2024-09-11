#ifndef DELTA_ROBOT_H
#define DELTA_ROBOT_H


#include "inverse_geometry.h"
#include "trajectory_generator.h"



class DeltaRobot
{
public:
    DeltaRobot();
    void                set_robot_status(robot_status_t status);
    void                set_calibration_status(bool status);
    bool                set_trajectory_ptp(position_t pos_goal);
    void                set_pos_current(position_t pos);

    robot_status_t      get_robot_status(void);
    bool                get_joints_via_point(position_t *pos, joints_t *q);

private:
    robot_status_t      robot_status;
    bool                is_robot_calibrated;

    TrajectoryGenerator tg;
    InverseGeometry     ig;

    position_t          pos_home;
    position_t          pos_current;
};


#include "../delta_robot.cpp"

#endif  // DELTA_ROBOT_H