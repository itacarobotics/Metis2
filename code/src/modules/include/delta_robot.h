#ifndef DELTA_ROBOT_H
#define DELTA_ROBOT_H


#include "include/inverse_geometry.h"
#include "include/trajectory_generator.h"


class DeltaRobot
{
public:
    DeltaRobot();

private:
    TrajectoryGenerator     tg;
    InverseGeometry         ig;

    

};


#include "../delta_robot.cpp"

#endif  // DELTA_ROBOT_H