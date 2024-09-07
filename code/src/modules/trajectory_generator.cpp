#include "include/trajectory_generator.h"


TrajectoryGenerator::TrajectoryGenerator(float max_vel, float max_acc,
    float via_points_distance, float via_points_threshold)
{
    this->max_vel = max_vel;
    this->max_acc = max_acc;
    this->via_points_distance = via_points_distance;
    this->via_points_threshold = via_points_threshold;

    return;
}

TrajectoryGenerator::~TrajectoryGenerator()
{
}


bool TrajectoryGenerator::set_pos_end(position_t pos_end)
{
    if (via_point_idx != 0) {
        return false;
    }

    this->pos_end = pos_end;
    return true;
}
