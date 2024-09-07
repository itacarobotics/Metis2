#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include "data_t.h"

class TrajectoryGenerator
{
private:
    float       max_vel;
    float       max_acc;
    float       via_points_distance;
    float       via_points_threshold;

    float       a3, a4, a5; // a0 = 0, a1 = 0, a2 = 0 --> parameters of quintic polynomial

    position_t  pos_start;
    position_t  pos_end;
    float       via_point_idx;    // a value [0,1]
    float       via_point_step;   // via_point_idx += via_point_step

    float       get_path_length(void);
    float       get_best_effort_time(float path_length);
    void        set_via_point_step(float path_length);

public:
    TrajectoryGenerator(float max_vel, float max_acc, 
        float via_points_distance, float via_points_threshold);
    ~TrajectoryGenerator();

    bool        set_trajectory(position_t pos_start, position_t pos_end);
    position_t  get_next_via_point(void);

};


#endif  // TRAJECTORY_GENERATOR_H