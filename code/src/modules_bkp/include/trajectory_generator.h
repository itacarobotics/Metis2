#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <algorithm>
#include "math.h"

#include "configuration.h"
#include "data_t.h"


// some math constants used in this class
#define SQRT_10     3.16228
#define SQRT2_3     1.31607


class TrajectoryGenerator
{
public:
    TrajectoryGenerator();
    ~TrajectoryGenerator();

    bool        set_trajectory_ptp(position_t pos_start, position_t pos_end);
    bool        get_via_point(position_t *pos);

private:
    float       a3, a4, a5; // a0 = 0, a1 = 0, a2 = 0 --> parameters of quintic polynomial

    position_t  pos_start;
    position_t  pos_end;
    float       travel_time;

    float       via_point_idx;          // a value [0, 1]
    float       via_point_step;         // increment of via_point_idx
    float       via_point_time;         // a value [0, T]

    float       get_path_length(position_t *pos_start, position_t *pos_end);
    float       get_best_effort_time(float path_length, float max_vel, float max_acc);

};


#include "../trajectory_generator.cpp"

#endif  // TRAJECTORY_GENERATOR_H