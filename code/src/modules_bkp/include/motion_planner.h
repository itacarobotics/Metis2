#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include "include/data_t.h"
#include "include/inverse_geometry.h"
#include "include/trajectory_generator.h"


extern MotionPlanner mp;


class MotionPlanner
{
private:
    float       a3, a4, a5; // a0 = 0, a1 = 0, a2 = 0 --> parameters of quintic polynomial

    gcode_t     pos_start;
    gcode_t     pos_end;
    float       travel_time;

    float       via_point_idx;          // a value [0, 1]
    float       via_point_step;         // increment of via_point_idx
    float       via_point_time;         // a value [0, T]

    float       get_path_length(position_t *pos_start, position_t *pos_end);
    float       get_best_effort_time(float path_length, float max_vel, float max_acc);

    
    bool        set_motion(gcode_t cmd);
    bool        get_motion_cmd(gcode_t *cmd);

public:
    MotionPlanner();
    
    bool        get_joints_motion();



};


#endif  // MOTION_PLANNER_H