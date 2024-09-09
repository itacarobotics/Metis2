#include "include/trajectory_generator.h"


TrajectoryGenerator::TrajectoryGenerator()
{
    this->time_step_idx   = 0;
    this->time_step       = 0;

    return;
}

TrajectoryGenerator::~TrajectoryGenerator()
{
}


/**
 *  @brief Point to point trajectory with quintic polynomial velocity profile
 *  @param  pos_start  Current end-effector position.
 *  @param  pos_end  The desired goal position.
 *  @return true if trajectory is feasable, false otherwise.
 *
 *  Sets the parameters needed to make a point to point trajectory with a quintic polynomial
 *  velocity profile.
 */
bool TrajectoryGenerator::set_trajectory_ptp(position_t pos_start, position_t pos_end)
{
    bool rc;

    // update attributes
    this->pos_start     = pos_start;
    this->pos_end       = pos_end;
    this->time_step     = 0;
    this->time_step_idx = 0;


    /**********************************
     *   END-EFFECTOR PATH MOTION
     **********************************/
    float path_length       = 0;
    float path_travel_time  = 0;
    float path_time_step    = 0;

    path_length = get_path_length(&pos_start, &pos_end);
    if (path_length != 0) {
        path_travel_time = get_best_effort_time(path_length, MAX_LINEAR_VEL, MAX_LINEAR_ACC);
        path_travel_time = std::max(pos_end.t, path_travel_time);   // override if input value is greater
        path_time_step = PATH_STEP_DISTANCE / path_length;
    }


    /**********************************
     *   END-EFFECTOR AXIS ROTATION
     **********************************/
    float rot_length        = 0;
    float rot_travel_time   = 0;
    float rot_time_step     = 0;

    rot_length = abs(pos_end.k - pos_start.k);
    if (rot_length != 0) {
        rot_travel_time = get_best_effort_time(rot_length, MAX_ROTATION_VEL, MAX_ROTATION_ACC);
        rot_travel_time = std::max(pos_end.t, rot_travel_time);     // override if input value is greater
        rot_time_step = ROTATION_STEP_DISTANCE / rot_length;
    }

    // travel time that satisfies vel and acc constraints
    travel_time = std::max(path_travel_time, rot_travel_time);
    if (travel_time == 0) {
        return false;
    }

    // Quintic polynomial coefficents, used for velocity profile
    // a0 = 0;
    // a1 = 0;
    // a2 = 0;
    this->a3 =   10 / pow(travel_time, 3);
    this->a4 =  -15 / pow(travel_time, 4);
    this->a5 =    6 / pow(travel_time, 5);


    // step increase
    time_step = std::min(path_time_step, rot_time_step);
    if (time_step <= 0) {
        time_step = std::max(path_time_step, rot_time_step);    // in case one of the two is zero
    }

    return true;    // trajectory is valid
}


/**
 *  @brief Get the via point with time stamp
 *  @param  pos  Pointer to via point.
 *  @return true if via point available, false if end point is reached.
 *
 *  When this method is called, the next via point is computed with its respective time
 *  stamp to follow a quintic polynomial velocity profile. When the end position is reached,
 *  this method returns false. 
 */
bool TrajectoryGenerator::get_next_via_point(position_t *pos)
{
    if (time_step_idx > 1) {
        return false;           // end point reached
    }
    
    pos->t = travel_time * time_step_idx;   // via point time stamp

    float via_point_idx;                    // remapping index with quintic polynomial
    if ((time_step_idx + time_step) < 1) {
        // a0 = 0; a1 = 0; a2 = 0;
        via_point_idx = a3 * pow(pos->t, 3) + a4 * pow(pos->t, 4) + a5 * pow(pos->t, 5);
    } else {
        // this is to avoid floating point errors when adding the time step
        pos->t = travel_time;
        via_point_idx = 1;
    }

    // mapping end-effector position
    pos->x = ((1 - via_point_idx) * pos_start.x) + (via_point_idx * pos_end.x);
    pos->y = ((1 - via_point_idx) * pos_start.y) + (via_point_idx * pos_end.y);
    pos->z = ((1 - via_point_idx) * pos_start.z) + (via_point_idx * pos_end.z);
    pos->k = ((1 - via_point_idx) * pos_start.k) + (via_point_idx * pos_end.k);

    // update step, value [0, 1]
    time_step_idx += time_step;
    return true;
}


float TrajectoryGenerator::get_path_length(position_t *pos_start, position_t *pos_end)
{
    float dx, dy, dz;
    
    dx = this->pos_end.x - this->pos_start.x;
    dy = this->pos_end.y - this->pos_start.y;
    dz = this->pos_end.z - this->pos_start.z;
    
    return sqrt((dx*dx + dy*dy + dz*dz));   // path length --> pytagoras theorem
}


float TrajectoryGenerator::get_best_effort_time(float path_length, float max_vel, float max_acc)
{
    float time_vel_constrained;
    time_vel_constrained = (15 * path_length) / (8 * max_vel);

    float time_acc_constrained;
    time_acc_constrained = (SQRT_10 * sqrt(path_length)) / (SQRT2_3 * sqrt(max_acc));

    return std::max(time_vel_constrained, time_acc_constrained);
}