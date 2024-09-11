#include "include/trajectory_generator.h"


TrajectoryGenerator::TrajectoryGenerator()
{
    this->via_point_idx     = 0;
    this->via_point_time    = 0;

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
    this->pos_start         = pos_start;
    this->pos_end           = pos_end;
    this->via_point_idx     = 0;
    this->via_point_step    = 0;
    this->via_point_time    = 0;


    // end effector position
    float path_length       = 0;
    float path_travel_time  = 0;

    path_length = get_path_length(&pos_start, &pos_end);
    if (path_length != 0) {
        path_travel_time = get_best_effort_time(path_length, MAX_LINEAR_VEL, MAX_LINEAR_ACC);
        path_travel_time = std::max(pos_end.t, path_travel_time);   // override if input value is greater
    }

    // end effector rotation
    float rot_length        = 0;
    float rot_travel_time   = 0;

    rot_length = abs(pos_end.k - pos_start.k);
    if (rot_length != 0) {
        rot_travel_time = get_best_effort_time(rot_length, MAX_ROTATION_VEL, MAX_ROTATION_ACC);
        rot_travel_time = std::max(pos_end.t, rot_travel_time);     // override if input value is greater
    }


    // travel time that satisfies vel and acc constraints
    this->travel_time = std::max(path_travel_time, rot_travel_time);
    if (this->travel_time == 0) {
        return false;
    }

    // Quintic polynomial coefficents, used for velocity profile
    // a0 = 0;
    // a1 = 0;
    // a2 = 0;
    this->a3 =   10 / pow(this->travel_time, 3);
    this->a4 =  -15 / pow(this->travel_time, 4);
    this->a5 =    6 / pow(this->travel_time, 5);

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
bool TrajectoryGenerator::get_via_point(position_t *pos)
{
    // mapping position index with quintic polynomial profile
    // a0 = 0; a1 = 0; a2 = 0;
    this->via_point_idx  = a3 * pow(this->via_point_time, 3);
    this->via_point_idx += a4 * pow(this->via_point_time, 4);
    this->via_point_idx += a5 * pow(this->via_point_time, 5);

    if (this->via_point_idx > 1) {
        return false;           // end point reached
    }

    if ((2*this->via_point_idx - this->via_point_step) > 1) {     // end point is almost reached
        this-> via_point_idx    = 1;
        this-> via_point_time   = this->travel_time;
    }

    // mapping end-effector position
    pos->x = ((1 - this->via_point_idx) * pos_start.x) + (this->via_point_idx * pos_end.x);
    pos->y = ((1 - this->via_point_idx) * pos_start.y) + (this->via_point_idx * pos_end.y);
    pos->z = ((1 - this->via_point_idx) * pos_start.z) + (this->via_point_idx * pos_end.z);
    pos->k = ((1 - this->via_point_idx) * pos_start.k) + (this->via_point_idx * pos_end.k);
    pos->t = via_point_time;   // via point time stamp

    // update attribute
    this->via_point_time += VIA_POINTS_TIME_STEP;
    this->via_point_step = this->via_point_idx;     // save previous via_point_idx
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