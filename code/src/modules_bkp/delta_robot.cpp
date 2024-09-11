#include "include/delta_robot.h"


DeltaRobot::DeltaRobot()
{
    this->pos_home = {.x = 0, .y = 0, .z = POSITION_HOME_OFFSET_Z,.k = 0, .t = 0};
    
    this->set_calibration_status(false);    // pos_current still needs to be initialized
    this->set_robot_status(IDLE);

    return;
}


void DeltaRobot::set_robot_status(robot_status_t status)
{
    this->robot_status = status;
    return;
}

/**
 *  @brief Needs to be called after homing calibration.
 *  @param  status True if robot has been calibrated.
 *  @return nothing.
 * 
 *  If this flag is set to false, new motion tasks are not permitted. 
 */
void DeltaRobot::set_calibration_status(bool status)
{
    if (status) {
        this->pos_current = this->pos_home;
    }
    this->is_robot_calibrated = status;

    return;
}

/**
 *  @brief Sets the parameters for a ptp trajectory.
 *  @param  pos_goal Desired end-effector goal position.
 *  @return true if position is feasable, false otherwise.
 *  
 */
bool DeltaRobot::set_trajectory_ptp(position_t pos_goal)
{
    bool rc;

    if (this->is_robot_calibrated == false) {   // a homing hasn't been done yet
        return false;
    }

    rc = tg.set_trajectory_ptp(this->pos_current, pos_goal);
    return rc;
}


robot_status_t DeltaRobot::get_robot_status(void)
{
    return this->robot_status;
}

/**
 *  @brief The joint positions of the next via point.
 *  @param  *pos Pointer to desired end-effector position.
 *  @param  *q Pointer to respective joints position.
 *  @return true if position is feasable, false otherwise.
 *  
 */
bool DeltaRobot::get_joints_via_point(position_t *pos, joints_t *q)
{
    bool rc;
    
    // get via point in cartesian coordinates
    rc = tg.get_via_point(pos);
    if (rc == false) {
        return rc;
    }

    rc = ig.inverse_geometry(pos, q);
    if (rc == false) {
        return rc;
    }

    return true;
}
