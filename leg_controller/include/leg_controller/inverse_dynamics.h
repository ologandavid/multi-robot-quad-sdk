#ifndef INVERSE_DYNAMICS_H
#define INVERSE_DYNAMICS_H

#include <leg_controller/leg_controller_template.h>

//! Implements inverse dynamics as a controller within the ROS framework.
/*!
   InverseDynamicsController implements inverse dynamics logic. It should expose a constructor that does any initialization required and an update method called at some frequency.
*/
class InverseDynamicsController : public LegControllerTemplate {
  public:
    /**
     * @brief Constructor for InverseDynamicsController
     * @return Constructed object of type InverseDynamicsController
     */
    InverseDynamicsController();

    /**
     * @brief Compute the leg command array message for a given current state and reference plan
     * @param[in] local_plan_msg Message of the local referance plan
     */
    void updateLocalPlanMsg(quad_msgs::RobotPlan::ConstPtr msg);

    /**
     * @brief Compute the leg command array message for a given current state and reference plan
     * @param[in] robot_state_msg Message of the current robot state
     * @param[out] leg_command_array_msg Command message after solving inverse dynamics and including reference setpoints for each joint
     * @param[out] grf_array_msg GRF command message
     */
    bool computeLegCommandArray(
      const quad_msgs::RobotState::ConstPtr &robot_state_msg,
      quad_msgs::LegCommandArray &leg_command_array_msg,
      quad_msgs::GRFArray &grf_array_msg);

private:
  quad_msgs::RobotPlan::ConstPtr last_local_plan_msg_;

};


#endif // INVERSE_DYNAMICS_H
