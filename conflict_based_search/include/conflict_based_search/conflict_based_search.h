#ifndef CONFLICT_BASED_SEARCH_H
#define CONFLICT_BASED_SEARCH_H
#include <ros/ros.h>
#include <quad_msgs/RobotPlan.h>
#include <quad_msgs/RobotState.h>
#include <quad_utils/quad_kd.h>
#include <quad_utils/ros_utils.h>
#include <math.h>
#include <string>
#include <vector>
#include <map>


class ConflictBasedSearch {
 public:
  /**
   * @brief Constructor for ConflictBasedSearch Class
   * @param[in] nh Node handle
   * @return Constructed object of type ConflictBasedSearch
   */
  ConflictBasedSearch(ros::NodeHandle nh);

  /**
   * @brief Calls ros spinOnce and pubs data at set frequency
   */
  void spin();

  /**
   * @brief Callback function to handle new plans
   * @param[in] msg Robot state trajectory message
   */
  void robotPlanCallback(const quad_msgs::RobotPlan::ConstPtr &msg);
  // void robotPlanCallback(const quad_msgs::RobotPlan::ConstPtr &msg, const std::string &robot_name);


  /**
   * @brief Callback function to handle new plans
   * @param[in] msg Robot state trajectory message
   */
  void discreteRobotPlanCallback(const quad_msgs::RobotPlan::ConstPtr &msg, const std::string &robot_name);

  void printRobotPlans();

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  std::vector<std::string> robot_names_;

  ros::Subscriber body_plan_sub_;
  /// ROS subscriber for incoming body plans
  std::map<std::string, ros::Subscriber> body_plan_subs_;

  /// ROS subscriber for incoming body plans
  std::map<std::string, ros::Subscriber> discrete_body_plan_subs_;

  /// Most recent robot plan
  // quad_msgs::RobotPlan::ConstPtr body_plan_msg_;
  std::map<std::string, quad_msgs::RobotPlan::ConstPtr> body_plan_msg_;

  /// Most recent robot plan
  std::map<std::string, quad_msgs::RobotPlan::ConstPtr> discrete_body_plan_msg_;

  /// Update rate for sending and receiving data;
  double update_rate_;


}; // CONFLICT_BASED_SEARCH_H

#endif  // CONFLICT_BASED_SEARCH_H
