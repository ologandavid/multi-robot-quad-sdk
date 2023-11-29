#include <ros/ros.h>
#include "conflict_based_search/conflict_based_search.h"

ConflictBasedSearch::ConflictBasedSearch(ros::NodeHandle nh){
    nh_ = nh;
    nh.param<double>("/conflict_based_search/update_rate", update_rate_,250);
    // Load rosparams
    std::string body_plan_topic, discrete_body_plan_topic;

    quad_utils::loadROSParam(nh_, "topics/global_plan", body_plan_topic);
    quad_utils::loadROSParam(nh_, "topics/global_plan_discrete", discrete_body_plan_topic);

    //Initialize Pubs and Subs

    body_plan_sub_ = nh_.subscribe(body_plan_topic, 1,
                                   &ConflictBasedSearch::robotPlanCallback, this);

    discrete_body_plan_sub_ = nh_.subscribe(discrete_body_plan_topic, 1, 
                                            &ConflictBasedSearch::discreteRobotPlanCallback, this);

    std::cout << "Subs Created"<< std::endl;
}


void ConflictBasedSearch::robotPlanCallback(
    const quad_msgs::RobotPlan::ConstPtr &msg) {
    body_plan_msg_ = msg;
}

void ConflictBasedSearch::discreteRobotPlanCallback(
    const quad_msgs::RobotPlan::ConstPtr &msg) {
    discrete_body_plan_msg_ = msg;
}


void ConflictBasedSearch::spin() {
  ros::Rate r(update_rate_);

  // Wait until we get map and state data
  // waitForData();

  // Enter main spin
  while (ros::ok()) {
    // Process callbacks
    ros::spinOnce();
    // ROS_INFO_STREAM("Spinning");

    std::cout << "Getting Both Plans"<< std::endl;

    // Set the start and goal states
    // setStartState();
    // setGoalState();

    // Call the planner
    // callPlanner();

    // Publish the results if valid
    // publishCurrentPlan();

    r.sleep();
  }
}
