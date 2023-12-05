#include <ros/ros.h>
#include <boost/bind.hpp>
#include "conflict_based_search/conflict_based_search.h"
#include "global_body_planner/global_body_planner.h"
// #include "global_body_planner/ExampleService.h"


ConflictBasedSearch::ConflictBasedSearch(ros::NodeHandle nh){
    nh_ = nh;
    // Load rosparams
    std::string body_plan_topic; //, discrete_body_plan_topic;
    quad_utils::loadROSParam(nh_, "/conflict_based_search/update_rate", update_rate_);
    quad_utils::loadROSParam(nh_, "/conflict_based_search/robot_names",robot_names_);
    pathFound = true;

    //Set up Publishers for Each Robot's Topic to Output Best Plan for Each Robot
    for (const auto robot : robot_names_) {
      robot_plan_pubs_[robot] = nh_.advertise<quad_msgs::RobotPlan>("/" + robot + "/global_plan", 1);
    }
}

void ConflictBasedSearch::createServiceClients(){
  for (const auto robot : robot_names_){
    // std::cout << "Client for " << robot << "Successfully Setup" << std::endl;
    robot_clients_[robot] = nh_.serviceClient<global_body_planner::ExampleService>("/" + robot + "/add_two_ints");
  }
}

void ConflictBasedSearch::requestPaths(){
  for (const auto robot : robot_names_){
    // std::cout <<" 1" << std::endl;
    global_body_planner::ExampleService::Request req;
    req.a = 5;
    req.b = 7;
    global_body_planner::ExampleService::Response res;
    if(robot_clients_[robot].call(req, res)){
      robot_plan_map_[robot] = res.plan;
      // ROS_INFO_STREAM(robot_plan_map_[robot]);
    }
  }
  return;
}

void ConflictBasedSearch::parsePaths(){
  return;
}


void ConflictBasedSearch::spin() {
  ros::Rate r(update_rate_);

  createServiceClients();
  ros::Duration timeout_duration(10.0);  // Specify the timeout duration (in seconds)

  // Enter main spin
  while (ros::ok()) {
    // Process callbacks
    ros::spinOnce();
    // ros::Duration(5.0).sleep();
    // Wait for the service to become available
    bool service_available = ros::service::waitForService("/robot_1/add_two_ints", timeout_duration);
    if (service_available){
      if (pathFound){
      requestPaths();
      pathFound = false;
      }
    }
    r.sleep();
  }
}


// void ConflictBasedSearch::printRobotPlans() {
//     for (const auto &robot : robot_names_) {
//         if (body_plan_msg_.count(robot) && discrete_body_plan_msg_.count(robot)) {
//             ROS_INFO_STREAM("Body plan for" << robot << ": " << body_plan_msg_[robot]);
//             // ROS_INFO_STREAM("Discrete body plan for robot " << robot << ": " << discrete_body_plan_msg_[robot]);
//         } else {
//             ROS_WARN_STREAM("No plan available for " << robot);
//         }
//     }
// }
