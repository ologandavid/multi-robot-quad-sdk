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

  int counter = 0;
  for (const auto robot_a : robot_names_){
    for (const auto robot_b : robot_names_){
      if (robot_a != robot_b){
        for(int i = 0; i < max_path_length_; i++){
            State r1_state;
            State r2_state;
            r1_state.pos = Eigen::Vector3d{robot_plan_map_[robot_a].states[i].body.pose.position.x, 
                                              robot_plan_map_[robot_a].states[i].body.pose.position.y, 
                                                robot_plan_map_[robot_a].states[i].body.pose.position.z};                                              
            r2_state.pos = Eigen::Vector3d{robot_plan_map_[robot_b].states[i].body.pose.position.x, 
                                              robot_plan_map_[robot_b].states[i].body.pose.position.y, 
                                                robot_plan_map_[robot_b].states[i].body.pose.position.z};
            if (poseDistance(r1_state, r2_state) < threshold){
              ros::Time r1_time = robot_plan_map_[robot_a].states[i].header.stamp;
              ros::Time r2_time = robot_plan_map_[robot_b].states[i].header.stamp;
              ros::Duration timeDifference = r1_time - r2_time;
              double timeDifferenceInSeconds = timeDifference.toSec();
              if (std::abs(timeDifferenceInSeconds) < time_thresh){
                std::cout << "Finds Collision: " << robot_plan_map_[robot_a].states[i].body.pose.position.x << robot_plan_map_[robot_b].states[i].body.pose.position.x << ", at Time:" <<  timeDifferenceInSeconds << std::endl;
                counter++;
              }
            }
          }
      }
    }
  }
  std::cout << "Counter Value" << counter << std::endl;
  return;
}

// double ConflictBasedSearch::calcDistance(double x1, double y1, double z1
//                                           double x2, double y2, double z2){
//   double dist = std::sqrt(std::pow((x1-x2),2)+ std::pow((y1-y2),2) + std::pow((z1-z2),2));
//   return dist;
// }

double ConflictBasedSearch::poseDistance(const State &s1, const State &s2) {
  return (s1.pos - s2.pos).norm();
}

void ConflictBasedSearch::publishPaths(){
  for (const auto robot : robot_names_)
    robot_plan_pubs_[robot].publish(robot_plan_map_[robot]);
  return;
}

void ConflictBasedSearch::equalizePaths(){
  std::cout << "Original Robot 1 State Vector Size: " << robot_plan_map_["robot_1"].states.size() <<std::endl;
  std::cout << "Original Robot 2 State Vector Size: " << robot_plan_map_["robot_2"].states.size() <<std::endl;

  // Equalize the timescale of each RobotPlan for CBS
  max_path_length_= 0; // Find the length of the longest path
  for (const auto robot : robot_names_){
    if (robot_plan_map_[robot].states.size() > max_path_length_){
      max_path_length_ = robot_plan_map_[robot].states.size();
    }
  }

  for (const auto robot : robot_names_){
    if (robot_plan_map_[robot].states.size() < max_path_length_){
      quad_msgs::RobotState finalState = robot_plan_map_[robot].states.back();
      ros::Time t_f = finalState.header.stamp;
      std::vector<quad_msgs::RobotState>* shorterStates = &robot_plan_map_[robot].states;

      int diff = max_path_length_ - robot_plan_map_[robot].states.size(); 
      for (int i= 0; i < diff; i++){
        ros::Duration duration_to_add(i*0.03);
        finalState.header.stamp += duration_to_add;
        shorterStates->push_back(finalState);
      }
      robot_plan_map_[robot].states= *shorterStates;
    }
  }
  ROS_INFO_STREAM(robot_plan_map_["robot_1"]);
  std::cout << "New Robot 1 State Vector Size: " << robot_plan_map_["robot_1"].states.size() <<std::endl;
  std::cout << "New Robot 2 State Vector Size: " << robot_plan_map_["robot_2"].states.size() <<std::endl;
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
        equalizePaths();
        parsePaths();
        publishPaths(); //Check to See if Duplicated Paths are Valid
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
