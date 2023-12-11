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

void ConflictBasedSearch::requestInitialPaths(){
  for (const auto robot : robot_names_){
    // std::cout <<" 1" << std::endl;
    global_body_planner::ExampleService::Request req;
    req.a = 5;
    req.b = 7;
    global_body_planner::ExampleService::Response res;
    if(robot_clients_[robot].call(req, res)){
      robot_plan_map_[robot] = res.plan;
    }
  }
  return;
}

// Compares Relative Pose Distances Between Two Robot States
double ConflictBasedSearch::comDistance(const quad_msgs::RobotState &s1, const quad_msgs::RobotState &s2) {
  double x1 = s1.body.pose.position.x;
  double y1 = s1.body.pose.position.y;
  double z1 = s1.body.pose.position.z;

  double x2 = s2.body.pose.position.x;
  double y2 = s2.body.pose.position.y;
  double z2 = s2.body.pose.position.z;
  return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2) + std::pow(z2 - z1, 2));
  // return (s1.body.pose - s2.body.pose).norm();
}

// Given two inputs states, collision threshold checks pose distance for collision
bool ConflictBasedSearch::statesIntersect(const quad_msgs::RobotState &state_1,
                      const quad_msgs::RobotState &state_2, double threshold){
  if (comDistance(state_1, state_2) < threshold){
    return true;
  }
  return false;
}


// Identifies and updates collisions between each robot path
// Updates Map with Collision Struct ([ts, tf], r1, r2)
bool ConflictBasedSearch::doPlansCollide(){ // Maybe Take the Top Two loops outside of this fcn for cleanliness
  for (const auto& robot_a : robot_names_){
    for (const auto& robot_b : robot_names_){ // Iterate over each pair of robots
      if (robot_a != robot_b){ // Avoid Repeats

      int t_s = 0; // Indicies of Collision Start and End
      int l_collision = 0;

        for (int i = 0; i < robot_plan_map_[robot_a].plan_indices.back(); i++){  //Check Local Planner if RobotState Msg is iterable
          quad_msgs::RobotState current_state_;
          current_state_ = robot_plan_map_[robot_a].states[i];
          double t = robot_plan_map_[robot_a].states[i].header.stamp.toSec(); // Time in Plan A
          double t_plan_b_final = robot_plan_map_[robot_b].states.back().header.stamp.toSec(); // Time of Last Element of Plan B
          double t_interp = std::min(t, t_plan_b_final); // Make sure we account for different path lengths

          //Interpolate to get the Corresponding Robot State in the Second Plan
          quad_msgs::RobotState interp_state_;
          int c_idx; // Keep track of the corresponding time index after interpolation
          for (int j = 0; j < robot_plan_map_[robot_b].states.size()-1; j++){
            if ((t_interp >= robot_plan_map_[robot_b].states[j].header.stamp.toSec()) &&
                    (t_interp < robot_plan_map_[robot_b].states[j+1].header.stamp.toSec())){
                      quad_utils::interpRobotState(robot_plan_map_[robot_b].states[j],
                      robot_plan_map_[robot_b].states[j+1], t_interp, interp_state_);
                      c_idx = j;
            }
          }
          // std::cout << "Indices" << i << " , " << c_idx <<std::endl;

          //Check for collisions after finding the Corresponding Robot State
          if (statesIntersect(current_state_, interp_state_, threshold)){
            if (l_collision == 0){
              l_collision++;
              t_s = i;
            }
            else{
              l_collision++;
            }
          }
          else{
            if (l_collision != 0){
              // Reset counter to 0, add the existing conflict to the conflict list
              std::tuple<std::string, std::string, int, int> conflict;
              std::cout << "Conflict Tuple: " << robot_a << " , " << robot_b << " , " << t_s << " , " << i-1 << std::endl;
              conflict = std::make_tuple(robot_a, robot_b, t_s, i-1);
              conflict_list.push_back(conflict);
              l_collision = 0;
              ROS_INFO_STREAM("Adding Conflicts");
            }
          }
        }
      }
    }
  }
  return false;
}

// Publishes the found Collision Free Path
void ConflictBasedSearch::publishPaths(){
  for (const auto robot : robot_names_)
    robot_plan_pubs_[robot].publish(robot_plan_map_[robot]);
  return;
}

// Converts Conflicts in the Conflict Tree to Position Constraints for the Robot
void ConflictBasedSearch::getConstaintFromConflict(){
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
        requestInitialPaths();
        bool collide = doPlansCollide();
        if (collide){
          publishPaths(); //Check to See if Duplicated Paths are Valid
        }
        else{
          getConstraintFromConflict();
        }
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
