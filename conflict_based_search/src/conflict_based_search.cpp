#include <ros/ros.h>
#include <boost/bind.hpp>
#include "conflict_based_search/conflict_based_search.h"
#include "global_body_planner/ExampleService.h"


ConflictBasedSearch::ConflictBasedSearch(ros::NodeHandle nh){
    nh_ = nh;

    // nh.param<double>("/conflict_based_search/update_rate", update_rate_,250);

    // Load rosparams
    std::string body_plan_topic; //, discrete_body_plan_topic;
    quad_utils::loadROSParam(nh_, "/conflict_based_search/update_rate", update_rate_);
    quad_utils::loadROSParam(nh_, "/conflict_based_search/robot_names",robot_names_);


    // body_plan_sub_ = nh_.subscribe("/robot_1/global_plan", 1,
    //                                &ConflictBasedSearch::robotPlanCallback, this);
    //Initialize Pubs and Subs, Keep Track of Robots using Map
    for (const auto robot : robot_names_) {
      std::cout << robot << std::endl;
      // body_plan_subs_[robot] = nh_.subscribe("/" + robot + "/global_plan", 1,
      //                              &ConflictBasedSearch::robotPlanCallback, this);

}
}

// void ConflictBasedSearch::robotPlanCallback(const quad_msgs::RobotPlan::ConstPtr &msg) {

//     body_plan_msg_[robot_name] = msg;
// }



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

void ConflictBasedSearch::spin() {
  ros::Rate r(update_rate_);

  // Wait until we get map and state data
  // waitForData();
  ros::ServiceClient client = nh_.serviceClient<global_body_planner::ExampleService>("/robot_1/add_two_ints");
  std::cout << "client setup" << std::endl;
  // Enter main spin
  while (ros::ok()) {
    // Process callbacks
    ros::spinOnce();
    // Create a request message
    global_body_planner::ExampleService::Request req;
    req.a = 5;
    req.b = 7;
    // Send request to the service server
    global_body_planner::ExampleService::Response res;
    if (client.call(req, res))
    {
        ROS_INFO("Sum: %d", res.sum);
    }
    else
    { 
      ROS_INFO("Tuff");
        // ROS_ERROR("Failed to call service");
    }

    r.sleep();
  }
}
