#include <ros/ros.h>
#include "global_body_planner/global_body_planner.h"
// #include "global_body_planner/ExampleService.h"

// bool add(global_body_planner::ExampleService::Request &req,
//          global_body_planner::ExampleService::Response &res)
// {
//     res.sum = req.a + req.b;
//     ROS_INFO("Received request: a=%d, b=%d. Sending response: sum=%d", req.a, req.b, res.sum);
//     return true;
// }

int main(int argc, char **argv) {
  ros::init(argc, argv, "global_body_planner");
  ros::NodeHandle nh;
  GlobalBodyPlanner global_body_planner(nh);

  // ros::ServiceServer service = nh.advertiseService("add_two_ints", add);
  // ROS_INFO("Service node ready to provide service.");

  global_body_planner.spin();
  return 0;
}
