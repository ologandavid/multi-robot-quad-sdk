#include <ros/ros.h>
#include "conflict_based_search/conflict_based_search.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "cbs");
  ros::NodeHandle nh;
  ROS_INFO_STREAM("Running RN");

  ConflictBasedSearch conflict_based_search(nh);
  conflict_based_search.spin();
  return 0;
}
