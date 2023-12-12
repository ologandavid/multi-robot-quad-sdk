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
    runOnce = true;
    goal_reached_ = false; 

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

std::vector<double> ConflictBasedSearch::flattenConstraint(const std::vector<std::vector<double>>& input,
                                      int& originalRows, int& originalColumns) {
    std::vector<double> flattenedVector;

    // Store original row and column counts
    originalRows = input.size();
    if (originalRows > 0) {
        originalColumns = input[0].size();
    } else {
        originalColumns = 0;
    }

    for (const auto& row : input) {
        for (const auto& value : row) {
            flattenedVector.push_back(value);
        }
    }

    return flattenedVector;
}

// Requests Paths for Both Robots for the Root Node
void ConflictBasedSearch::requestInitialPaths(GraphNode& node){
  quad_msgs::RobotPlanConflicts plan_conflicts;
  plan_conflicts.robot_pos = {};
  plan_conflicts.rows = 0;
  plan_conflicts.cols = 0;
  double c = 0.0;
  for (const auto robot : node.robot_names_){
    // std::cout <<" 1" << std::endl;
    global_body_planner::ExampleService::Request req;
    req.conflicts = plan_conflicts;
    global_body_planner::ExampleService::Response res;
    if(robot_clients_[robot].call(req, res)){
      node.robot_plan_map_[robot] = res.plan;
      c += res.path_length;
    }
  }
  node.cost = c;
  return;
}

// Returns Path for a Single Robot during Successor Search
void ConflictBasedSearch::requestPath(GraphNode& node, 
      std::tuple<std::string, std::string, int, int> conflict, bool flip){
  std::string robot;
  if (!flip){ //r1, r2 case (Robot 1 has conflicts)
      // Generate Request to Replan for Robot 1 with Conflicts
    robot = std::get<0>(conflict);
  }
  else{
    robot = std::get<1>(conflict);
  }
  quad_msgs::RobotPlanConflicts plan_conflicts;
  int rows, cols;
  std::vector<double> flattenedConstraint = 
        flattenConstraint(node.constraints_[robot], rows, cols);
  //Write a function to flatten our constraints into a 1d vector that can be recomposed
  plan_conflicts.robot_pos = flattenedConstraint;
  plan_conflicts.rows = rows;
  plan_conflicts.cols = cols;
  // printVector(flattenedConstraint);
  // std::cout << "Vector Length" << flattenedConstraint.size() << std::endl;
 
  // // Send Request
  global_body_planner::ExampleService::Request req;
  req.conflicts = plan_conflicts;
  global_body_planner::ExampleService::Response res;
 
  if(robot_clients_[robot].call(req, res)){
   
    node.robot_plan_map_[robot] = res.plan;
    // c += res.path_length; // Fix How I Update Path Cost
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
// Maybe Take the Top Two loops outside of this fcn for cleanliness
bool ConflictBasedSearch::doPlansCollide(GraphNode& node, 
          std::vector<std::tuple<std::string, std::string, int, int>>& conflict_list){ 
  bool collides = false;
  for (const auto& robot_a : node.robot_names_){
    for (const auto& robot_b : node.robot_names_){ // Iterate over each pair of robots
      if (robot_a != robot_b){ // Avoid Repeats

      int t_s = 0; // Indicies of Collision Start and End
      int l_collision = 0;

        for (int i = 0; i < node.robot_plan_map_[robot_a].plan_indices.back(); i++){  //Check Local Planner if RobotState Msg is iterable
          quad_msgs::RobotState current_state_;
          current_state_ = node.robot_plan_map_[robot_a].states[i];
          double t = node.robot_plan_map_[robot_a].states[i].header.stamp.toSec(); // Time in Plan A
          double t_plan_b_final = node.robot_plan_map_[robot_b].states.back().header.stamp.toSec(); // Time of Last Element of Plan B
          double t_interp = std::min(t, t_plan_b_final); // Make sure we account for different path lengths

          //Interpolate to get the Corresponding Robot State in the Second Plan
          quad_msgs::RobotState interp_state_;
          int c_idx; // Keep track of the corresponding time index after interpolation
          for (int j = 0; j < node.robot_plan_map_[robot_b].states.size()-1; j++){
            if ((t_interp >= node.robot_plan_map_[robot_b].states[j].header.stamp.toSec()) &&
                    (t_interp < node.robot_plan_map_[robot_b].states[j+1].header.stamp.toSec())){
                      quad_utils::interpRobotState(node.robot_plan_map_[robot_b].states[j],
                      node.robot_plan_map_[robot_b].states[j+1], t_interp, interp_state_);
                      c_idx = j;
            }
          }
          // std::cout << "Indices" << i << " , " << c_idx <<std::endl;

          //Check for collisions after finding the Corresponding Robot State
          if (statesIntersect(current_state_, interp_state_, threshold)){
            if (l_collision == 0){
              l_collision++;
              t_s = i;
              collides = true;
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
  return collides;
}

// Takes in Constraint Vector and Returns Positional Constraints for the Robot
void ConflictBasedSearch::getConstraintFromConflict(GraphNode& node, 
            std::tuple<std::string, std::string, int, int>& conflict, 
            std::vector<std::vector<double>>& constraints){
    
    std::cout << std::get<1>(conflict) << std::endl;
    ROS_INFO_STREAM(node.robot_plan_map_["robot_1"].states[2].body.pose.position.x);
    for (int i = std::get<2>(conflict); i < std::get<3>(conflict); i++){
      double x = node.robot_plan_map_[std::get<1>(conflict)].states[i].body.pose.position.x;
      double y = node.robot_plan_map_[std::get<1>(conflict)].states[i].body.pose.position.y;
      double z = node.robot_plan_map_[std::get<1>(conflict)].states[i].body.pose.position.z;
      std::vector<double> pos = {x, y, z};
      constraints.push_back(pos);
    }
  return;
}

void ConflictBasedSearch::updateSuccessors(GraphNode& node1, GraphNode& node2, 
              std::tuple<std::string, std::string, int, int>& curr_conflict){

  // std::tuple<std::string, std::string, int, int> curr_conflict = conflict_list.front();

  std::string r1;
  std::string r2;
  int t1;
  int t2;
  std::tie(r1, r2, t1, t2) = curr_conflict;

  std::tuple<std::string, std::string, int, int> r1_conflict = 
      std::make_tuple(r1, r2, t1, t2);

  std::tuple<std::string, std::string, int, int> r2_conflict = 
      std::make_tuple(r2, r1, t1, t2);

  std::vector<std::vector<double>> curr_constraints_1;
  getConstraintFromConflict(node1, r1_conflict, curr_constraints_1);
  node1.constraints_[r1].insert(node1.constraints_[r1].end(), curr_constraints_1.begin(), curr_constraints_1.end());
  // node1.constraints_[r1].push_back(curr_constraints_1);

  std::vector<std::vector<double>> curr_constraints_2;
  getConstraintFromConflict(node2, r2_conflict, curr_constraints_2);
  node2.constraints_[r2].insert(node2.constraints_[r2].end(), curr_constraints_2.begin(), curr_constraints_2.end());
  // node2.constraints_[r2].push_back(curr_constraints_2);
  // printConstraints(curr_constraints);
  return;
}


// Publishes The found Collision Free Path
void ConflictBasedSearch::publishPaths(GraphNode& node){
  for (const auto robot : node.robot_names_)
    robot_plan_pubs_[robot].publish(node.robot_plan_map_[robot]);
  return;
}

void ConflictBasedSearch::printConstraints(std::vector<std::vector<double>>& positions){
  for (size_t i = 0; i < positions.size(); ++i) {
        std::cout << "Position " << i + 1 << ": ";
        for (size_t j = 0; j < positions[i].size(); ++j) {
            std::cout << positions[i][j] << " ";
        }
        std::cout << std::endl;
    }
  return;
}

void ConflictBasedSearch::printVector(const std::vector<double>& vec) {
    std::cout << "Vector contents:";
    for (const auto& value : vec) {
        std::cout << " " << value;
    }
    std::cout << std::endl;
}

void ConflictBasedSearch::spin() {
  ros::Rate r(update_rate_);

  createServiceClients();
  ros::Duration timeout_duration(10.0);  // Specify the timeout duration (in seconds)

  // Enter main spin
  while (ros::ok()) {
    // Process callbacks
    ros::spinOnce();

    // Wait for the service to become available
    bool service_available = ros::service::waitForService("/robot_1/add_two_ints", timeout_duration);
    if (service_available){ 
      if (runOnce){ // Run This Node Once

        std::priority_queue<GraphNode*, std::vector<GraphNode*>, GraphNode::CompareCost> queue;

        //Generate Root Node, Priority Queue
        GraphNode* root = new GraphNode(robot_names_);
        requestInitialPaths(*root); 
        // std::cout << "Cost of Node" << root.cost << std::endl;
        // bool collide = doPlansCollide(root); // Update Collision List with Collisions
         // Convert these Conflicts into Nodes in the Tree

        queue.push(root);

        while(!queue.empty() || !goal_reached_){ // Main CBS Loop
          GraphNode* current_node_ = queue.top();
          queue.pop();
          std::vector<std::tuple<std::string, std::string, int, int>> conflict_list;

          if (current_node_->robot_plan_map_.empty()){
            //Request paths and Check for Collisions, Make sure to use new when generating new nodes
            requestInitialPaths(*current_node_);
            if (!doPlansCollide(*current_node_, conflict_list)){
              // publishPaths(*current_node_);
              // std::cout << conflict_list.size() <<std::endl;
              goal_reached_ = true;
            }
            else{
              ROS_INFO_STREAM("Pop off First Collision 1");
              // std::cout << conflict_list.size() <<std::endl;
              // std::tuple<std::string, std::string, int, int> curr_conflict_ = conflict_list.front()


              //Pop off the first element Collision, add both nodes to the queue
              break; 
            }
          }


          else{ //Check paths for Collisions, and Publish Paths if Not
            if (!doPlansCollide(*current_node_, conflict_list)){
              ROS_INFO_STREAM("FOUND COLLISION FREE PATHS");
              // publishPaths(*current_node_);
              // std::cout << conflict_list.size() <<std::endl;
              goal_reached_ = true;
            }
            else{
              ROS_INFO_STREAM("Pop off First Collision 2");
              // std::cout << conflict_list.size() <<std::endl;
              // std::tuple<std::string, std::string, int, int> curr_conflict = conflict_list.front();
              // std::vector<std::vector<double>> curr_constraints;
              // getConstraintFromConflict(*current_node_, curr_conflict, curr_constraints);
              // printConstraints(curr_constraints);

              
              GraphNode* successor_1 = new GraphNode(robot_names_);
              GraphNode* successor_2 = new GraphNode(robot_names_);
              successor_1->copyFrom(*current_node_);
              successor_2->copyFrom(*current_node_);

              std::tuple<std::string, std::string, int, int> curr_conflict = conflict_list.front();
              updateSuccessors(*successor_1, *successor_2, curr_conflict);
              // ROS_INFO_STREAM("Successfully Generated Successors");
              // std::cout <<" Successor 1: " << std::endl;
              // printConstraints(successor_1->constraints_["robot_1"]);
              // std::cout <<" Successor 2: " << std::endl;
              // printConstraints(successor_2->constraints_["robot_2"]);

              requestPath(*successor_1, curr_conflict, 0);
              requestPath(*successor_2, curr_conflict, 1);
              

              // printConstraints(successor_1->constraints_);
              queue.push(successor_1);
              queue.push(successor_2);

              
              //Pop off the first element Collision, add both nodes to the queue
              // Handle Collisions, Pop off First Collision, 
              //Add both Variations to Queue, Convert them to disticnt positions 
              break;
            }
          }
        }
        runOnce = false; // Make Sure the Node only spins once
      }
    }
    // publishPaths(current_node_);
    r.sleep();
  }
}


// For Debugging Purposes
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
