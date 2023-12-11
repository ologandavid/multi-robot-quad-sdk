#include "global_body_planner/gbpl.h"

using namespace planning_utils;

GBPL::GBPL() {}

int GBPL::connect(PlannerClass &T, MultipleState s, const PlannerConfig &planner_config,
                  int direction, ros::Publisher &tree_pub) {
  // Find nearest neighbor
  flipDirection(s.rob1);
  flipDirection(s.rob2);
  int s_near_index = T.getNearestNeighbor(s);
  MultipleState s_near = T.getVertex(s_near_index);
  MultipleStateActionResult result;
  int connect_result = TRAPPED;
  // Try to connect to nearest neighbor, add to graph if REACHED or ADVANCED
  int connect_result_rob1 =
      attemptConnect(s_near.rob1, s.rob1, result.rob1, planner_config, direction);
  int connect_result_rob2 =
      attemptConnect(s_near.rob2, s.rob2, result.rob2, planner_config, direction);
  if ((connect_result_rob1 != TRAPPED) && (connect_result_rob2 != TRAPPED)) {
    int s_new_index = T.getNumVertices();
    MultipleState newVertexState;
    newVertexState.rob1 = result.rob1.s_new;
    newVertexState.rob2 = result.rob2.s_new;
    MultipleAction newVertexAction;
    newVertexAction.rob1 = result.rob1.a_new;
    newVertexAction.rob2 = result.rob2.a_new;
    double edge_cost = result.rob1.length + result.rob2.length;
    T.addVertex(s_new_index, newVertexState);
    T.addEdge(s_near_index, s_new_index, edge_cost);
    T.addAction(s_new_index, newVertexAction);
    if (connect_result_rob1 == connect_result_rob2) {
      connect_result = connect_result_rob1;
    }
    else {
      connect_result = ADVANCED;
    }

#ifdef VISUALIZE_TREE
    publishStateActionPair(s_near, result.a_new, s, planner_config,
                           tree_viz_msg_, tree_pub);
#endif
  }

  return connect_result;
}

std::vector<MultipleAction> GBPL::getActionSequenceReverse(PlannerClass &T,
                                                   std::vector<int> path) {
  // Assumes that actions are synched with the states at which they are executed
  // (opposite of the definition in RRT)
  std::vector<MultipleAction> action_sequence;
  for (int i = 0; i < path.size() - 1; ++i) {
    action_sequence.push_back(T.getAction(path.at(i)));
  }
  return action_sequence;
}

void GBPL::postProcessPath(std::vector<MultipleState> &state_sequence,
                           std::vector<MultipleAction> &action_sequence,
                           const PlannerConfig &planner_config) {
  auto t_start = std::chrono::steady_clock::now();

  // Initialize first and last states
  MultipleState s_goal = state_sequence.back();
  MultipleState s = state_sequence.front();
  MultipleState s_next;
  MultipleState dummy;
  MultipleAction a_new;
  MultipleAction a_next;

  // Initialize state and action sequences
  std::vector<MultipleState> new_state_sequence;
  new_state_sequence.push_back(s);
  std::vector<MultipleAction> new_action_sequence;
  MultipleAction temp_anew;
  path_length_ = 0;

  // Iterate until the goal has been added to the state sequence
  while (s != s_goal) {
    // Make a copy of the original state and action sequences
    std::vector<MultipleState> state_sequence_copy = state_sequence;
    std::vector<MultipleAction> action_sequence_copy = action_sequence;

    // Start at the back of the sequence
    s_next = state_sequence_copy.back();
    a_next = action_sequence_copy.back();
    MultipleState old_state;
    MultipleAction old_action;
    MultipleStateActionResult result;

    // Try to connect to the last state in the sequence
    // if unsuccesful remove the back and try again until successful or no
    // states left
    while ((attemptConnect(s.rob1, s_next.rob1, result.rob1, planner_config, FORWARD) !=
            REACHED) && (attemptConnect(s.rob2, s_next.rob2, result.rob2, planner_config, FORWARD) !=
            REACHED) &&
           (s != s_next)) {
      old_state = s_next;
      old_action = a_next;
      state_sequence_copy.pop_back();
      action_sequence_copy.pop_back();
      s_next = state_sequence_copy.back();
      a_next = action_sequence_copy.back();
    }

    // If a new state was found add it to the sequence, otherwise add the next
    // state in the original sequence
    if (s != s_next) {
      new_state_sequence.push_back(s_next);
      temp_anew.rob1 = result.rob1.a_new;
      temp_anew.rob2 = result.rob2.a_new;
      new_action_sequence.push_back(temp_anew);
      path_length_ += (result.rob1.length + result.rob2.length);
      s = s_next;

    } else {
      new_state_sequence.push_back(old_state);
      new_action_sequence.push_back(old_action);

      // Recompute path length
      isValidStateActionPair(old_state.rob1, old_action.rob1, result.rob1, planner_config);
      isValidStateActionPair(old_state.rob2, old_action.rob2, result.rob2, planner_config);
      path_length_ += (result.rob1.length + result.rob2.length);
      s = old_state;
    }
  }

  // Replace the old state and action sequences with the new ones
  state_sequence = new_state_sequence;
  action_sequence = new_action_sequence;

  auto t_end = std::chrono::steady_clock::now();
  std::chrono::duration<double> processing_time = t_end - t_start;
}

void GBPL::extractPath(PlannerClass &Ta, PlannerClass &Tb,
                       std::vector<MultipleState> &state_sequence,
                       std::vector<MultipleAction> &action_sequence,
                       const PlannerConfig &planner_config) {
  // Get both paths, remove the back of path_b and reverse it to align with path
  // a
  std::vector<int> path_a = pathFromStart(Ta, Ta.getNumVertices() - 1);
  std::vector<int> path_b = pathFromStart(Tb, Tb.getNumVertices() - 1);

  std::reverse(path_b.begin(), path_b.end());
  std::vector<MultipleAction> action_sequence_b = getActionSequenceReverse(Tb, path_b);
  for (int i = 0; i < action_sequence_b.size(); i++) {
    flipDirection(action_sequence_b[i].rob1);
    flipDirection(action_sequence_b[i].rob2);
  }
  path_b.erase(path_b.begin());

  state_sequence = getStateSequence(Ta, path_a);
  std::vector<MultipleState> state_sequence_b = getStateSequence(Tb, path_b);
  for (int i = 0; i < state_sequence_b.size(); i++) {
    flipDirection(state_sequence_b[i].rob1);
    flipDirection(state_sequence_b[i].rob2);
  }
  state_sequence.insert(state_sequence.end(), state_sequence_b.begin(),
                        state_sequence_b.end());

  action_sequence = getActionSequence(Ta, path_a);
  action_sequence.insert(action_sequence.end(), action_sequence_b.begin(),
                         action_sequence_b.end());

  // Post process to reduce the path length
  postProcessPath(state_sequence, action_sequence, planner_config);
}

void GBPL::extractClosestPath(PlannerClass &Ta, const MultipleState &s_goal,
                              std::vector<MultipleState> &state_sequence,
                              std::vector<MultipleAction> &action_sequence,
                              const PlannerConfig &planner_config) {
  std::vector<int> path_a = pathFromStart(Ta, Ta.getNearestNeighbor(s_goal));
  state_sequence = getStateSequence(Ta, path_a);
  action_sequence = getActionSequence(Ta, path_a);
  postProcessPath(state_sequence, action_sequence, planner_config);
}

int GBPL::findPlan(const PlannerConfig &planner_config, MultipleState s_start,
                   MultipleState s_goal, std::vector<MultipleState> &state_sequence,
                   std::vector<MultipleAction> &action_sequence,
                   ros::Publisher &tree_pub) {
  // Perform validity checking on start and goal states
  if ((!isValidState(s_start.rob1, planner_config, LEAP_STANCE)) || (!isValidState(s_start.rob2, planner_config, LEAP_STANCE))) {
    return INVALID_START_STATE;
  }
  State s_startR1 = s_start.rob1;
  State s_startR2 = s_start.rob2;
  State s_goalR1 = s_goal.rob1;
  State s_goalR2 = s_goal.rob2;
  // Set goal height to nominal distance above terrain
  s_goal.rob1.pos[2] =
      getTerrainZFromState(s_goal.rob1, planner_config) + planner_config.h_nom;
  s_goal.rob2.pos[2] =
      getTerrainZFromState(s_goal.rob2, planner_config) + planner_config.h_nom;
  if ((!isValidState(s_goal.rob1, planner_config, LEAP_STANCE)) || (!isValidState(s_goal.rob2, planner_config, LEAP_STANCE))) {
    return INVALID_GOAL_STATE;
  }
  if ((poseDistance(s_start.rob1, s_goal.rob1) <= 1e-1) || (poseDistance(s_start.rob2, s_goal.rob2) <= 1e-1)) {
    return INVALID_START_GOAL_EQUAL;
  }

  // Initialize timing information
  auto t_start_total_solve = std::chrono::steady_clock::now();
  auto t_start_current_solve = std::chrono::steady_clock::now();
  int result;

  PlannerClass Ta(FORWARD, planner_config);
  PlannerClass Tb(REVERSE, planner_config);
  Ta.init(s_start);
  flipDirection(s_goal.rob1);
  flipDirection(s_goal.rob2);
  Tb.init(s_goal);

#ifdef VISUALIZE_TREE
  tree_viz_msg_.markers.clear();
  tree_viz_msg_.markers.resize(1);
#endif
  //Ask David what to do here
  anytime_horizon_init = 0.01;
  anytime_horizon =
      std::max(poseDistance(s_start, s_goal) / planning_rate_estimate,
               anytime_horizon_init);

  while (ros::ok()) {
    auto t_current = std::chrono::steady_clock::now();
    std::chrono::duration<double> total_elapsed =
        t_current - t_start_total_solve;
    std::chrono::duration<double> current_elapsed =
        t_current - t_start_current_solve;

#ifndef VISUALIZE_TREE
    if (total_elapsed.count() >= planner_config.max_planning_time) {
      elapsed_to_first_ = total_elapsed;
      num_vertices_ = (Ta.getNumVertices() + Tb.getNumVertices());
      break;
    }

    if (current_elapsed.count() >= anytime_horizon) {
      auto t_start_current_solve = std::chrono::steady_clock::now();
      anytime_horizon = anytime_horizon * horizon_expansion_factor;
      Ta = PlannerClass(FORWARD, planner_config);
      Tb = PlannerClass(REVERSE, planner_config);
      tree_viz_msg_.markers.clear();
      Ta.init(s_start);
      Tb.init(s_goal);
      continue;
    }
#endif

    // Generate random s
    MultipleState s_rand = Ta.randomState(planner_config);

    if (isValidState(s_rand.rob1, planner_config, LEAP_STANCE) && isValidState(s_rand.rob2, planner_config, LEAP_STANCE)) {
      if (extend(Ta, s_rand, planner_config, FORWARD, tree_pub) != TRAPPED) {
        MultipleState s_new = Ta.getVertex(Ta.getNumVertices() - 1);

#ifdef VISUALIZE_TREE
        Action a_new = Ta.getAction(Ta.getNumVertices() - 1);
        State s_parent =
            Ta.getVertex(Ta.getPredecessor(Ta.getNumVertices() - 1));
        publishStateActionPair(s_parent, a_new, s_rand, planner_config,
                               tree_viz_msg_, tree_pub);
#endif

        if (connect(Tb, s_new, planner_config, FORWARD, tree_pub) == REACHED) {
          goal_found = true;
          auto t_end = std::chrono::steady_clock::now();
          elapsed_to_first_ = t_end - t_start_total_solve;
          path_length_ = Ta.getGValue(Ta.getNumVertices() - 1) +
                         Tb.getGValue(Tb.getNumVertices() - 1);
          break;
        }
      }
    }

    s_rand = Tb.randomState(planner_config);

    if (isValidState(s_rand.rob1, planner_config, LEAP_STANCE) && isValidState(s_rand.rob2, planner_config, LEAP_STANCE)) {
      if (extend(Tb, s_rand, planner_config, FORWARD, tree_pub) != TRAPPED) {
        MultipleState s_new = Tb.getVertex(Tb.getNumVertices() - 1);

#ifdef VISUALIZE_TREE
        Action a_new = Tb.getAction(Tb.getNumVertices() - 1);
        State s_parent =
            Tb.getVertex(Tb.getPredecessor(Tb.getNumVertices() - 1));
        publishStateActionPair(s_parent, a_new, s_rand, planner_config,
                               tree_viz_msg_, tree_pub);
#endif

        if (connect(Ta, s_new, planner_config, FORWARD, tree_pub) == REACHED) {
          goal_found = true;
          auto t_end = std::chrono::steady_clock::now();
          elapsed_to_first_ = t_end - t_start_total_solve;
          path_length_ = Ta.getGValue(Ta.getNumVertices() - 1) +
                         Tb.getGValue(Tb.getNumVertices() - 1);
          break;
        }
      }
    }
  }

  num_vertices_ = (Ta.getNumVertices() + Tb.getNumVertices());

  if (goal_found == true) {
    extractPath(Ta, Tb, state_sequence, action_sequence, planner_config);
    result = VALID;
  } else {
    extractClosestPath(Ta, s_goal, state_sequence, action_sequence,
                       planner_config);
    result = (state_sequence.size() > 1) ? VALID_PARTIAL : UNSOLVED;
  }

  auto t_end = std::chrono::steady_clock::now();
  elapsed_total_ = t_end - t_start_total_solve;

  path_duration_ = 0.0;
  int rob1_path_dur = 0;
  int rob2_path_dur = 0;
  for (MultipleAction a : action_sequence) {
    rob1_path_dur += (a.rob1.t_s_leap + a.rob1.t_f + a.rob1.t_s_land);
    rob2_path_dur += (a.rob2.t_s_leap + a.rob2.t_f + a.rob2.t_s_land);
  }
  path_duration_ = rob1_path_dur + rob2_path_dur;
  double r1_dist_to_goal_ = poseDistance(s_goal.rob1, state_sequence.back().rob1);
  double r2_dist_to_goal_ = poseDistance(s_goal.rob2, state_sequence.back().rob2);
  return result;
}
