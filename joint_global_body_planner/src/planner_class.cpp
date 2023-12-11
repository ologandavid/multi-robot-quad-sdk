#include "global_body_planner/planner_class.h"

#include <chrono>
#include <queue>

using namespace planning_utils;

PlannerClass::PlannerClass(int direction, const PlannerConfig &planner_config) {
  direction_ = direction;

  double vel_mean = planner_config.v_nom;
  double vel_sigma = (planner_config.v_max - planner_config.v_nom) / 3.0;
  double vel_mean_log =
      std::log(vel_mean * vel_mean /
               std::sqrt(vel_mean * vel_mean + vel_sigma * vel_sigma));
  double vel_sigma_log =
      std::sqrt(std::log(1 + (vel_sigma * vel_sigma) / (vel_mean * vel_mean)));
  vel_distribution_ = std::make_shared<std::lognormal_distribution<double>>(
      vel_mean_log, vel_sigma_log);
}

MultipleState PlannerClass::randomState(const PlannerConfig &planner_config) {
  double x_min, x_max, y_min, y_max;
  getMapBounds(planner_config, x_min, x_max, y_min, y_max);

  State rob1_q;
  State rob2_q;
  MultipleState q;

  // Lognormal distribution sampling
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);

  rob1_q.pos[0] = (x_max - x_min) * (double)rand() / RAND_MAX + x_min;
  rob1_q.pos[1] = (y_max - y_min) * (double)rand() / RAND_MAX + y_min;
  rob1_q.pos[2] = planner_config.h_nom + getTerrainZFromState(rob1_q, planner_config);

  double phi = (2.0 * M_PI) * (double)rand() / RAND_MAX;
  double v = (*vel_distribution_)(generator);
  v = std::min(std::max(v, 0.0), planner_config.v_max);
  rob1_q.vel[0] = v * cos(phi);
  rob1_q.vel[1] = v * sin(phi);
  rob1_q.vel[2] = getDzFromState(rob1_q, planner_config);

  rob2_q.pos[0] = (x_max - x_min) * (double)rand() / RAND_MAX + x_min;
  rob2_q.pos[1] = (y_max - y_min) * (double)rand() / RAND_MAX + y_min;
  rob2_q.pos[2] = planner_config.h_nom + getTerrainZFromState(rob2_q, planner_config);

  phi = (2.0 * M_PI) * (double)rand() / RAND_MAX;
  v = (*vel_distribution_)(generator);
  v = std::min(std::max(v, 0.0), planner_config.v_max);
  rob2_q.vel[0] = v * cos(phi);
  rob2_q.vel[1] = v * sin(phi);
  rob2_q.vel[2] = getDzFromState(rob2_q, planner_config);

  q.rob1 = rob1_q;
  q.rob2 = rob2_q;

  return q;
}

std::vector<int> PlannerClass::neighborhoodN(MultipleState q, int N) const {
  std::priority_queue<Distance, std::vector<Distance>, std::greater<Distance>>
      closest;

  std::unordered_map<int, MultipleState>::const_iterator itr;
  for (itr = vertices.begin(); itr != vertices.end(); itr++) {
    closest.push(std::make_pair(stateDistance(q, itr->second), itr->first));
  }

  if (N > closest.size()) N = closest.size();
  std::vector<int> neighbors;
  for (int i = 0; i < N; i++) {
    neighbors.push_back(closest.top().second);
    closest.pop();
  }

  return neighbors;
}
/*
std::vector<int> PlannerClass::neighborhoodDist(State q, double dist) const {
  std::vector<int> neighbors;

  std::unordered_map<int, State>::const_iterator itr;
  for (itr = vertices.begin(); itr != vertices.end(); itr++) {
    if ((stateDistance(q, itr->second) <= dist) &&
        stateDistance(q, itr->second) > 0) {
      neighbors.push_back(itr->first);
    }
  }

  return neighbors;
}
*/
int PlannerClass::getNearestNeighbor(MultipleState q) const {
  std::vector<int> closest_q = neighborhoodN(q, 1);
  return closest_q.front();
}
