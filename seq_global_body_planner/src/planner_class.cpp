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

State PlannerClass::randomState(const PlannerConfig &planner_config) {
  double x_min, x_max, y_min, y_max;
  getMapBounds(planner_config, x_min, x_max, y_min, y_max);

  State q;

  // Lognormal distribution sampling
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);

  q.pos[0] = (x_max - x_min) * (double)rand() / RAND_MAX + x_min;
  q.pos[1] = (y_max - y_min) * (double)rand() / RAND_MAX + y_min;
  q.pos[2] = planner_config.h_nom + getTerrainZFromState(q, planner_config);

  double phi = (2.0 * M_PI) * (double)rand() / RAND_MAX;
  double v = (*vel_distribution_)(generator);
  v = std::min(std::max(v, 0.0), planner_config.v_max);
  q.vel[0] = v * cos(phi);
  q.vel[1] = v * sin(phi);
  q.vel[2] = getDzFromState(q, planner_config);

  return q;
}

std::vector<int> PlannerClass::neighborhoodN(State q, int N) const {
  std::priority_queue<Distance, std::vector<Distance>, std::greater<Distance>>
      closest;

  std::unordered_map<int, State>::const_iterator itr;
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

int PlannerClass::getNearestNeighbor(State q) const {
  std::vector<int> closest_q = neighborhoodN(q, 1);
  return closest_q.front();
}
