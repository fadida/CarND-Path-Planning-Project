#include "PathPlanner.hpp"

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

using namespace std;

constexpr size_t SENSOR_FUSION_CAR_ID_IDX = 0;
constexpr size_t SENSOR_FUSION_CAR_VEL_X_IDX = 3;
constexpr size_t SENSOR_FUSION_CAR_VEL_Y_IDX = 4;
constexpr size_t SENSOR_FUSION_CAR_S_IDX = 5;
constexpr size_t SENSOR_FUSION_CAR_D_IDX = 6;


/**
 * @brief Finds the maximum value index in a vector
 * @param vec The vector to search in.
 * @return the index of the maximum value element.
 */
template <typename T>
inline unsigned int arg_max(const vector<T>& vec)
{
  T max_val = vec[0];
  unsigned int max_idx = 0;

  for (size_t idx = 1; idx < vec.size(); ++idx)
  {
    if (vec[idx] > max_val)
    {
      max_val = vec[idx];
      max_idx = idx;
    }
  }

  return max_idx;
}

/**
 * @brief Symmetric clipping of number to the range [-bound, bound].
 * @param n - a number.
 * @param bound - a bound.
 * @return a number in range of [-bound, bound].
 */
template <typename T>
T clip(const T& n, const T& bound) {
  return max(-bound, min(n, bound));
}


PathPlanner::PathPlanner(double speed_limit, double max_acceleration, float max_jerk, float lane_width, size_t n_lanes,
                         double delta_t, size_t path_size, double max_s_val) :
speed_limit_(speed_limit), max_acceleration_(max_acceleration), max_jerk_(max_jerk), lane_width_(lane_width),
path_size_(path_size),
delta_t_(delta_t), safety_distance_(INFINITY), max_s_val_(max_s_val), n_lanes_(n_lanes)
{
  target_lane_ = PathPlanner::UNKNOWN_LANE;

  prev_acceleration_ = 0;

  // Reduce the limits in order to leave a safety zone for mistakes.
  speed_limit_ = 0.99 * speed_limit_;
  max_acceleration_ = 0.90 * max_acceleration_;
  max_jerk_ = 0.9 * max_jerk_;

  lane_velocities_ = vector<double>(n_lanes_, speed_limit_);

  critical_s_loc_ = -INFINITY;

  safety_time_ = delta_t_ / 2;
}

/**
 * @brief Create a path for the vehicle to follow.

 * @param vehicle               The vehicle initial position for path
 * @param obstacle_predictions  The obstacles
 * @return a path
 */
Path PathPlanner::planPath(const Vehicle& vehicle, const vector<vector<Vehicle>>& obstacle_predictions)
{
  unsigned int current_lane = getLaneIndex(vehicle.d);
  vector<double> lane_velocities = calculateLaneVelocities(vehicle, obstacle_predictions);
  updateLaneVelocities(lane_velocities);

  // initialize planner variables at first run.
  if (target_lane_  == UNKNOWN_LANE)
  {
    target_lane_ = current_lane;
    initial_d_pos_ = vehicle.d;
    initial_s_pos_ = vehicle.s;
    critical_s_loc_ = vehicle.s;
  }

  vector<bool> clear_lanes = checkIfPathsAreClear(vehicle, obstacle_predictions);

  // If lane is not clear (ie, we collide with car on it) reduce its
  // velocity as a penalty.
  for (size_t lane_idx = 0; lane_idx < n_lanes_; ++lane_idx)
  {
    if (!clear_lanes[lane_idx])
    {
      lane_velocities_[lane_idx] *= 1;
    }
  }


  if (!clear_lanes[current_lane])
  {
    lane_velocities_[current_lane] *= 0.7;
  }

  if (canMakePathDecision(vehicle))
  {
    initial_d_pos_ = vehicle.d;
    initial_s_pos_ = vehicle.s;

    vector<double> costs = lane_velocities_;
    // Current lane gets a bonus in order to prefer staying on lane
    // when velocities for all lanes are the same.
    costs[current_lane] *= 1.001;
    unsigned int fastest_lane = arg_max(costs);

    // Limit `target_lane` deviation to one lane.
    if (current_lane == fastest_lane)
    {
      target_lane_ = current_lane;
    }
    else if (current_lane > fastest_lane)
    {
      target_lane_ = current_lane - 1;
    }
    else
    {
      target_lane_ = current_lane + 1;
    }

    // Set the critical location: the point in which anther
    // decision will be made.
    // When we choose to keep lane the decision is immediate,
    // and when we choose to change lanes we wait for the end of the change.
    critical_s_loc_ = vehicle.s;
    if (target_lane_ != current_lane)
    {
      critical_s_loc_ += lane_velocities_[target_lane_] * delta_t_ * path_size_ ;
    }
  }
  else
  {
    // Override lane change decision when collision is detected.
    if (!clear_lanes[target_lane_])
    {
      target_lane_ = current_lane;
    }
  }

  return createPath(vehicle, target_lane_);
}


/**
 * @brief Calculates lane velocities based on obstacles velocity.
 *
 * @param vehicle       the vehicle data.
 * @param sensor_fusion the sensor fusion data.
 */
vector<double> PathPlanner::calculateLaneVelocities(const Vehicle& vehicle, const vector<vector<Vehicle>>& obstacle_predictions)
{
  vector<double> velocities(n_lanes_, speed_limit_);

  updateSafetyDistance(vehicle);
  for (size_t obstacle_idx = 0; obstacle_idx < obstacle_predictions.size(); ++obstacle_idx)
  {
    Vehicle obstacle = obstacle_predictions[obstacle_idx][0];

    if (isObstacleBehind(vehicle, obstacle))
    {
      continue;
    }

    if (!isObstacleTooClose(vehicle, obstacle))
    {
      continue;
    }

    unsigned int obstacle_lane = getLaneIndex(obstacle.d);
    velocities[obstacle_lane] = min(velocities[obstacle_lane], obstacle.velocity);
  }

  return velocities;
}



/**
 * @brief Gets the lane index (starting from zero) from Frenet d coordinate.
 * @param d Frenet d coordinate.
 * @return lane index.
 */
unsigned int PathPlanner::getLaneIndex(double d)
{
  return d / lane_width_;
}

/**
 * @brief Gets the D coordinate that correspond to the lane `lane_idx`.
 * @param lane_idx Lane index
 * @return Frenet d coordinate
 */
double PathPlanner::getDFromLane(unsigned int lane_idx)
{
  // Add an offset of half the lane width so that d coordinate
  // will be placed in the middle of the lane.
  return lane_width_ / 2 + lane_width_ * lane_idx;
}

/**
 * @brief Checks which paths the vehicle can take
 * @param vehicle The vehicle.
 * @param obstacle_predictions The obstacles predictions.
 * @return
 */
vector<bool> PathPlanner::checkIfPathsAreClear(const Vehicle& vehicle, const vector<vector<Vehicle>>& obstacle_predictions)
{
  vector<bool> is_clear(n_lanes_, true);
  unsigned int current_lane = getLaneIndex(vehicle.d);

  // Go over each lane
  for(unsigned int lane_idx = 0; lane_idx < n_lanes_; ++lane_idx)
  {

    // If the difference between the current path and the target path is more then one lane, skip the calculation
    // for this lane.
    if ((max(current_lane, lane_idx) - min(current_lane, lane_idx)) >= 2)
    {
      continue;
    }

    Path path = createDummyPath(vehicle, lane_idx);

    // Insert the present state into the path in order to check collisions in present state.
    path.d.insert(path.d.begin(), vehicle.d);
    path.s.insert(path.s.begin(), vehicle.s);

    Vehicle future = vehicle;
    for (size_t path_idx = 0; is_clear[lane_idx] && path_idx < path_size_ + 1; ++path_idx)
    {
      future.d  = path.d[path_idx];
      future.s  = path.s[path_idx];

      unsigned int future_vehicle_lane = getLaneIndex(future.d);

      for (vector<Vehicle> obstacle_pred : obstacle_predictions)
      {
        Vehicle obstacle = obstacle_pred[path_idx];
        unsigned int obstacle_lane = getLaneIndex(obstacle.d);
        if (obstacle_lane == future_vehicle_lane && isObstacleTooClose(future, obstacle))
        {
          is_clear[lane_idx] = false;
          break;
        }
      }
      // Vehicle has constant acceleration so we need to update the speed.
      future.velocity += path.acceleration * delta_t_;
      // acceleration is calculated for the next `delta_t_`, most likely that after
      // that it will become zero in order to preserve vehicle speed.
      path.acceleration = 0;
    }
  }

  return is_clear;
}

/**
 * @brief update the safety distance based on vehicle state.
 * @param vehicle The vehicle.
 */
void PathPlanner::updateSafetyDistance(const Vehicle& vehicle)
{
  safety_distance_ = safety_time_ * vehicle.velocity;
}

/**
 * @brief Checks if two vehicles distance is less then the safety_distance.
 * @param vehicle
 * @param obstacle
 * @return True/False
 */
bool PathPlanner::isObstacleTooClose(const Vehicle& vehicle, const Vehicle& obstacle)
{
  double delta_s = obstacle.s - vehicle.s;
  double delta_d = obstacle.d - vehicle.d;
  double safety_distance = safety_time_ * max(vehicle.velocity, obstacle.velocity);

  return sqrt(delta_s * delta_s + delta_d * delta_d) < safety_distance;
}

/**
 * @brief Checks if `obstacle` is directly behind `vehicle`
 * @param vehicle
 * @param obstacle
 * @return True/False
 */
bool PathPlanner::isObstacleBehind(const Vehicle& vehicle, const Vehicle& obstacle)
{
  unsigned int vehicle_lane = getLaneIndex(vehicle.d);
  unsigned int obstacle_lane = getLaneIndex(obstacle.d);

  return /*vehicle_lane == obstacle_lane &&*/ obstacle.s < vehicle.s;
}

/**
 * @brief Creates a path for a specific target lane.
 * @param vehicle The vehicle
 * @param target_lane_idx The lane the path will lead to.
 * @return A path.
 */
Path PathPlanner::createPath(const Vehicle& vehicle, unsigned int target_lane_idx)
{
  Path path(path_size_);

  path.acceleration = getPathAccelaration(vehicle, target_lane_idx);

  unsigned int current_lane = getLaneIndex(vehicle.d);

  double s = vehicle.s;
  double d = getDFromLane(current_lane);

  // When current lane and source lane are different those
  // values are used in order to lead the vehicle for it current d location `src_d`
  // to the target lane d value `dst_d`.
  double src_d = initial_d_pos_;//vehicle.d;
  double dst_d = getDFromLane(target_lane_idx);

  // Assuming constant acceleration the maximum change in s will be `delta_s`.
  double delta_s = vehicle.velocity * delta_t_ + 0.5 * path.acceleration * delta_t_ * delta_t_;
  double delta_d = dst_d - src_d;

  double dst_s = initial_s_pos_ + delta_s * path_size_ * 0.8;

  for (size_t path_idx = 0; path_idx < path_size_; ++path_idx) {
    s += delta_s;

    if (current_lane != target_lane_idx)
    {
      // When planning lane change, advance in a linear line in Frenet space.
      if (s <= dst_s)
      {
        d = src_d + delta_d*(s / dst_s);
      }
      else
      {
        d = dst_d;
        current_lane = target_lane_idx;
      }
    }

    path.s[path_idx] = s;
    path.d[path_idx] = d;
  }

  return path;
}

/**
 * @brief Calculate the path acceleration.
 * @param vehicle
 * @param target_lane_idx
 * @return acceleration
 */
double PathPlanner::getPathAccelaration(const Vehicle& vehicle, unsigned int target_lane_idx)
{
  // Calculate acceleration based on current lane velocity
  double acceleration = getTimeDerivative(vehicle.velocity, lane_velocities_[target_lane_idx]);
  acceleration = clip(acceleration, max_acceleration_);

  double jerk = getTimeDerivative(prev_acceleration_, acceleration);
  jerk = clip(jerk, max_jerk_);

  if (fabs(jerk) == max_jerk_)
  {
    acceleration = jerk * delta_t_;
  }

  prev_acceleration_ = acceleration;

  return acceleration;
}

/**
 * @brief approximate derivative using the planner `delta_t_`.
 * @param x_initial
 * @param x_final
 * @return
 */
double PathPlanner::getTimeDerivative(double x_initial, double x_final)
{
  // This formula is not really a derivative because delta_t is not much smaller then 1.
  double delta_x = x_final - x_initial;
  return delta_x / delta_t_;
}

/**
 * @brief Check if a new decision can be made.
 * @param vehicle
 * @return True/False
 */
bool PathPlanner::canMakePathDecision(const Vehicle& vehicle)
{
  if (vehicle.s > critical_s_loc_)
  {
    return true;
  }

  // Check for wrap-around
  if (critical_s_loc_ > 0.9 * max_s_val_ && vehicle.s < 0.1 * max_s_val_ )
  {
    // Fix warp-around
    return (max_s_val_ + vehicle.s) > critical_s_loc_;
  }

  return false;
}

/**
 * @brief Update the lane velocities using moving average.
 * @param lane_velocities
 */
void PathPlanner::updateLaneVelocities(const std::vector<double>& lane_velocities)
{
  double alpha = 0.7;
  for (unsigned int lane_idx = 0; lane_idx < n_lanes_; ++lane_idx)
  {
    lane_velocities_[lane_idx] = (1-alpha) * lane_velocities_[lane_idx] + alpha * lane_velocities[lane_idx];
  }
}

/**
 * @brief Create a new path without side-effects.
 * @param vehicle
 * @param target_lane_idx
 * @return the path
 */
Path PathPlanner::createDummyPath(const Vehicle& vehicle, unsigned int target_lane_idx)
{
  double prev_acceleration = prev_acceleration_;
  Path path = createPath(vehicle, target_lane_idx);
  prev_acceleration_ = prev_acceleration;

  return path;
}
