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
 * @brief helper function for printing vectors.
 * @param vec The vector to print
 */
template <typename T>
inline void print_vector(const vector<T>& vec)
{
  cout << '[';
  for (size_t idx = 0; idx < vec.size(); ++idx)
  {
    cout << vec[idx];
    if (idx < vec.size() - 1)
    {
      cout << ", ";
    }
  }
  cout << ']';
}

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
 * Path planner constructor
 * @param speed_limit
 * @param max_acceleration
 * @param max_jerk
 * @param lane_width
 * @param path_size
 * @param safety_time
 */
PathPlanner::PathPlanner(double speed_limit, double max_acceleration, float max_jerk, float lane_width, size_t path_size, float safety_time) :
speed_limit_(speed_limit), max_acceleration_(max_acceleration), max_jerk_(max_jerk), lane_width_(lane_width), path_size_(path_size), safety_time_(safety_time)
{
  target_lane_ = PathPlanner::UNKNOWN_LANE;
  prev_state_ = state_ = PATH_PLANNER_STATE_KEEP_LANE;
  // Initialize path vector
  path_ = {vector<double>(path_size_), vector<double>(path_size_), 0.0};

  delta_t_ = 0.02;

  speed_limit_ = 0.99 * speed_limit_;
  speed_max_threshold_ = 0.99 * speed_limit_;
  speed_min_threshold_ = 0.7 * speed_limit_;

  target_vehicle_id_ = 0;
}

/**
 * @brief Create a path for the vehicle to follow.
 * @details
 *        Steps:
 *        1. Calculate velocities for each lane.
 *        2. Create path points.
 *        3. Calculate path acceleration.
 * @param vehicle        The vehicle initial position for path
 * @param sensor_fusion  Sensor fusion data.
 * @return a path
 */
Path PathPlanner::planPath(const Vehicle& vehicle, const vector<vector<double>>& sensor_fusion)
{
  cout << "s=" << vehicle.s << ", d=" << vehicle.d << endl;
  vector<double> lane_velocities = getLaneVelocities(vehicle, sensor_fusion);
  unsigned int current_lane = getLaneIndex(vehicle.d);
  unsigned int fastest_lane = arg_max(lane_velocities);

  cout << "Lane velocities: ";
  print_vector(lane_velocities);
  cout << endl;

  // Decide on a path based on lane velocities and vehicle velocity.
  if (current_lane == target_lane_)
  {
    int delta_lane = fastest_lane - current_lane;
    if (1 || delta_lane == 0)
    {
      // We are on the fastest lane, keep driving on this lane.
      state_ = PATH_PLANNER_STATE_KEEP_LANE;
      target_lane_ = current_lane;
    }
    else if (delta_lane < -1)
    {
      state_ = PATH_PLANNER_STATE_SWITCH_LANE_LEFT;
      target_lane_ = current_lane - 1;
    }
    else
    {
      state_ = PATH_PLANNER_STATE_SWITCH_LANE_RIGHT;
      target_lane_ = current_lane + 1;
    }
  }
  else if (target_lane_ == PathPlanner::UNKNOWN_LANE)
  {
    target_lane_ = current_lane;
  }
  Path path = generatePathPoints(vehicle, lane_velocities);

  cout << "Prev_State=" << prev_state_ << ", State=" << state_ << endl;
  cout << "curr_lane="<< current_lane << ", target_lane=" << target_lane_ << endl;
  // Calculate acceleration based on current lane velocity
  double delta_velocity = lane_velocities[current_lane] - vehicle.velocity;

  cout << "lane_vel="<<lane_velocities[current_lane]<<", vehicle speed=" << vehicle.velocity << ", delta=" << delta_velocity << endl;

  path.acceleration = delta_velocity / delta_t_;

  // Check that acceleration match max acceleration and jerk requirements.
  if (path.acceleration > max_acceleration_)
  {
    path.acceleration = max_acceleration_;
  }
  else if (path.acceleration < -1 * max_acceleration_)
  {
    path.acceleration = -max_acceleration_;
  }

  cout << "accel = " << path.acceleration << endl;

  return path;
}

Path PathPlanner::generatePathPoints(const Vehicle& vehicle, const vector<double>& lane_velocities)
{
  Path path = {vector<double>(path_size_), vector<double>(path_size_), 0.0};


  double d = vehicle.d;
  double s = vehicle.s;
  double delta_d = 0;
  double delta_s = 0;

  double target_d = getDFromLane(target_lane_);

  switch (state_)
  {
    case PATH_PLANNER_STATE_KEEP_LANE:
      delta_d = (target_d - vehicle.d) / path_size_;
      delta_s = speed_limit_;
      prev_state_ = PATH_PLANNER_STATE_KEEP_LANE;
      break;

    case PATH_PLANNER_STATE_SWITCH_LANE_LEFT:
      delta_d = (target_d - vehicle.d) / path_size_;
      delta_s = 0.8 * speed_limit_;
      cout << "Changing lane left - d_d=" << delta_d << ", d_s=" << delta_s << endl;
      prev_state_ = PATH_PLANNER_STATE_SWITCH_LANE_LEFT;
      break;

    case PATH_PLANNER_STATE_SWITCH_LANE_RIGHT:
      delta_d = (target_d - vehicle.d) / path_size_;
      delta_s = 0.8 * speed_limit_;
      cout << "Changing lane right - d_d=" << delta_d << ", d_s=" << delta_s << endl;
      prev_state_ = PATH_PLANNER_STATE_SWITCH_LANE_RIGHT;
      break;

    default:
      break;
  }

  for (size_t path_idx = 0; path_idx < path_size_; ++path_idx) {
    d += delta_d;
    s += delta_s;

    path.d[path_idx] = d;
    path.s[path_idx] = s;
  }

  return path;

}

/**
 * @brief Gets lane velocities based on other vehicles velocity.
 *
 * @details
 *
 *
 * @param[IN] vehicle       the vehicle data.
 * @param[IN] sensor_fusion the sensor fusion data.
 *
 * @return a vector of velocities.
 */
vector<double> PathPlanner::getLaneVelocities(const Vehicle& vehicle, const vector<vector<double>>& sensor_fusion)
{
  vector<double> velocities(3, speed_limit_);

  double s_ref = vehicle.s;
  double saftey_distance = safety_time_ * vehicle.velocity;

  cout << "safety distance=" << saftey_distance << endl;
  cout << "############################ getting velocities #######################" << endl;
  for (size_t vehicle_idx = 0; vehicle_idx < sensor_fusion.size(); ++vehicle_idx)
  {
    cout << "processing obstacle (" << sensor_fusion[vehicle_idx][SENSOR_FUSION_CAR_ID_IDX] << ")" << endl;
    double obstacle_s = sensor_fusion[vehicle_idx][SENSOR_FUSION_CAR_S_IDX];
    double obstacle_d = sensor_fusion[vehicle_idx][SENSOR_FUSION_CAR_D_IDX];

    double obstacle_speed_x = sensor_fusion[vehicle_idx][SENSOR_FUSION_CAR_VEL_X_IDX];
    double obstacle_speed_y = sensor_fusion[vehicle_idx][SENSOR_FUSION_CAR_VEL_Y_IDX];

    // Maybe ignore safety distance when vehicle is behind us and not in our lane.
#if 0
    // Ignore obstacles behind the vehicle.
    if (obstacle_s < s_ref)
    {
      cout << "Ignore due to s location (obs_s=" << obstacle_s << ")"<< endl;
      continue;
    }
#endif
    // Ignore obstacles outside the safety distance.
    double delta_s = obstacle_s - vehicle.s;
    double delta_d = obstacle_d - vehicle.d;
    if (sqrt(delta_s * delta_s + delta_d * delta_d) > saftey_distance)
    {
      cout << "Ignore due to safety distance." << endl;
      continue;
    }

    double obstacle_speed = sqrt(obstacle_speed_x * obstacle_speed_x + obstacle_speed_y * obstacle_speed_y);
    unsigned int obstacle_lane = getLaneIndex(obstacle_d);

    cout << "obs_lane=" << obstacle_lane << ", speed=" << obstacle_speed << endl;

    velocities[obstacle_lane] = min(velocities[obstacle_lane], obstacle_speed);
  }
  cout << "############################ done getting velocities #######################" << endl;

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
