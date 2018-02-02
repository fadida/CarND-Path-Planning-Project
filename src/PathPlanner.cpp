#include "PathPlanner.hpp"

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

using namespace std;

PathPlanner::PathPlanner(float speed_limit, float lane_width, float dt, float seconds_ahead) :
speed_limit_(speed_limit), lane_width_(lane_width), dt_(dt), seconds_ahead_(seconds_ahead)
{
  curr_lane_ = PathPlanner::UNKNOWN_LANE;
  path_size_ = seconds_ahead / dt; /* Floor division because `path_size_` is size_t. */

  // Initialize path vector
  path_ = {vector<double>(path_size_), vector<double>(path_size_), 0.0};
}


Path& PathPlanner::planPath(const Vehicle& vehicle, const vector<vector<double>>& sensor_fusion, size_t remaining_points_in_path)
{
#if 0
  size_t points_processed = 0;
  size_t next_point_idx = remaining_points_in_path;

  // The remaining points in path will be zero most likely when the
  // planner will run for the first time.
  if (remaining_points_in_path)
  {
    points_processed = path_.length() - remaining_points_in_path;

    // rotate vector so processed points will be at the end of the vector,
    // we do so in order to replace them with new points.
    rotate(path_.d.begin(), path_.d.begin() + points_processed, path_.d.end());
    rotate(path_.s.begin(), path_.s.begin() + points_processed, path_.s.end());

  }

#endif

  double d = vehicle.d;
  double s_0 = vehicle.s;
  for (size_t path_idx = 0; path_idx < path_size_; ++path_idx) {
    path_.d[path_idx] = d;
    path_.s[path_idx] = s_0 + path_idx * 0.37;
  }

  return path_;
}

/**
 * @brief Gets the lane index (starting from zero) from Frenet d coordinate.
 * @param d Frenet d coordinate.
 * @return lane index.
 */
size_t PathPlanner::getLaneIndex(double d)
{
  return d / lane_width_;
}

/**
 * @brief Gets the D coordinate that correspond to the lane `lane_idx`.
 * @param lane_idx Lane index
 * @return Frenet d coordinate
 */
double PathPlanner::getDFromLane(size_t lane_idx)
{
  // Add an offset of half the lane width so that d coordinate
  // will be placed in the middle of the lane.
  return lane_width_ / 2 + lane_width_ * lane_idx;
}

