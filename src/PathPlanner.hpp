/*
 * PathPlanner.hpp
 *
 *  Created on: Feb 2, 2018
 *      Author: sfadida
 */

#ifndef SRC_PATHPLANNER_HPP_
#define SRC_PATHPLANNER_HPP_

#include <iostream>
#include <vector>

struct Path
{
  std::vector<double> s;
  std::vector<double> d;

  size_t length()
  {
    return s.size();
  }
};

struct Vehicle
{
  double s;
  double d;
  double speed;
};


class PathPlanner {
 public:
  PathPlanner(double speed_limit, double lane_width);

  Path& planPath(const Vehicle& vehicle, const std::vector<std::vector<double>>& sensor_fusion, size_t remaining_points_in_path);


 private:
  /* The path that the planner has created,
   * represented as a vector of a double vector when
   * the outer vector is for coordinate (s or d) and
   * the inner vector is for the actual points. */
  Path path_;
  double speed_limit_;
  double lane_width_;

};

#endif /* SRC_PATHPLANNER_HPP_ */
