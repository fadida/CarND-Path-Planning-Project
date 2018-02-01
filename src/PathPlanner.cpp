/*
 * PathPlanner.cpp
 *
 *  Created on: Feb 2, 2018
 *      Author: sfadida
 */

#include "PathPlanner.hpp"

using namespace std;

PathPlanner::PathPlanner(double speed_limit, double lane_width) :
speed_limit_(speed_limit), lane_width_(lane_width)
{

}

Path& PathPlanner::planPath(const Vehicle& vehicle, const vector<vector<double>>& sensor_fusion, size_t remaining_points_in_path)
{
  return path_;
}

