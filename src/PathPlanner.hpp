#ifndef SRC_PATHPLANNER_HPP_
#define SRC_PATHPLANNER_HPP_

#include <iostream>
#include <vector>
#include <cmath>

/**
 * Represents the path that the path planner
 * will return.
 */
struct Path
{
  std::vector<double> s;
  std::vector<double> d;

  double acceleration;

  Path(size_t size): s(size), d(size), acceleration(0)
  {}

  size_t size()
  {
    return s.size();
  }

};

/**
 * Vehicle data
 */
struct Vehicle
{
  static constexpr size_t NOT_OBSTACLE_VEHICLE_ID = 0xFFFFFFFF;
  unsigned int id;
  double s;
  double d;
  double velocity; /* Vehicle velocity in meter per second. */
};

class PathPlanner {
 public:
  static constexpr size_t UNKNOWN_LANE = 0xFFFFFFFF;

  PathPlanner(double speed_limit, double max_acceleration, float max_jerk, float lane_width, size_t n_lanes, double delta_t, size_t path_size, double max_s_val = INFINITY);

  Path planPath(const Vehicle& vehicle, const std::vector<std::vector<Vehicle>>& obstacle_predictions);

 private:

  unsigned int getLaneIndex(double d);

  double getDFromLane(unsigned int lane_idx);

  std::vector<double> calculateLaneVelocities(const Vehicle& vehicle, const std::vector<std::vector<Vehicle>>& obstacle_predictions);

  std::vector<bool> checkIfPathsAreClear(const Vehicle& vehicle, const std::vector<std::vector<Vehicle>>& obstacle_predictions);

  void updateSafetyDistance(const Vehicle& vehicle);

  bool isObstacleTooClose(const Vehicle& vehicle, const Vehicle& obstacle);

  bool isObstacleBehind(const Vehicle& vehicle, const Vehicle& obstacle);

  Path createPath(const Vehicle& vehicle, unsigned int target_lane_idx);

  Path createDummyPath(const Vehicle& vehicle, unsigned int target_lane_idx);

  void updateLaneVelocities(const std::vector<double>& lane_velocities);

  double getPathAccelaration(const Vehicle& vehicle, unsigned int target_lane_idx);

  bool canMakePathDecision(const Vehicle& vehicle);

  double getTimeDerivative(double x_initial, double x_final);


  /* The point that the vehicle needs to pass in order for the planner
   * to recalculate the path. */
  double critical_s_loc_;
  /* Maximum s value we can reach (in a cyclic track)*/
  double max_s_val_;
  /* Speed limit in meter per second. */
  double speed_limit_;
  /* The maximum acceleration allowed. */
  double max_acceleration_;
  /* The maximum jerk allowed. */
  double max_jerk_;
  /* The previous calculated acceleration. */
  double prev_acceleration_;
  /* Width of the lane. */
  float lane_width_;
  /* vehicle s position at the last decision.*/
  double initial_s_pos_;
  /* vehicle d position at the last decision.*/
  double initial_d_pos_;
  /* The number of points in generated path. */
  size_t path_size_;
  /* Number of lanes in track. */
  size_t n_lanes_;
  /* The lane velocities. */
  std::vector<double> lane_velocities_;

  /* The time in seconds to keep from the other vehicles.
   * Turns into the safety distance by multiplying by the vehicle speed.*/
  float safety_time_;
  /* Safety distance*/
  double safety_distance_;
  /* Delta time. */
  double delta_t_;
  /* The lane the planner choose. */
  unsigned int target_lane_;

};

#endif /* SRC_PATHPLANNER_HPP_ */
