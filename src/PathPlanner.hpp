#ifndef SRC_PATHPLANNER_HPP_
#define SRC_PATHPLANNER_HPP_

#include <iostream>
#include <vector>

/**
 * Represents the path that the path planner
 * will return.
 */
struct Path
{
  std::vector<double> s;
  std::vector<double> d;

  double acceleration;

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
  unsigned int id;
  double s;
  double d;
  double velocity; /* Vehicle velocity in meter per second. */
};


class PathPlanner {
 public:
  static constexpr size_t UNKNOWN_LANE = 0xFFFFFFFF;

  PathPlanner(double speed_limit, double max_acceleration, float max_jerk, float lane_width, size_t path_size = 3, float safety_time = 0.5);

  Path planPath(const Vehicle& vehicle, const std::vector<std::vector<double>>& sensor_fusion);

 private:

  /**
   * @brief Gets the lane index (starting from zero) from Frenet d coordinate.
   * @param d Frenet d coordinate.
   * @return lane index.
   */
  unsigned int getLaneIndex(double d);

  /**
   * @brief Gets the D coordinate that correspond to the lane `lane_idx`.
   * @param lane_idx Lane index
   * @return Frenet d coordinate
   */
  double getDFromLane(unsigned int lane_idx);

  Path generatePathPoints(const Vehicle& vehicle, const std::vector<double>& lane_velocities);

  std::vector<double> getLaneVelocities(const Vehicle& vehicle, const std::vector<std::vector<double>>& sensor_fusion);

  /* Path that the planner has created,
   * represented as a vector of a double vector when
   * the outer vector is for coordinate (s or d) and
   * the inner vector is for the actual points. */
  Path path_;
  /* Speed limit in meter per second. */
  double speed_limit_;
  double max_acceleration_;
  double max_jerk_;
  /* Width of the lane. */
  float lane_width_;
  /* The number of points in generated path. */
  size_t path_size_;
  /* The time in seconds to keep from the other vehicles.
   * Turns into the safety distance by multiplying by the vehicle speed.*/
  float safety_time_;

  double speed_max_threshold_;
  double speed_min_threshold_;

  double delta_t_;

  /////////////////////////////////////
  /// FSM parameters.
  ////////////////////////////////////

  /*
   * States for Path Planner FSM.
   */
  enum PlannerState
  {
    PATH_PLANNER_STATE_KEEP_LANE,           // Keep lane state.
    PATH_PLANNER_STATE_MATCH_SPEED,         // Match vehicle speed state.
    PATH_PLANNER_STATE_SWITCH_LANE_LEFT,    // Switch lane left state.
    PATH_PLANNER_STATE_SWITCH_LANE_RIGHT,   // Switch lane right state.
  } state_;

  PlannerState prev_state_;

  unsigned int target_lane_;
  unsigned int target_vehicle_id_;

};

#endif /* SRC_PATHPLANNER_HPP_ */
