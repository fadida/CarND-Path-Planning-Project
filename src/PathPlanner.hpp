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

  double accelaration;

  size_t length()
  {
    return s.size();
  }
};

/**
 * Vehicle data
 */
struct Vehicle
{
  double s;
  double d;
  double speed; /* Vehicle speed in mph */
};


class PathPlanner {
 public:
  static constexpr size_t UNKNOWN_LANE = 0xFFFFFFFF;

  PathPlanner(float speed_limit, float lane_width, float dt, float seconds_ahead);

  Path& planPath(const Vehicle& vehicle, const std::vector<std::vector<double>>& sensor_fusion, size_t remaining_points_in_path);


 private:
  /**
   * @brief Gets the lane index (starting from zero) from Frenet d coordinate.
   * @param d Frenet d coordinate.
   * @return lane index.
   */
  size_t getLaneIndex(double d);

  /**
   * @brief Gets the D coordinate that correspond to the lane `lane_idx`.
   * @param lane_idx Lane index
   * @return Frenet d coordinate
   */
  double getDFromLane(size_t lane_idx);


  /* Path that the planner has created,
   * represented as a vector of a double vector when
   * the outer vector is for coordinate (s or d) and
   * the inner vector is for the actual points. */
  Path path_;
  /* Speed limit in mph. */
  float speed_limit_;
  /* Width of the lane. */
  float lane_width_;
  /* Current lane the can is on. */
  size_t curr_lane_;
  /* Time delta used in the simulator path. */
  float dt_;
  /* How many seconds ahead to plan. */
  float seconds_ahead_;
  /* Path size that planner will create. */
  size_t path_size_;
};

#endif /* SRC_PATHPLANNER_HPP_ */
