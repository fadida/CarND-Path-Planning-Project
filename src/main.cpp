#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

#include "PathPlanner.hpp"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// For converting mile per hour to meter per second.
constexpr double MS2MPS_CONVERSION_FACTOR = 2.23694;
constexpr double ms2mph(double ms) { return ms * MS2MPS_CONVERSION_FACTOR; }
constexpr double mph2ms(double mph) { return mph / MS2MPS_CONVERSION_FACTOR; }

////////////////////////////////////////////
///  Project parameters
//////////////////////////////////////////
constexpr double SPEED_LIMIT = mph2ms(50.0);
constexpr double MAX_ACCELARAION = 10;
constexpr double MAX_JERK = 10;
constexpr float LANE_WIDTH = 4;
constexpr size_t LANE_NUM = 3;
constexpr float DELTA_T = 0.02;
constexpr float SECONDS_AHEAD = 1;
constexpr double PLANNER_DELTA_T = 1.5;
constexpr size_t PLANNER_PATH_SIZE = 4;


constexpr size_t SENSOR_FUSION_CAR_ID_IDX = 0;
constexpr size_t SENSOR_FUSION_CAR_POS_X_IDX = 1;
constexpr size_t SENSOR_FUSION_CAR_POS_Y_IDX = 2;
constexpr size_t SENSOR_FUSION_CAR_VEL_X_IDX = 3;
constexpr size_t SENSOR_FUSION_CAR_VEL_Y_IDX = 4;
constexpr size_t SENSOR_FUSION_CAR_S_IDX = 5;
constexpr size_t SENSOR_FUSION_CAR_D_IDX = 6;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

/**
 * @brief Calculates the euclidean distance between two points.
 *
 * @param x1 The X coordinate of the 1st point.
 * @param y1 The Y coordinate of the 1st point.
 * @param x2 The X coordinate of the 2nd point.
 * @param y2 The Y coordinate of the 2nd point.
 * @return the distance.
 */
double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

/**
 * @brief Calculates the norm of a vector that starts from (0,0)
 *        and ends at (x, y)
 *
 * @param x The X coordinate.
 * @param y The Y coordinate.
 * @return The norm of the vector.
 */
double norm(double x, double y)
{
  return distance(0, 0, x, y);
}

/**
 * @brief Find the closest map waypoint to our current location.
 *
 * @param x       Current X position.
 * @param y       Current Y position.
 * @param maps_x  Vector of X position waypoints.
 * @param maps_y  Vector of X position waypoints.
 * @return The index of the closest waypoint
 */
int closestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen   = INFINITY;
	int closestWaypointIdx = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist  = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypointIdx = i;
		}

	}

	return closestWaypointIdx;
}

/**
 * @brief The next waypoint to get to from the car current location and heading.
 *
 * @param x       The X location of the car in map coordinates.
 * @param y       The Y location of the car in map coordinates.
 * @param theta   The car heading in map coordinates.
 * @param maps_x  Map waypoints in X coordinate.
 * @param maps_y  Map waypoints in Y coordinate.
 * @return The index of the next waypoint to drive to.
 */
int nextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypointIdx = closestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypointIdx];
	double map_y = maps_y[closestWaypointIdx];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypointIdx++;
  if (closestWaypointIdx == maps_x.size())
  {
    closestWaypointIdx = 0;
  }
  }

  return closestWaypointIdx;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = nextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

/**
 * @brief Predicts the obstacles locations in Frenet coordinates.
 *
 * @param[IN] sensor_fusion - Sensor fusion data from simulator
 * @param[IN] t_0 - The initial time to start predicting from. Represent the time offset for the present.
 * @param[IN] delta_t - The time difference between predictions.
 * @param[IN] num_points - The number of predictions in addition to the current state.
 * @param[IN] maps_x - The map waypoints in X coordinate.
 * @param[IN] maps_y - The map waypoints in Y coordinate.
 * @return a vector of predictions.
 */
vector<vector<Vehicle>> predict_obstacle_locations(const vector<vector<double>>& sensor_fusion, const double t_0, const double delta_t, const size_t num_points)
{
  vector<vector<Vehicle>> obstacle_predictions(sensor_fusion.size());

  for (size_t obstacle_idx = 0; obstacle_idx < sensor_fusion.size(); ++obstacle_idx) {
    Vehicle obstacle;

    obstacle.id = sensor_fusion[obstacle_idx][SENSOR_FUSION_CAR_ID_IDX];
    obstacle.s = sensor_fusion[obstacle_idx][SENSOR_FUSION_CAR_S_IDX];
    obstacle.d = sensor_fusion[obstacle_idx][SENSOR_FUSION_CAR_D_IDX];

    double obstacle_speed_x = sensor_fusion[obstacle_idx][SENSOR_FUSION_CAR_VEL_X_IDX];
    double obstacle_speed_y = sensor_fusion[obstacle_idx][SENSOR_FUSION_CAR_VEL_Y_IDX];
    obstacle.velocity = sqrt(obstacle_speed_x * obstacle_speed_x + obstacle_speed_y * obstacle_speed_y);


    // Use simple physical model in order to predict the car locations.
    obstacle.s = obstacle.s + obstacle.velocity * (t_0 - delta_t);
    for (size_t pred_idx = 0; pred_idx <= num_points; ++pred_idx)
    {
      // Assuming that obstacle acceleration is zero and that obstacles
      // don't change lanes in order to simplify predictions.
      obstacle.s += obstacle.velocity * delta_t;
      obstacle_predictions[obstacle_idx].push_back(obstacle);
    }

  }
  return obstacle_predictions;
}

int main() {
  uWS::Hub h;

  double planner_delta_t = PLANNER_DELTA_T;
  size_t planner_path_size = PLANNER_PATH_SIZE;



  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  double last_car_speed = 0;

  PathPlanner planner(SPEED_LIMIT, MAX_ACCELARAION, MAX_JERK, LANE_WIDTH, LANE_NUM, planner_delta_t, planner_path_size, max_s);

  size_t path_size = SECONDS_AHEAD / DELTA_T;

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          //////////////////////////////////////////////////////
          /// Define variables and extract data from simulator
          //////////////////////////////////////////////////////
          // j[1] is the data JSON object
          
      	  // Main car's localization Data
        	double car_x = j[1]["x"];
        	double car_y = j[1]["y"];
        	double car_s = j[1]["s"];
        	double car_d = j[1]["d"];
        	double car_yaw = j[1]["yaw"];
        	double car_speed = j[1]["speed"];

        	if (!last_car_speed)
        	{
        	  last_car_speed = mph2ms(car_speed);
        	}

        	// Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];

          size_t previous_path_size = previous_path_x.size();

          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          ///////////////////////////////////
          /// Get vehicle starting point
          ///////////////////////////////////
          // Starting point can be either the current vehicle position
          // or the end location of the previous path that was sent.

          // Previous car location, used to make a smoother spline.
          double prev_car_x = 0;
          double prev_car_y = 0;

          // Reference car location, used for XY calculations later on.
          double ref_car_x = car_x;
          double ref_car_y = car_y;
          double ref_car_yaw = car_yaw;
          double ref_t = 0;

          // Check if we have at least two points in previous path because
          // in order to get the correct heading we need at least two points.
          if (previous_path_size > 2)
          {
            prev_car_x = previous_path_x[previous_path_size - 2];
            prev_car_y = previous_path_y[previous_path_size - 2];

            // Update the reference location because the new reference is in the future.
            ref_car_x = previous_path_x[previous_path_size - 1];
            ref_car_y = previous_path_y[previous_path_size - 1];
            ref_car_yaw = atan2(ref_car_y - prev_car_y, ref_car_x - prev_car_x);

            ref_t = previous_path_size * DELTA_T;
          }
          else
          {
            // Use the car heading to take the car to its previous location
            prev_car_x = car_x - cos(car_yaw);
            prev_car_y = car_y - sin(car_yaw);
          }

          /////////////////////////////////////////////////////
          /// Call the path planner and let it do its magic
          /////////////////////////////////////////////////////
          vector<double> ref_car_fernet = getFrenet(ref_car_x, ref_car_y, ref_car_yaw, map_waypoints_x, map_waypoints_y);
          Vehicle vehicle = {Vehicle::NOT_OBSTACLE_VEHICLE_ID, ref_car_fernet[0], ref_car_fernet[1], last_car_speed};
          auto obstacle_predications = predict_obstacle_locations(sensor_fusion, ref_t, planner_delta_t, planner_path_size);
          //cout << "Calling path planner." << endl;
          Path car_path = planner.planPath(vehicle, obstacle_predications);
          //cout << "Calling path planner done." << endl;

          ////////////////////////
          /// Calculate spline
          ////////////////////////

          vector<double> spline_x;
          vector<double> spline_y;

          // Push previous car location and reference car location in order to make the spline smoother.
          spline_x.push_back(prev_car_x);
          spline_y.push_back(prev_car_y);

          spline_x.push_back(ref_car_x);
          spline_y.push_back(ref_car_y);

          // convert path points to Cartesian coordinates and add them to spline.
          for (size_t point_idx = 0; point_idx < car_path.size(); ++point_idx) {
            vector<double> cartesian = getXY(car_path.s[point_idx], car_path.d[point_idx], map_waypoints_s, map_waypoints_x, map_waypoints_y);

            spline_x.push_back(cartesian[0]);
            spline_y.push_back(cartesian[1]);
          }

          // In order to make math easier, shift all spline points to car coordinates

          // Cache cos and sin calculations.
          double shift_yaw = 0 - ref_car_yaw; // shift the heading to zero.
          double shift_cos = cos(shift_yaw);
          double shift_sin = sin(shift_yaw);
          for(size_t point_idx = 0; point_idx < spline_x.size(); ++point_idx)
          {
            double shift_x = spline_x[point_idx] - ref_car_x;
            double shift_y = spline_y[point_idx] - ref_car_y;

            spline_x[point_idx] = shift_x*shift_cos - shift_y*shift_sin;
            spline_y[point_idx] = shift_x*shift_sin + shift_y*shift_cos;

          }

          tk::spline spline;
          spline.set_points(spline_x, spline_y);


          ///////////////////////////////////////////////////////////////
          /// Generate next XY points from spline and path acceleration
          ///////////////////////////////////////////////////////////////


          // Set the next path to the previous path because we want to build the next path
          // on top of the previous one.
          vector<double> next_x_vals = previous_path_x;
          vector<double> next_y_vals = previous_path_y;

          size_t points_to_add = path_size - previous_path_size;

          //TODO: save last speed or calculate it, the current speed is not the one that will be at the end.
          double t_final = previous_path_size * DELTA_T;
          double next_v = last_car_speed;
          double next_x = 0;
          double next_y = spline(next_x);
          double next_yaw = 0;

          // Caching of cosine and sine values for map coordinates transformation.
          double cos_ref_yaw = cos(ref_car_yaw);
          double sin_ref_yaw = sin(ref_car_yaw);
          for (size_t point = 1; point <= points_to_add; ++point)
          {
            double prev_x = next_x;
            double prev_y = next_y;

            next_x += next_v*DELTA_T*cos(next_yaw);
            next_y = spline(next_x);
            next_v += car_path.acceleration*DELTA_T;
            // Negative velocities are invalid so set the minimum velocity possible to zero.
            if (next_v < 0)
            {
              next_v = 0;
            }

            next_yaw = atan2(next_y - prev_y, next_x - prev_x);

            // Transform the points back to map coordinates.
            double x = ref_car_x + next_x*cos_ref_yaw - next_y*sin_ref_yaw;
            double y = ref_car_y + next_x*sin_ref_yaw + next_y*cos_ref_yaw;

            next_x_vals.push_back(x);
            next_y_vals.push_back(y);
          }

          last_car_speed = next_v;

          ///////////////////////////////////////
          /// Send path to the simulator
          ///////////////////////////////////////

          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
