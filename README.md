# CarND-Path-Planning-Project

In this project I implemented a path planner for a vehicle driving
on a 3 lanes highway with other vehicles.


This code is running in Udacity third term simulator.

## Project Structure
This project is composed of two main parts: Path planner and Simulator interface.

## Simulator interface
The simulator interface is implemented in the `main.cpp` file and is working by using the following flow:

simulator update -> get info from simulator -> define vehicle position (real or relative) -> call Path planner -> build spline -> generate next (x,y) points for simulator -> send points to simulator

### Simulator Update
Every granularity period the simulator sends an 'Telemetry' request to the program.
This request contains the following data:
* Vehicle position in map coordinates.
* Vehicle yaw in relation to map. 
* Vehicle position in Frenet coordinates.
* Vehicle velocity in mph.
* List of previous path points (in both X and Y) that vehicle has yet to visit. 
* The end of path location in Frenet coordinates. 
* Sensor fusion data on other vehicles.

### Define vehicle position
In order to avoid throwing previous work, the program sets the vehicle relative
position to be at the end of the previous path if there is such path.
If there is no previous path to use, the program will use the vehicle current location. 
In this step I also calculate the vehicle previous location in order to use it to smooth up the planned path and also in order to determine the car future heading which will be used in the next steps.

### Call path planner
The program turns the vehivcle reative position into Frenet coordinates and push it to the path planner along with the obstacle location.
The obstacle locations are predicted from the sensor fusion data by assuming
that the obstacles drive only on their lane and with constant velocity.
The Path planner returns general path points in Frenet coordinates and the car accelaretion for this path.

### Build spline
After the path planner returns a path, the program will build a spline using those points and also the car relative and previous location (in order to make the path smoother). The spline is calculated on Vehicle coordinates and not on map coordinates. The reason I used Vehicle coordiantes is that in order to make a spline the X axes points need to be sorted from low to high. In map coordinates there is no promise that this condition will hold, ecepcialy in a round track.
For that I use Vehicle coordinates with path that is relativly short in relation to track in order make sure this condition is setisfied.

### Generate next XY points
After building the spline the program will check how many points it need to generate based on predefined future horizon of 1 sec (each point represents 0.02 seconds).
The generated points will be placed in a way that will make the car accelerate at roughly the acceleration that the path planner outputted and will follow the path of the spline. 

## The Path Planner
The path planner is implemented in `PathPlanner.cpp`.
The path planner works with Frenet coordinates only and uses the following algorithm (implemented in `planPath()`):
1. Calculate lane velocities from obstacles.
2. Check which paths can be taken from the current location.
3. If current lane is blocked by an obstacle, decrease the lane velocity.
4. Check if it is time to make a new path decision.
4.1 If it is time 
4.1.1 Find the lane with the highest cost
4.1.2 Choose the path closest to the choosen lane.
4.1.3 Define the place the s location that vehicle needs to be for the next decision to take place.
4.2 Id it is not the time
4.2.2 Change path decision to the current path if the path has obstacles.
5. Create the designated path.

### Lane velocities calculation
The path planner uses the obstacles location and velocities in order to 
calculate the lane velocities vector by choosing the minimum from the speed limit and all the relevant obstacles in that lane.
The relevant obsticles are obsticles that are placed in front or to the side of the vehicle and are found withing the radius of the vehicle safety distance.

After the current state lane velocities are calculated, the planner updates the lane velocities by using a moving average (geometric average with the previous state) in order to filter noises.

### Check which paths can be taken from the current location
Some paths are blocked by obsticles. This step function is to find out
which paths are blocked.
In this step the path planner checks for every path point generated within a path that the vehicle don't have any obstacles in its lane within a radious of safety distance.
If the above condition holds, the path is clear.

### Decision time calculation
When the planner makes a decision, there needs to be a cool down in which the planner can't make new decisions. The reason for this is to make the path as smooth as posible and also to avoid interrupting a lane change.

The decision time is defined by a critical s location.
When vehicle passes this location a decision can be made.
For lane change, the distance is calculated approximately at middle of the maneuver and for keeping a lane the decision time is instant.

### Choosing the fastest lane
The planner is using the path velocities as its cost function. The lane with the highest velocity will be the one that will be picked.
In order to make sure that the vehicle won't change lanes in a case of a tie, the current lane gets an additional velocity boost of 0.001 of its current velocity.
In addition, if the fastest lane is not adjacent to the current lane, the closest adjacent lane will be picked instead.

### Backing out on lane change
When the planner is in the middle of a lane change, the planner checks if the path is still clear and if its blocked, the lane change is cancelled.
When cancelling the lane change the planner is not updating the planned decision time in order to avoid a "war" between this step and the decision step. 