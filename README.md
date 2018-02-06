# CarND-Path-Planning-Project

In this project I implemented a path planner for a vehicle driving
on a 3 lanes highway with other vehicles.


This code is running in Udacity third term simulator.

## Project Structure
This project is composed of two main parts: Path planner and Simulator interface.

## Simulator interface
The simulator interface is implemented in the `main.cpp` file and is working by using the following flow: []

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
The program turns the vehivcle reative position into Frenet coordinates and push it to the path planner along with sensor fusion data.
The Path planner returns general path points in Frenet coordinates and the car accelaretion for this path.

### Build spline
After the path planner returns a path, the program will build a spline using those points and also the car relative and previous location (in order to make the path smoother). The spline is calculated on Vehicle coordinates and not on map coordinates. The reason I used Vehicle coordiantes is that in order to make a spline the X axes points need to be sorted from low to high. In map coordinates there is no promise that this condition will hold, ecepcialy in a round track.
For that I use Vehicle coordinates with path that is relativly short in relation to track in order make sure this condition is setisfied.

### Generate next XY points
After building the spline the program will check how many points it need to generate based on predefined future horizon of 1 sec (each point represents 0.02 seconds).
The generated points will be placed in a way that will make the car accelerate at roughly the acceleration that the path planner outputted and will follow the path of the spline. 

## The Path Planner
