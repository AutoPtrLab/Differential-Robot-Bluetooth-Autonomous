// this file takes care of the navegation of the robot , it selects the decision basesd on the
// reactive sensors
//  and then the current goal of the robot
#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "distance_sensor.h"
#include "math_helper.h"
#include "robot_info.h"

// values to to calibrate the force camp system
#define KATRACTIVE 100720
#define KREPULSIVE 0
// minimum distance that interferes with the robot
#define MIN_DIST 100
// maximum distance we want to believe our sensors
#define MAX_DIST FLOAT_TO_Q16(0.8)

// maximum of waypoints in the global planner por the JPS
#define MAX_WAYPOINTS 36

// struct that returns the information for the controller
typedef struct VelData {
    q16_t linear_vel;
    q16_t ang_vel;
} VelData;

// the local planner of the robot, this takes the desired position (return from the global planner)
// and returns the ideal velocity to the controllers
// This function receives position via value since the data is small
VelData local_planer(RobotInfo *info, Pose2D desired_position);
// WARNING maybe pass the pointer if robot info grows too much

// The global planner of the robot , this function takes care of the global routes of the robot and
// also knows if the robot
//  is searching a certain path or just exploring
Pose2D global_planer(RobotInfo *info);
// updates the occupancy grid
void fill_occupancy_grid(const RobotInfo *info, const SensorDistances distances);
#endif // NAVIGATION_H
