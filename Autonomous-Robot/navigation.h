// this file takes care of the navegation of the robot , it selects the decision basesd on the
// reactive sensors
//  and then the current goal of the robot
#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "math_helper.h"
#include "robot_info.h"

// values to to calibrate the force camp system
#define KATRACTIVE 131072
#define KREPULSIVE 65536
// minimum distances that interferes with the robot
#define MIN_DIST 1000

// this distance is the maximum dist we want the obstacle to be able to interfere with the robot
#define MAX_DIST 7000
// struct that returns the information for the controller
typedef struct VelData {
    q16_t linear_vel;
    q16_t ang_vel;
} VelData;

// the local planner of the robot, this takes the desired position (return from the global planner)
// and returns the ideal velocity to the controllers
// This function receives position via value since the data is small
VelData local_planer(Pose2D const robot_pos, Pose2D const desired_position);
// WARNING maybe pass the pointer if robot info grows too much

// The global planner of the robot , this function takes care of the global routes of the robot and
// also knows if the robot
//  is searching a certain path or just exploring
Pose2D global_planer(RobotInfo const info);
#endif // NAVIGATION_H
