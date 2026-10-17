// this file takes care of the navegation of the robot , it selects the decision basesd on the
// reactive sensors
//  and then the current goal of the robot
#ifndef NAVIGATION_H
#define NAVIGATION_H


#include "math_helper.h"
#include "robot_info.h"
#define HC_THRESHOLD 250 // the minimun distance in mm that the sensor recognizes as an obstacle
//maximum int32 possible, since is not a possible angle
// it represent that the final orentation is not necessary
#define NO_ANGLE  4294967296


//struct that return the information for the controller
typedef struct VelData{
   q16_t linear_vel;
   q16_t ang_vel;
} VelData;

// the local planner of the robot, this takes the desired position (return from the global planner)
// and returns the ideal velocity to the controllers
// This function receives position via value since the data is small
// If the final angle is not important in the desried_position the angle should be
// NO_ANGLE
VelData local_planer( Pose2D const robot_pos, Pose2D const desired_position);
//WARNING maybe pass the pointer if robot info grows too much
// 
//The global planner of the robot , this function takes care of the global routes of the robot and also knows if the robot
// is searching a certain path or just exploring
Pose2D global_planer(RobotInfo const info);
#endif // NAVIGATION_H
