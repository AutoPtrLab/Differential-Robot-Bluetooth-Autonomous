// the struct containing all the info the robot needs to have

#ifndef ROBOT_INFO_H
#define ROBOT_INFO_H

#include "math_helper.h"
#include "trigonometry_helper.h"
#include <stdbool.h>

extern volatile bool stop_robot;

// Enum that represents the current status of the robot, so the global planner knows how to act
typedef enum {
    NAV_HOME,
    NAV_SCANNING,
    NAV_WAYPOINT,
    NAV_EXPLORING,
    NAV_GUIDED,
    NAV_STOP
} NavigationState;

typedef struct {
    Vec2 pos;     // two dimensional vector
    ang_unit ang; // BAM angle (wraps around automatically)
} Pose2D;
typedef struct {
    Pose2D pose;
    q16_t ang_vel;
    NavigationState nav_state;
    bool reached_waypoint; // indicates if the robot has reached the last waypoitn the global
    // planner has set for the local planner

    bool going_home;
    bool is_stucked;
} RobotInfo;

static inline RobotInfo default_robot_info(void) {
    RobotInfo r;
    r.pose.pos.x = 0;
    r.pose.pos.y = 0;
    r.ang_vel = 0;

    r.is_stucked = false;

    r.going_home = false;
    r.pose.ang = 0;
    r.nav_state = NAV_STOP;
    r.reached_waypoint = false;
    return r;
}

#endif // ROBOT_INFO_H
