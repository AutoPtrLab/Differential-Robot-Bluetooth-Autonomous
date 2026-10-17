#include "navigation.h"
#include "config.h"
#include "distance_sensor.h"

#include "robot_info.h"
#include <stdbool.h>
//Firstly we should ajust the angle to get to the posiiton , then we should
// reach the position avoiding obstacles , and then in the desired_position
// we adjust the angle
VelData local_planer(const Pose2D robot_pos, const Pose2D desired_position){


        
}

Pose2D global_planer(const RobotInfo info){
    
}
