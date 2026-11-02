#include "navigation.h"
#include "distance_sensor.h"

#include "math_helper.h"
#include "robot_info.h"
#include "trigonometry_helper.h"
#include <stdbool.h>
// this is a potencial_field_planner  (since we only have 3 sensors and the HC-SR04 cannot give the
// distance in a rect angle, it detects in a cone ,we asume the possible obstacle is perpedicular to
// de sensor)

q16_t linear_vel;
q16_t ang_vel;

q16_t ang;
VelData local_planer(const Pose2D robot_pos_in, const Pose2D desired_position) {

    ang = robot_pos_in.ang;
    Pose2D robot_pos = (Pose2D){(Vec2){0, 0}, 0};
    Vec2 total_force = VEC_ZERO();
    Vec2 goal_vec = VEC_sub(&desired_position.pos, &robot_pos.pos);
    // changing the goal vec from global coordinates to local
    goal_vec = (Vec2){.x = Q16_mult(goal_vec.x, Q16_cos(robot_pos.ang)) +
                           Q16_mult(goal_vec.y, Q16_sin(robot_pos.ang)),
                      .y = Q16_mult(goal_vec.x, -Q16_sin(robot_pos.ang)) +
                           Q16_mult(goal_vec.y, Q16_cos(robot_pos.ang))};
    Vec2 atractive_force = VEC_scalar_mut(&goal_vec, KATRACTIVE);
    total_force = VEC_sum(&total_force, &atractive_force);

    SensorDistances distances = ds_get_dist();

    // since we only have 3 possible "obstacles" we check the 3 distances
    // repulsive_force = k_repulsive * obstacle_vector / distance**2, since we dont have the
    // obstacle_vector we suppose that the "obstacle" is perpendicular to where the distance sensors
    // are , front = {1,0},left = {0,1},right = {0,-1}, and since that is already the norm vec we
    // just have to divide by the dist one time

    //  left obstacle
    // if (distances.left_dist > MIN_DIST && distances.left_dist < MAX_DIST) {
    //     Vec2 repulsive_force =
    //         VEC_scalar_mut(&(Vec2){.x = 0, .y = -1}, Q16_div(KREPULSIVE, distances.left_dist));
    //     total_force = VEC_sum(&total_force, &repulsive_force);
    // }

    // // front obstacle
    // if (distances.front_dist > MIN_DIST && distances.front_dist < MAX_DIST) {
    //     Vec2 repulsive_force =
    //         VEC_scalar_mut(&(Vec2){.x = -1, .y = 0}, Q16_div(KREPULSIVE, distances.front_dist));
    //     total_force = VEC_sum(&total_force, &repulsive_force);
    // }
    // // right  obstacle
    // if (distances.right_dist > MIN_DIST && distances.right_dist < MAX_DIST) {
    //     Vec2 repulsive_force =
    //         VEC_scalar_mut(&(Vec2){.x = 0, .y = 1}, Q16_div(KREPULSIVE, distances.right_dist));
    //     total_force = VEC_sum(&total_force, &repulsive_force);
    // }
    VelData data;
    data.linear_vel = Q16_mult(total_force.x, 0);

    // data.ang_vel = 0;
    data.ang_vel = Q16_mult(total_force.y, 0);
    // data.linear_vel = total_force.x * 5;
    // data.linear_vel = INT_TO_Q16(1) >> 4;
    q16_t max_linear = INT_TO_Q16(1);  // Límite de 1.0 m/s
    q16_t max_angular = INT_TO_Q16(2); // Límite de giro

    if (data.linear_vel > max_linear)
        data.linear_vel = max_linear;
    if (data.linear_vel < -max_linear)
        data.linear_vel = -max_linear;

    if (data.ang_vel > max_angular)
        data.ang_vel = max_angular;
    if (data.ang_vel < -max_angular)
        data.ang_vel = -max_angular;

    linear_vel = data.linear_vel;
    ang_vel = data.ang_vel;
    return data;
}
Pose2D global_planer(const RobotInfo info) {
    // variable holding the waypoints of the exploring route
    static Vec2 waypoints[15];
    switch (info.nav_state) {

    case NAV_EXPLORING:

        break;
    case NAV_GUIDED:

        break;
    }
    return (Pose2D){(Vec2){.x = INT_TO_Q16(5), .y = 0}, info.pose.ang};
}
