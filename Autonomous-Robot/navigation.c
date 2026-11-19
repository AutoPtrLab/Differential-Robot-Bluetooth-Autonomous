
#include "navigation.h"
#include "distance_sensor.h"
#include "machine/_limits.h"
#include "motor.h"
#include "msp430f5529.h"
#include "occupancy_grid.h"

#include "math_helper.h"
#include "robot_info.h"
#include "trigonometry_helper.h"
#include <stdbool.h>
#include <stdint.h>
// this is a potencial_field_planner  (since we only have 3 sensors and the HC-SR04 cannot give the
// distance in a rect angle, it detects in a cone ,we asume the possible obstacle is perpedicular to
// de sensor)

q16_t linear_vel;
q16_t ang_vel;
q16_t ang;

static void nav_stop_routine(void);
static void frontier_searcher(void);
VelData local_planer(const Pose2D robot_pos, const Pose2D desired_position) {

    // ang = robot_pos_in.ang;
    // Pose2D robot_pos = (Pose2D){(Vec2){0, 0}, 0};
    Vec2 total_force = VEC_ZERO();
    Vec2 goal_vec = VEC_sub(&desired_position.pos, &robot_pos.pos);
    // if we have already reached the desired position in a certain range
    if (VEC_length_sq(&goal_vec) < 4000000) {
        return (VelData){0, 0};
    }
    Vec2 atractive_force = VEC_scalar_Q16_mut(&goal_vec, KATRACTIVE);
    total_force = VEC_sum(&total_force, &atractive_force);

    // now we search around the grid to make our potencial field
    for (int i = -5; i <= 5; i++) {
        for (int j = -5; j <= 5; j++) {
            // we dont check the robot current cell(division by zero)
            if (j == 0 && i == 0) {
                continue;
            }

            // we pass the positions to the grid index
            int16_t rx = (int16_t)(robot_pos.pos.x >> 13) + HALF_SIZE;
            int16_t ry = (int16_t)(robot_pos.pos.y >> 13) + HALF_SIZE;
            // Dentro del bucle (-3 a +3 celdas = ±37.5 cm):
            uint8_t value = get_grid_from_bresenham(rx + j, ry + i);
            // if the value is zero we dont really have to add any force
            if (value <= 7) {
                continue;
            }
            int32_t dist_sq = j * j + i * i;
            Vec2 repulsive_vec = {.x = (-j * INT_TO_Q16(1)) / dist_sq,
                                  .y = (-i * INT_TO_Q16(1)) / dist_sq};
            repulsive_vec = VEC_scalar_Q16_mut(&repulsive_vec, value * KREPULSIVE);
            total_force = VEC_sum(&total_force, &repulsive_vec);
        }
    }

    // we change the global vectors to local coordinates of the robot
    total_force = (Vec2){.x = Q16_mult(total_force.x, Q16_cos(robot_pos.ang)) +
                              Q16_mult(total_force.y, Q16_sin(robot_pos.ang)),
                         .y = Q16_mult(total_force.x, -Q16_sin(robot_pos.ang)) +
                              Q16_mult(total_force.y, Q16_cos(robot_pos.ang))};
    VelData data;
    data.linear_vel = Q16_TO_INT(total_force.x) + 1;
    data.ang_vel = Q16_TO_INT(total_force.y) + 1;
    // linear_vel = data.linear_vel;
    // ang_vel = data.ang_vel;
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
    case NAV_STOP:
        nav_stop_routine();
        break;
    }
    return (Pose2D){(Vec2){.x = INT_TO_Q16(2), .y = 0}, info.pose.ang};
}
static void nav_stop_routine() {

    mt_stop();

    P2DIR &= ~BIT1;

    P2REN |= BIT1;
    // pullup
    P2OUT |= BIT1;

    while (true) {

        if ((P2IN & BIT1) == 0) {
            __delay_cycles(10000);
            send_mapgrid();
        }
        if (stop_robot == false) {

            break;
        }
    }
}
static void frontier_searcher(void) {
    // RAM LEFT :1520 bytes of ram left
    //
    // The idea of the frontier searcher is first using a BFS (Breadth First Search) begginning in
    // the robot  to search for the first unexplored tile of our map (the map is scaled so we
    // consider the 4 tiles in the uint16_t as a unique node) . When we find the nearest
    // unexplored tile, since we dont have enough memory we cannot save the parents to have the
    // complete path, we will use a JPS (Jump Point Search) to obtain the path waypoints to the
    // desired position (ideally the waypoints should be more or less on a straight line)
}
void fill_occupancy_grid(const Pose2D *pose, const SensorDistances distances) {

    // Firstly lets calculate the position in global coordinates of the obstacles, since the
    // distance sensors
    //  are position in the front {1,0} , the left { 0,1 } and the right { 0 , -1} in local
    //  coord of the robot we have to change the reference of the obstacles from local to global

    ang_unit robot_ang = pose->ang;
    Vec2 robot_vec = pose->pos;

    if (distances.front_dist < INT_TO_Q16(2)) {
        Vec2 front_obstacle =
            VEC_sum(&robot_vec, &(Vec2){Q16_mult(distances.front_dist, Q16_cos(robot_ang)),
                                        Q16_mult(distances.front_dist, Q16_sin(robot_ang))});
        obstacle_update(&robot_vec, &front_obstacle);
    } else {

        Vec2 front_obstacle = VEC_sum(&robot_vec, &(Vec2){Q16_mult(MAX_DIST, Q16_cos(robot_ang)),
                                                          Q16_mult(MAX_DIST, Q16_sin(robot_ang))});
        no_obstacle_update(&robot_vec, &front_obstacle);
    }
    if (distances.left_dist < INT_TO_Q16(2)) {
        Vec2 left_obstacle =
            VEC_sum(&robot_vec, &(Vec2){Q16_mult(distances.left_dist, -Q16_sin(robot_ang)),
                                        Q16_mult(distances.left_dist, Q16_cos(robot_ang))});
        obstacle_update(&robot_vec, &left_obstacle);
    } else {

        Vec2 left_obstacle = VEC_sum(&robot_vec, &(Vec2){Q16_mult(MAX_DIST, -Q16_sin(robot_ang)),
                                                         Q16_mult(MAX_DIST, Q16_cos(robot_ang))});
        no_obstacle_update(&robot_vec, &left_obstacle);
    }
    if (distances.right_dist < INT_TO_Q16(2)) {
        Vec2 right_obstacle =
            VEC_sum(&robot_vec, &(Vec2){Q16_mult(distances.right_dist, Q16_sin(robot_ang)),
                                        Q16_mult(-distances.right_dist, Q16_cos(robot_ang))});
        obstacle_update(&robot_vec, &right_obstacle);
    } else {
        Vec2 right_obstacle = VEC_sum(&robot_vec, &(Vec2){Q16_mult(MAX_DIST, Q16_sin(robot_ang)),
                                                          Q16_mult(-MAX_DIST, Q16_cos(robot_ang))});
        no_obstacle_update(&robot_vec, &right_obstacle);
    }
}
