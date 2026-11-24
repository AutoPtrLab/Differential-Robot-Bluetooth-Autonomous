
#include "navigation.h"
#include "distance_sensor.h"
#include "motor.h"
#include "msp430f5529.h"
#include "occupancy_grid.h"
#include "search_helper.h"

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

#define MAX_GOAL_DIST FLOAT_TO_Q16(0.5)
static uint16_t find_patrol_goal(uint8_t curr_x, uint8_t curr_y);
static void nav_stop_routine(RobotInfo *info);
VelData local_planer(RobotInfo *info, Pose2D desired_position) {

    // ang = info.in.e.posang;
    // Pose2D info.= (e.posPose2D){(Vec2){0, 0}, 0};
    Vec2 total_force = VEC_ZERO();
    Vec2 goal_vec = VEC_sub(&desired_position.pos, &info->pose.pos);
    if (goal_vec.x > MAX_GOAL_DIST)
        goal_vec.x = MAX_GOAL_DIST;
    if (goal_vec.x < -MAX_GOAL_DIST)
        goal_vec.x = -MAX_GOAL_DIST;
    if (goal_vec.y > MAX_GOAL_DIST)
        goal_vec.y = MAX_GOAL_DIST;
    if (goal_vec.y < -MAX_GOAL_DIST)
        goal_vec.y = -MAX_GOAL_DIST;

    // if we have already reached the desired position in a certain range
    if (VEC_length_sq(&goal_vec) < 10000000) {

        info->reached_waypoint = true;
        return (VelData){0, 0};
    }
    Vec2 atractive_force = VEC_scalar_Q16_mut(&goal_vec, KATRACTIVE);
    total_force = VEC_sum(&total_force, &atractive_force);
    // we pass the positions to the grid index
    int16_t rx = (int16_t)(info->pose.pos.x >> 13) + HALF_SIZE;
    int16_t ry = (int16_t)(info->pose.pos.y >> 13) + HALF_SIZE;

    // now we search around the grid to make our potencial field
    for (int i = -3; i <= 3; i++) {
        for (int j = -3; j <= 3; j++) {
            // we dont check the robot current cell(division by zero)
            if (j == 0 && i == 0) {
                continue;
            }

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
    total_force = (Vec2){.x = Q16_mult(total_force.x, Q16_cos(info->pose.ang)) +
                              Q16_mult(total_force.y, Q16_sin(info->pose.ang)),
                         .y = Q16_mult(total_force.x, -Q16_sin(info->pose.ang)) +
                              Q16_mult(total_force.y, Q16_cos(info->pose.ang))};
    VelData data;

    data.linear_vel = (total_force.x >> 15);
    data.ang_vel = (total_force.y >> 15);
    // linear_vel = data.linear_vel;
    // ang_vel = data.ang_vel;
    return data;
}

Pose2D global_planer(RobotInfo *info) {
    // if (info->is_stucked) {
    // info->is_stucked = false;
    // info->nav_state = NAV_EXPLORING;
    // }
    // variable holding the waypoints of the exploring route
    static uint16_t waypoints[MAX_WAYPOINTS];
    // number of waypoints active
    static uint8_t num_waypoints = 0;
    // current destiny waypoint
    static uint8_t curr_waypoint = 0;

    Pose2D desired_pose = info->pose;

    switch (info->nav_state) {

    // when the robot has the waypoints defined and it has to follow them
    case NAV_WAYPOINT:
        if (info->reached_waypoint) {

            info->reached_waypoint = false;

            curr_waypoint++;
            // if we have reached the final waypoint we explore once again
            if (curr_waypoint >= num_waypoints) {
                info->nav_state = NAV_EXPLORING;
                break;
            }
        }
        // we scaled once again , multiplying by 2^14 ,substracting the offset and adding
        // the 12.5 cm to go to the center of the tile
        desired_pose.pos.x =
            (((q16_t)(uint8_t)(waypoints[curr_waypoint] >> 8) - (GRID_SIZE / 2)) << 14) +
            ((q16_t)1 << 13);
        desired_pose.pos.y =
            (((q16_t)(uint8_t)(waypoints[curr_waypoint]) - (GRID_SIZE / 2)) << 14) +
            ((q16_t)1 << 13);
        break;
    case NAV_EXPLORING: {
        if (info->going_home) {
            stop_robot = true;
            info->going_home = false;
            break;
        }

        curr_waypoint = 0;
        num_waypoints = 0;

        // we convert the position from Q16.16 into the "tile" position
        // since each mini tile is 12.5cm * 12.5 cm and the Bigger tile that are represented in
        // the search algorithms are 25cm * 25 cm we have to divide between  0.25 m in q16 which
        // is 16384 or 2**14
        //
        // the offset of GRID_SIZE / 2 is because the (0,0) of   the robot is the middle of the grid

        int16_t raw_gx = (int16_t)(info->pose.pos.x >> 14) + (GRID_SIZE / 2);
        int16_t raw_gy = (int16_t)(info->pose.pos.y >> 14) + (GRID_SIZE / 2);

        if (raw_gx < 0)
            raw_gx = 0;
        else if (raw_gx >= GRID_SIZE)
            raw_gx = GRID_SIZE - 1;

        if (raw_gy < 0)
            raw_gy = 0;
        else if (raw_gy >= GRID_SIZE)
            raw_gy = GRID_SIZE - 1;

        uint8_t grid_pos_x = (uint8_t)raw_gx;
        uint8_t grid_pos_y = (uint8_t)raw_gy;
        uint16_t dst_pos = BFS_frontier_search(_grid, grid_pos_x, grid_pos_y);

        // if there is no frontier we give another try
        if (dst_pos == 0xFFFF) {

            dst_pos = find_patrol_goal(grid_pos_x, grid_pos_y);
            if (dst_pos == 0xFFFF)
                break;
        }

        uint8_t dst_x = (uint8_t)(dst_pos >> 8);
        uint8_t dst_y = (uint8_t)(dst_pos);
        num_waypoints = JPS_find_path(_grid, grid_pos_x, grid_pos_y, dst_x, dst_y, waypoints);
        // if there is no way to get to that point
        if (num_waypoints == 0) {
            break;
        } else {
            info->reached_waypoint = false;
            info->nav_state = NAV_WAYPOINT;
            // we omit the first waypoint that is the position of the robot
            curr_waypoint = (num_waypoints > 1) ? 1 : 0;
            desired_pose.pos.x =
                (((q16_t)(uint8_t)(waypoints[curr_waypoint] >> 8) - (GRID_SIZE / 2)) << 14) +
                ((q16_t)1 << 13);
            desired_pose.pos.y =
                (((q16_t)(uint8_t)(waypoints[curr_waypoint]) - (GRID_SIZE / 2)) << 14) +
                ((q16_t)1 << 13);
        }

    }

    break;

    case NAV_HOME: {
        int16_t raw_gx = (int16_t)(info->pose.pos.x >> 14) + (GRID_SIZE / 2);
        int16_t raw_gy = (int16_t)(info->pose.pos.y >> 14) + (GRID_SIZE / 2);

        if (raw_gx < 0)
            raw_gx = 0;
        else if (raw_gx >= GRID_SIZE)
            raw_gx = GRID_SIZE - 1;

        if (raw_gy < 0)
            raw_gy = 0;
        else if (raw_gy >= GRID_SIZE)
            raw_gy = GRID_SIZE - 1;

        uint8_t grid_pos_x = (uint8_t)raw_gx;
        uint8_t grid_pos_y = (uint8_t)raw_gy;

        num_waypoints =
            JPS_find_path(_grid, grid_pos_x, grid_pos_y, GRID_SIZE / 2, GRID_SIZE / 2, waypoints);
        if (num_waypoints == 0) {
            break;
        } else {
            info->reached_waypoint = false;
            info->nav_state = NAV_WAYPOINT;
            // we omit the first waypoint that is the position of the robot
            curr_waypoint = (num_waypoints > 1) ? 1 : 0;
            desired_pose.pos.x =
                (((q16_t)(uint8_t)(waypoints[curr_waypoint] >> 8) - (GRID_SIZE / 2)) << 14) +
                ((q16_t)1 << 13);
            desired_pose.pos.y =
                (((q16_t)(uint8_t)(waypoints[curr_waypoint]) - (GRID_SIZE / 2)) << 14) +
                ((q16_t)1 << 13);
        }
        info->going_home = true;

    } break;

    case NAV_GUIDED:
        P2DIR &= ~BIT1;
        P2REN |= BIT1;
        // pullup
        P2OUT |= BIT1;

        if ((P2IN & BIT1) == 0) {
            info->nav_state = NAV_HOME;
        }
        break;
    case NAV_STOP:
        nav_stop_routine(info);
        break;
    }
    return desired_pose;
}
static inline void nav_stop_routine(RobotInfo *info) {

    mt_stop();

    P2DIR &= ~BIT1;
    P2REN |= BIT1;
    // pullup
    P2OUT |= BIT1;

    // green led
    P4DIR |= BIT7;
    P4OUT |= BIT7;

    while (stop_robot) {
        if ((P2IN & BIT1) == 0) {
            __delay_cycles(20000);
            if ((P2IN & BIT1) == 0) {
                send_mapgrid();
                while ((P2IN & BIT1) == 0 && stop_robot) {
                }
                __delay_cycles(20000);
            }
        }
    }
    while (true) {

        if ((P2IN & BIT1) == 0) {
            __delay_cycles(10000);
            send_mapgrid();
        }
        if (stop_robot == false) {
            P4OUT &= ~BIT7;
            info->nav_state = NAV_EXPLORING;
            break;
        }
    }
}

// to make the map less prone to errors we will not update the map if the robot is goiing to fast
#define MAX_ANG_VEL FLOAT_TO_Q16(0.4)
#define FRONT_SENSOR_OFF FLOAT_TO_Q16(0.08)
#define SIDE_SENSOR_OFF FLOAT_TO_Q16(0.05)
void fill_occupancy_grid(const RobotInfo *info, const SensorDistances distances) {

    // if the robot is going to fast we dont calculate anything
    if (info->ang_vel >= MAX_ANG_VEL || info->ang_vel <= -MAX_ANG_VEL) {
        return;
    } // Firstly lets calculate the position in global coordinates of the obstacles, since the
    // distance sensors
    //  are position in the front {1,0} , the left { 0,1 } and the right { 0 , -1} in local
    //  coord of the robot we have to change the reference of the obstacles from local to global

    ang_unit robot_ang = info->pose.ang;
    Vec2 robot_vec = info->pose.pos;
    int16_t x = (int16_t)(robot_vec.x >> 13) + HALF_SIZE;
    int16_t y = (int16_t)(robot_vec.y >> 13) + HALF_SIZE;

    _update_cell(x, y, -15);

    if (distances.front_dist > MIN_DIST && distances.front_dist < MAX_DIST) {
        Vec2 front_obstacle =
            VEC_sum(&robot_vec,
                    &(Vec2){Q16_mult(distances.front_dist + FRONT_SENSOR_OFF, Q16_cos(robot_ang)),
                            Q16_mult(distances.front_dist + FRONT_SENSOR_OFF, Q16_sin(robot_ang))});
        obstacle_update(&robot_vec, &front_obstacle);
    } else if (distances.front_dist >= MAX_DIST) {

        Vec2 front_obstacle = VEC_sum(&robot_vec, &(Vec2){Q16_mult(MAX_DIST, Q16_cos(robot_ang)),
                                                          Q16_mult(MAX_DIST, Q16_sin(robot_ang))});
        // no_obstacle_update(&robot_vec, &front_obstacle);
    }
    if (distances.left_dist < MAX_DIST && distances.left_dist > MIN_DIST) {
        Vec2 left_obstacle =
            VEC_sum(&robot_vec,
                    &(Vec2){Q16_mult(distances.left_dist + SIDE_SENSOR_OFF, -Q16_sin(robot_ang)),
                            Q16_mult(distances.left_dist + SIDE_SENSOR_OFF, Q16_cos(robot_ang))});
        obstacle_update(&robot_vec, &left_obstacle);
    } else if (distances.left_dist >= MAX_DIST) {

        Vec2 left_obstacle = VEC_sum(&robot_vec, &(Vec2){Q16_mult(MAX_DIST, -Q16_sin(robot_ang)),
                                                         Q16_mult(MAX_DIST, Q16_cos(robot_ang))});
        // no_obstacle_update(&robot_vec, &left_obstacle);
    }
    if (distances.right_dist < MAX_DIST && distances.right_dist > MIN_DIST) {
        Vec2 right_obstacle = VEC_sum(
            &robot_vec,
            &(Vec2){Q16_mult(distances.right_dist + SIDE_SENSOR_OFF, Q16_sin(robot_ang)),
                    Q16_mult(-(distances.right_dist + SIDE_SENSOR_OFF), Q16_cos(robot_ang))});
        obstacle_update(&robot_vec, &right_obstacle);
    } else if (distances.right_dist >= MAX_DIST) {
        Vec2 right_obstacle = VEC_sum(&robot_vec, &(Vec2){Q16_mult(MAX_DIST, Q16_sin(robot_ang)),
                                                          Q16_mult(-MAX_DIST, Q16_cos(robot_ang))});
        // no_obstacle_update(&robot_vec, &right_obstacle);
    }
}
static uint16_t find_patrol_goal(uint8_t curr_x, uint8_t curr_y) {
    uint16_t best_pos = 0xFFFF;
    uint16_t max_dist_sq = 16;
    for (uint8_t x = 1; x < GRID_SIZE - 1; x++) {
        for (uint8_t y = 1; y < GRID_SIZE - 1; y++) {
            uint16_t cell = _grid[x][y];
            if ((cell & 0x8888) != 0x8888 && cell != 0x7777) {
                int16_t dx = (int16_t)x - (int16_t)curr_x;
                int16_t dy = (int16_t)y - (int16_t)curr_y;
                uint16_t dist_sq = (uint16_t)(dx * dx + dy * dy);

                if (dist_sq > max_dist_sq) {
                    max_dist_sq = dist_sq;
                    best_pos = ((uint16_t)x << 8) | y;
                }
            }
        }
    }

    return best_pos;
}
