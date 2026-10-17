
// HARDWARE

// constant to adapt the program
#ifndef CONFIG_H
#define CONFIG_H
#define BT_RX_BUF_SIZE 16 // maximum length of a command(including tthe ending character)
#define END_CHAR '#'
#define DATA_COUNT 32          // size of the maximum data in bytes
#define WHEEL_DIAMETER_MM 60UL // diameter of the motor wheels
#define ENCODER_GAPS                                                                               \
    20UL                        // number of gaps the encoder has (we use the x2
                                // precision)
#define WHEEL_WHEEL_DIST_MM 135 // distance between the two wheels
// defines of the task frequencies in miliseconds , this manages how often you
// need the task to execute, is not in HZ but in MILISECONDS

//=================
// TIMERS
// =============

// this shouldnt be below 50ms so the sensor has time to receive
#define FRONT_DS_MS 70U
// this shouldnt be below 50ms so the sensor has time to receive the wave
#define LEFT_RIGHT_DS_MS 200U

// this timer updates the belief of the robot and also sends the information via bluetooth
// this shouldnt be to late in case the encoders
// overflows, depends of the velocity and how precise
// is the encoder
#define POSITION_UPDATE_MS 300U
// this is the timer for the control of the motors
//  this timer should always be less than the position update
#define CONTROL_UPDATE_MS 50U

#endif // CONFIG_H
