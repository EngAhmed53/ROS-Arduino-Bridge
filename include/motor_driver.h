//
// Created by ahmed on 2/2/2024.
//

#ifndef ROS_ARDUINO_BRIDGE_MOTOR_DRIVER_H
#define ROS_ARDUINO_BRIDGE_MOTOR_DRIVER_H

#include <Arduino.h>

// RIGHT_MOTOR
#define RIGHT_MOTOR_FORWARD PD6
#define RIGHT_MOTOR_BACKWARD PD7
#define RIGHT_MOTOR_ENABLE 10

// LEFT_MOTOR
#define LEFT_MOTOR_FORWARD PB0
#define LEFT_MOTOR_BACKWARD PB1
#define LEFT_MOTOR_ENABLE 11

#define MOTOR_MAX_SIGNAL 255
#define MOTOR_MIN_SIGNAL (-255)

void set_motors_speed(int right_spd, int left_spd);
void stop_motors();

#endif //ROS_ARDUINO_BRIDGE_MOTOR_DRIVER_H
