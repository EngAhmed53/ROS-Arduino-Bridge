//
// Created by ahmed on 2/2/2024.
//
#include "motor_driver.h"

void set_motors_speed(int right_spd, int left_spd) {
    if (right_spd > 0) {
        PORTD |= _BV(RIGHT_MOTOR_FORWARD); // set high
        PORTD &= ~_BV(RIGHT_MOTOR_BACKWARD); // set low
    } else {
        right_spd *= -1;
        PORTD &= ~_BV(RIGHT_MOTOR_FORWARD); // set low
        PORTD |= _BV(RIGHT_MOTOR_BACKWARD); // set high
    }

    analogWrite(RIGHT_MOTOR_ENABLE, right_spd);


    if (left_spd > 0) {
        PORTB |= _BV(LEFT_MOTOR_FORWARD); // high
        PORTB &= ~_BV(LEFT_MOTOR_BACKWARD); // low
    } else {
        left_spd *= -1;
        PORTB &= ~_BV(LEFT_MOTOR_FORWARD); // low
        PORTB |= _BV(LEFT_MOTOR_BACKWARD); // high
    }

    analogWrite(LEFT_MOTOR_ENABLE, left_spd);
}


void stop_motors() {
    analogWrite(RIGHT_MOTOR_ENABLE, 0);
    analogWrite(LEFT_MOTOR_ENABLE, 0);
}