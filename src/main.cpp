#include <Arduino.h>
#include "encoder_driver.h"
#include "motor_driver.h"
#include "PID.h"
#include "model.h"

#define BAUDRATE     57600

PID right_pid(MOTOR_MAX_SIGNAL, MOTOR_MIN_SIGNAL, 0, 0);
PID left_pid(MOTOR_MAX_SIGNAL, MOTOR_MIN_SIGNAL, 0, 0);
bool pid_mode{false};
int right_motor_set_point{0}; // encoder counts per PID loop
int left_motor_set_point{0};  // encoder counts per PID loop
unsigned int loop_rate_millis = 30;
unsigned long lastTime{0};

long previous_right_enc_pos{0};
long previous_left_enc_pos{0};

void setup() {
    Serial.begin(BAUDRATE);

    right_pid.Gains(4, 8, .05);
    left_pid.Gains(4, 8, .05);

    DDRD &= ~_BV(RIGHT_ENC_PIN_A);
    DDRD &= ~_BV(RIGHT_ENC_PIN_B);
    DDRD &= ~_BV(LEFT_ENC_PIN_A);
    DDRD &= ~_BV(LEFT_ENC_PIN_B);

    PORTD |= _BV(RIGHT_ENC_PIN_A);
    PORTD |= _BV(RIGHT_ENC_PIN_B);
    PORTD |= _BV(LEFT_ENC_PIN_A);
    PORTD |= _BV(LEFT_ENC_PIN_B);

    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), right_enc_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), left_enc_ISR, RISING);

    DDRD |= _BV(RIGHT_MOTOR_FORWARD);
    DDRD |= _BV(RIGHT_MOTOR_BACKWARD);
    DDRB |= _BV(LEFT_MOTOR_FORWARD);
    DDRB |= _BV(LEFT_MOTOR_BACKWARD);

    pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
    pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
}

void handleUserCmd(char cmd, int p1, int p2) {
    if (cmd == 's') {
        pid_mode = true;
        right_motor_set_point = p1;
        left_motor_set_point = p2;
    } else if (cmd == 'c') {
        pid_mode = false;
        right_motor_set_point = 0;
        left_motor_set_point = 0;
        stop_motors();
        delay(250); // delay to make sure all motors are stooped
        resetEncoders();
        Serial.print("PID mode disabled");
    } else if (cmd == 'p') {
        Serial.print(readEncoder(DiffSide::RIGHT));
        Serial.print(" ");
        Serial.println(readEncoder(DiffSide::LEFT));
    } else if (cmd == 'r') {
        resetEncoders();
        Serial.println("OK");
    }
}

void loop() {
    char cmd = 0;
    int param_1 = 0, param_2 = 0;
    if (Serial.available() > 0) {
        String receivedString = Serial.readStringUntil('\n');
        sscanf(receivedString.c_str(), "%c %d %d", &cmd, &param_1, &param_2);
    }

    if (cmd != 0) handleUserCmd(cmd, param_1, param_2);

    unsigned long now = millis();

    if (pid_mode) {
        unsigned long timeChange = (now - lastTime);

        if (timeChange >= loop_rate_millis) {
            right_pid.SetPoint(right_motor_set_point);
            left_pid.SetPoint(left_motor_set_point);

            long right_enc_pos = readEncoder(DiffSide::RIGHT);
            long left_enc_pos = readEncoder(DiffSide::LEFT);

            // Serial.print("right_enc = ");
            // Serial.println(right_wheel_enc_counts);

            // Serial.print("previous_right_enc = ");
            // Serial.println(previous_right_wheel_enc_count);

            int right_enc_counts_per_loop = static_cast<int>(right_enc_pos - previous_right_enc_pos);
            previous_right_enc_pos = right_enc_pos;

            // Serial.print("per loop = ");
            // Serial.println(right_enc_counts_per_loop);

            int left_enc_counts_per_loop = static_cast<int>(left_enc_pos - previous_left_enc_pos);
            previous_left_enc_pos = left_enc_pos;

            double sample_time_sec = static_cast<double>(timeChange) / 1000.0;

            PIDModel right_motor_pid_model = right_pid.Compute(sample_time_sec, right_enc_counts_per_loop);
            PIDModel left_motor_pid_model = left_pid.Compute(sample_time_sec, left_enc_counts_per_loop);

            Serial.print(left_motor_set_point);
            Serial.print(",");
            Serial.print(left_motor_pid_model.system_constrained_total);
            Serial.print(",");
            Serial.print(left_enc_counts_per_loop);
            Serial.print(",");
            Serial.print(left_motor_pid_model.proportional);
            Serial.print(",");
            Serial.print(left_motor_pid_model.integral);
            Serial.print(",");
            Serial.println(left_motor_pid_model.derivative);

            set_motors_speed(static_cast<int>(right_motor_pid_model.system_constrained_total), static_cast<int>(left_motor_pid_model.system_constrained_total));

            lastTime = now;
        }
    } else {
        lastTime = now;
    }
}
