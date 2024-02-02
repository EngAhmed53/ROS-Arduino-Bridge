#include <Arduino.h>
#include <stdlib.h>
#include "PID.h"
#include "model.h"

#define RIGHT_M_CW_PIN 6
#define RIGHT_M_CCW_PIN 7
#define RIGHT_M_EN_PIN 10

#define LEFT_M_CW_PIN 8
#define LEFT_M_CCW_PIN 9
#define LEFT_M_EN_PIN 11

#define ENC_IN_RIGHT_A 2
#define ENC_IN_RIGHT_B 4

#define MOTOR_MAX_SIGNAL 255
#define MOTOR_MIN_SIGNAL -255

PID right_motor_pid(MOTOR_MAX_SIGNAL, MOTOR_MIN_SIGNAL, 0, 0);
PID left_motor_pid(MOTOR_MAX_SIGNAL, MOTOR_MIN_SIGNAL, 0, 0);
bool pid_mode{false};
int right_motor_setpoint{0}; // encoder counts per PID loop
int left_motor_setpoint{0};  // encoder counts per PID loop
unsigned int loop_rate_millis = 30;

volatile long right_motor_enc_count{0};
volatile long left_motor_enc_count{0};
volatile long previous_right_motor_enc_count{0};
volatile long previous_left_motor_enc_count{0};
unsigned long lastTime{0};

void right_motor_ISR()
{
  if (digitalRead(ENC_IN_RIGHT_B) > 0)
  {
    right_motor_enc_count++;
  }
  else
  {
    right_motor_enc_count--;
  }
}

void setup()
{
  Serial.begin(9600);

  right_motor_pid.Gains(4, 8, .05);
  left_motor_pid.Gains(4, 8, .05);

  pinMode(RIGHT_M_CW_PIN, OUTPUT);
  pinMode(RIGHT_M_CCW_PIN, OUTPUT);
  pinMode(RIGHT_M_EN_PIN, OUTPUT);

  pinMode(LEFT_M_CW_PIN, OUTPUT);
  pinMode(LEFT_M_CCW_PIN, OUTPUT);
  pinMode(LEFT_M_EN_PIN, OUTPUT);

  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_motor_ISR, RISING);
}

void setRightMotorSignal(int signal)
{

  if (signal > 0)
  {
    digitalWrite(RIGHT_M_CW_PIN, HIGH);
    digitalWrite(RIGHT_M_CCW_PIN, LOW);
  }
  else
  {
    digitalWrite(RIGHT_M_CW_PIN, LOW);
    digitalWrite(RIGHT_M_CCW_PIN, HIGH);
  }

  int corrected_signal = min(abs(signal), MOTOR_MAX_SIGNAL);
  analogWrite(RIGHT_M_EN_PIN, corrected_signal);
}

void setLeftMotorSignal(int signal)
{
  if (signal > 0)
  {
    digitalWrite(LEFT_M_CW_PIN, HIGH);
    digitalWrite(LEFT_M_CCW_PIN, LOW);
  }
  else
  {
    digitalWrite(LEFT_M_CW_PIN, LOW);
    digitalWrite(LEFT_M_CCW_PIN, HIGH);
  }

  int corrected_signal = min(abs(signal), MOTOR_MAX_SIGNAL);
  analogWrite(RIGHT_M_EN_PIN, corrected_signal);
}

void handleUserCmd(char cmd, int p1, int p2)
{
  if (cmd == 'c')
  {
    pid_mode = true;
    right_motor_setpoint = p1;
    left_motor_setpoint = p2;

    // Serial.print("PID mode enabled, setpoints:: ");
    // Serial.print(right_motor_setpoint);
    // Serial.print(" ");
    // Serial.println(left_motor_setpoint);
  }
  else if (cmd == 's')
  {
    pid_mode = false;
    right_motor_setpoint = 0;
    left_motor_setpoint = 0;
    previous_left_motor_enc_count = 0;
    previous_right_motor_enc_count = 0;

    setRightMotorSignal(0);
    setLeftMotorSignal(0);

    Serial.print("PID mode disabled");
  }
}

void loop()
{
  char cmd = 0;
  int parm1 = 0, parm2 = 0;
  if (Serial.available() > 0)
  {
    parm1 = Serial.read() - 48;
    cmd = 'c';
  }

  if (cmd != 0)
    handleUserCmd(cmd, parm1, parm2);

  unsigned long now = millis();

  if (pid_mode)
  {
    unsigned long timeChange = (now - lastTime);

    if (timeChange >= loop_rate_millis)
    {
      right_motor_pid.SetPoint(right_motor_setpoint);
      left_motor_pid.SetPoint(left_motor_setpoint);

      int right_enc_counts = right_motor_enc_count;
      int left_enc_counts = left_motor_enc_count;

      // Serial.print("right_enc = ");
      // Serial.println(right_wheel_enc_counts);

      // Serial.print("previous_right_enc = ");
      // Serial.println(previous_right_wheel_enc_count);

      int right_motor_counts_per_loop = right_enc_counts - previous_right_motor_enc_count;
      previous_right_motor_enc_count = right_enc_counts;

      // Serial.print("per loop = ");
      // Serial.println(right_motor_counts_per_loop);

      int left_motor_counts_per_loop = left_enc_counts - previous_left_motor_enc_count;
      previous_left_motor_enc_count = left_enc_counts;

      PIDModel right_motor_pid_model = right_motor_pid.Compute(timeChange / 1000.0, right_motor_counts_per_loop);
      // PIDModel left_motor_pid_model = left_motor_pid.Compute(loop_rate_millis / 1000.0, left_motor_counts_per_loop);

      Serial.print(right_motor_setpoint);
      Serial.print(",");
      Serial.print(right_motor_pid_model.total());
      Serial.print(",");
      Serial.print(right_motor_counts_per_loop);
      Serial.print(",");
      Serial.print(right_motor_pid_model.proportional);
      Serial.print(",");
      Serial.print(right_motor_pid_model.integral);
      Serial.print(",");
      Serial.println(right_motor_pid_model.derivative);

      setRightMotorSignal(static_cast<int>(right_motor_pid_model.total()));
      // setLeftMotorSignal(static_cast<int>(left_motor_pid_model.total()));

      lastTime = now;
    }
  }
  else
  {
    lastTime = now;
  }
}
