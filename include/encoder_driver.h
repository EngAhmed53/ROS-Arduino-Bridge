#ifndef ENCODER_H
#define ENCODER_H

#include "model.h"
#include <Arduino.h>

#define RIGHT_ENC_PIN_A PD2
#define RIGHT_ENC_PIN_B PD4
#define LEFT_ENC_PIN_A PD3
#define LEFT_ENC_PIN_B PD5

void left_enc_ISR();
void right_enc_ISR();
long readEncoder(DiffSide side);
void resetEncoders();

#endif