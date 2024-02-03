#include "encoder_driver.h"

volatile long right_enc_pos{0};
volatile long left_enc_pos{0};

int enc_count_diff[2]{-1, 1};

void left_enc_ISR() {
    left_enc_pos += (enc_count_diff[PIND >> LEFT_ENC_PIN_B & 1]) * -1; // multiply by -1 to reverse the direction
}

void right_enc_ISR() {
    right_enc_pos += enc_count_diff[PIND >> RIGHT_ENC_PIN_B & 1];
}

long readEncoder(DiffSide side) {
    long value = side == DiffSide::LEFT ? left_enc_pos : right_enc_pos;
    return value;
}

void resetEncoders() {
    left_enc_pos = 0;
    right_enc_pos = 0;
}