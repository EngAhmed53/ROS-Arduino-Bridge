#ifndef ESP_HOVER_MODEL_H
#define ESP_HOVER_MODEL_H

#include <Arduino.h>

struct PIDModel {
    double proportional{0};
    double integral{0};
    double derivative{0};

    double total() const {
        return proportional + integral - derivative;
    }
};
#endif