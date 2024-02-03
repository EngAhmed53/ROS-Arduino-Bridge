//
// Created by ahmed on 4/20/2023.
//

#ifndef ESP_HOVER_PID_H
#define ESP_HOVER_PID_H

#include <Arduino.h>
#include "model.h"

class PID {

public:
    PID(int highBoundary, int lowBoundary, int maxNeglectedError = 0,
                 int minNeglectedError = 0);

    void SetPoint(int setPoint);

    double SetPoint() const;

    void Gains(double kp, double ki, double kd);

    PIDModel Compute(double sampleTime, int measurement);

private:
    int _highBoundary{0}, _lowBoundary{0}, _maxNeglectedError{0}, _minNeglectedError{0};
    int _setPoint{0};
    double _kp{0}, _ki{0}, _kd{0};
    double _accumError{0};
    int _previous_m{0};
};

#endif //ESP_HOVER_PID_H