//
// Created by ahmed on 4/20/2023.
//

#include "PID.h"

PID::PID(int highBoundary, int lowBoundary, int maxNeglectedError, int minNeglectedError)
        : _highBoundary{highBoundary},
          _lowBoundary{lowBoundary},
          _maxNeglectedError{maxNeglectedError},
          _minNeglectedError{minNeglectedError} {}

// Will be updated from BLE core
void PID::Gains(double kp, double ki, double kd) {
    _accumError = 0; // Reset the accumulated error
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

// Will be updated from BLE core
void PID::SetPoint(const int setPoint) {
    _setPoint = setPoint;
}

double PID::SetPoint() const {
    return _setPoint;
}

PIDModel PID::Compute(double sampleTime, int measurement) {
    PIDModel model;

    // Serial.print("Sample time = ");
    // Serial.println(sampleTime);

    // Serial.print("process var = ");
    // Serial.println(measurement);

    int error = _setPoint - measurement;

    // Serial.print("error = ");
    // Serial.println(error);

    // // Proportional
    model.proportional = _kp * error;

    // Serial.print("porp = ");
    // Serial.println(model.proportional);

    // Integral
    if (error >= _maxNeglectedError || error <= _minNeglectedError) {
        double error_area = error * sampleTime;
        _accumError += error_area;

        double integral = _ki * _accumError;

        if (integral > _highBoundary) {
            integral = _highBoundary;
            _accumError -= error_area;
        } else if (integral < _lowBoundary) {
            integral = _lowBoundary;
            _accumError -= error_area;
        }

        model.integral = integral;
    } else {
        model.integral = _ki * _accumError;
    }

//    Serial.print("int = ");
//    Serial.println(model.integral);

    // Derivative
    double measurement_rate = (measurement - _previous_m) / sampleTime;
    model.derivative = _kd * measurement_rate;

    // Total
    double total = model.proportional + model.integral - model.derivative;
    model.system_constrained_total = constrain(total, _lowBoundary, _highBoundary);

    // Next loop setup
    _previous_m = measurement;

    return model;
}