#include "PID.h"

PID::PID(float kp,
         float ki,
         float kd,
         float out_limit,
         float integral_limit)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;

    _out_limit = out_limit;
    _integral_limit = integral_limit;

    _integral = 0.0f;
    _prev_error = 0.0f;
}

float PID::update(float target, float current)
{
    float error = target - current;

    // 積分
    _integral += error;

    if(_integral > _integral_limit)
        _integral = _integral_limit;

    if(_integral < -_integral_limit)
        _integral = -_integral_limit;

    // 微分
    float derivative = error - _prev_error;

    _prev_error = error;

    // PID
    float output =
          _kp * error
        + _ki * _integral
        + _kd * derivative;

    // 出力制限
    if(output > _out_limit)
        output = _out_limit;

    if(output < -_out_limit)
        output = -_out_limit;

    return output;
}

void PID::reset()
{
    _integral = 0.0f;
    _prev_error = 0.0f;
}

void PID::setGain(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}
