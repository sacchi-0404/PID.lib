#ifndef PID_H
#define PID_H

class PID
{
public:
    PID(float kp = 0.8f,
        float ki = 0.02f,
        float kd = 0.0f,
        float out_limit = 2048.0f,
        float integral_limit = 5000.0f);

    float update(float target, float current);

    void reset();

    void setGain(float kp, float ki, float kd);

private:
    float _kp;
    float _ki;
    float _kd;

    float _integral;
    float _prev_error;

    float _out_limit;
    float _integral_limit;
};

#endif
