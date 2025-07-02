#ifndef PID_H
#define PID_H

class PID
{
private:
    float target, error, last_error = 0;
    float integral = 0.0;
    float kp, ki, kd;
    float pfix, ifix, dfix;

    float last_output = 0.0;
    const float SMOOTHING_ALPHA = 0.2;

public:
    PID(float target, float kp, float ki, float kd)
    {
        this->target = target;
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }

    float get_result(float measurement, float currentServoPositio)
    {
        error = measurement + currentServoPositio - target;
        integral += error;
        pfix = error * kp;
        ifix = integral * ki;
        dfix = (error - last_error) * kd;
        last_error = error;

        float output = pfix + ifix + dfix;

        float smooth_output = (1.0 - SMOOTHING_ALPHA) * last_output + SMOOTHING_ALPHA * output;
        last_output = smooth_output;

        return smooth_output;
    }
};

#endif
