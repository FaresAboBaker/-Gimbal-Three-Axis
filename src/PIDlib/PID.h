#ifndef PID_H
#define PID_H
class PID
{
private:
    float target, error, last_error = 0;
    float integral = 0.0;
    float kp, ki, kd;
    float pfix, ifix, dfix;

public:
    PID(float target, float kp, float ki, float kd)
    {
        this->target = target;
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }
    float get_result(float measurement)
    {
        error = measurement - target;
        integral += error;
        pfix = error * kp;
        ifix = integral * ki;
        dfix = (error - last_error) * kd;
        last_error = error;
        return pfix + ifix + dfix;
    }
};

#endif