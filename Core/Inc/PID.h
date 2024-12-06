//
// Created by 26279 on 24-12-3.
//
#ifndef PID_H
#define PID_H
class PID{
private:
    float Kp_, Ki_, Kd_;
    float i_max_,out_max_;
    //i_max is the limit of error_sum
    float output_;

    //float ref_,dfb_;
    //reference & feedback
    float error_, error_sum, last_error;
    float pout_, iout_, dout_;
public:
    PID(float Kp, float Ki, float Kd, float i_max, float out_max);
    float calculate(float ref_, float dfb_);
};
#endif //PID_H
