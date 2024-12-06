//
// Created by 26279 on 24-12-3.
//
#include <PID.h>
PID::PID(float Kp, float Ki, float Kd, float i_max, float out_max) {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
    i_max_ = i_max;
    out_max_ = out_max;
}

float PID::calculate(float ref_, float fdb_) {
    last_error = error_;
    error_ = ref_ - fdb_;
    error_sum += error_;
    if(error_sum >= i_max_) error_sum = i_max_;
    if(error_sum < (-i_max_)) error_sum = -i_max_;
    iout_ = error_sum * Ki_;
    pout_ = Kp_ * error_;
    dout_ = Kd_ * (error_ - last_error);
    output_ = pout_ + iout_ + dout_;
    if(output_ > out_max_) output_ = out_max_;
    if(output_ < (-1 * out_max_)) output_ = (-1 * out_max_);
    return output_;
}