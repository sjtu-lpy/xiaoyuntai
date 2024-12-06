//
// Created by 26279 on 24-12-3.
//
#include <can.h>
#include <PID.h>
#include<cstdint>
#ifndef MOTOR_H
#define MOTOR_H

class Motor{
public:
    bool flag;
    int id;
    float reduction_ratio;
    CAN_RxHeaderTypeDef rxHeader={(uint32_t)0x204 + id,0,0,0,8};
    uint8_t rxData[8];
    uint8_t txData[8];
    float angle,last_angle,angle_sum;//当前反馈角度值，上次角度值，累加角度值（即当前绝对角度值）
    float speed;
    Motor(int id, float reduction_ratio);
    void get_message();
    void feedforward();//前馈
    float kp1,ki1,kd1,i_max1,out_max1;
    float kp2,ki2,kd2,i_max2,out_max2;
    PID speed_pid = PID(kp1,ki1,kd1,i_max1,out_max1);
    PID angle_pid = PID(kp2,ki2,kd2,i_max2,out_max2);
    uint16_t speed_regulation(float target_speed);
    uint16_t angle_regulation(float target_angle);
    uint16_t feedforward(float target_angle);
};

#endif //MOTOR_H
