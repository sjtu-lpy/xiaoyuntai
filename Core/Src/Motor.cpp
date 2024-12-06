//
// Created by 26279 on 24-12-3.
//
#include<Motor.h>
//#include<can.h>
#include<IMU.h>
extern IMU imu;
float linearMapping(float in_min, float in_max, float out_min, float out_max,float input){
    float k = (out_max - out_min) / (in_max - in_min);
    return out_min + k * (in_max - in_min);
}
Motor::Motor(int id,float reduction_ratio){
    id = id;
    reduction_ratio = reduction_ratio;
    flag = 0;
}
void Motor::get_message(){
    HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&rxHeader,rxData);
    //计算当前输出轴的绝对角度（要考虑减速比）
    int in_angle = (rxData[0] << 8) + rxData[1];
    last_angle = angle;
    angle = linearMapping(0.0,8191.0,0.0,360.0,in_angle);
    if(angle - last_angle > 180) angle_sum += (angle - last_angle - 360.0) / reduction_ratio;
    else if(angle - last_angle < -180) angle_sum += (angle - last_angle + 360.0) / reduction_ratio;
    else angle_sum += angle - last_angle / reduction_ratio;
    //计算输出轴的速度（要考虑减速比）
    int in_speed = (rxData[2] << 8) + rxData[3];
    speed = (float)in_speed / reduction_ratio;
}

uint16_t Motor::speed_regulation(float target_speed){
    float output = speed_pid.calculate(target_speed,speed);
    return (uint16_t)output;
}

uint16_t Motor::angle_regulation(float target_angle){
    float target_speed = angle_pid.calculate(target_angle,angle);
    float output = speed_pid.calculate(target_speed,speed);
    //imu version
    // float target_speed = angle_pid.calculate(target_angle,imu.pitch);
    return (uint16_t)output;
}
uint16_t Motor::feedforward(float target_angle) {
    uint16_t output;
    float k = 3.1415926/180;
    int16_t u_max = 1800;
    int16_t max_angle = -40;
    int16_t feedforward = u_max*(1-(target_angle-max_angle)*k*(target_angle-max_angle)*k/2);
    output = feedforward;
    return output;
}
