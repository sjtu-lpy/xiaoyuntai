//
// Created by 26279 on 24-12-3.
//

#ifndef IMU_H
#define IMU_H
#include<cstdint>
class IMU{
public:
    IMU();
    uint8_t rx_acc[7];
    uint8_t rx_gyro[6];
    int acc[3];
    int gyro[3];
    int acc_range,gyro_range;
    float pitch,roll,yaw,K = 0.003;//结算出的三轴角度及加速度权重K
    void BMI088_ReadReg_ACCEL();
    void BMI088_ReadReg_GYRO();
    void BMI088_WriteReg(uint8_t reg, uint8_t write_data);
    void BMI088_Init(void);
    void BMI088_ACCEL_NS_L(void);
    void BMI088_GYRO_NS_L(void);
    void BMI088_ACCEL_NS_H(void);
    void BMI088_GYRO_NS_H(void);
    void get_accel();
    void get_gyro();
    void calculate();

};
#endif //IMU_H
