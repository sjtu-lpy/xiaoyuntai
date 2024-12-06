//
// Created by 26279 on 24-12-3.
//
#include <IMU.h>
#include "gpio.h"
#include<spi.h>
#include<cmath>
IMU::IMU() {
    acc_range = 2;
    gyro_range = 0;
}
void IMU::BMI088_ACCEL_NS_L(void) {
    HAL_GPIO_WritePin(Accel_GPIO_Port, Accel_Pin, GPIO_PIN_RESET);
}
void IMU::BMI088_ACCEL_NS_H(void) {
    HAL_GPIO_WritePin(Accel_GPIO_Port, Accel_Pin, GPIO_PIN_SET);
}
void IMU::BMI088_GYRO_NS_L(void) {
    HAL_GPIO_WritePin(Gyro_GPIO_Port, Gyro_Pin, GPIO_PIN_RESET);
}
void IMU::BMI088_GYRO_NS_H(void) {
    HAL_GPIO_WritePin(Gyro_GPIO_Port, Gyro_Pin, GPIO_PIN_SET);
}

void IMU::BMI088_ReadReg_ACCEL() {
    uint8_t tx_data = 0x80 | 0x12;//第一位为1时表示读入模式，后面七位reg表示要改写的寄存器地址
    BMI088_ACCEL_NS_L();//先拉低电平，详见spi通信原理
    HAL_SPI_Transmit(&hspi1,&tx_data, 1, 1000);//先告诉寄存器我要读
    HAL_SPI_Receive(&hspi1, rx_acc, 7, 1000);//从reg往后7个寄存器挨个通信获取信息，且不再读地址信息，直接读八位数据
    //这里加速度计有小bug，会多发一个无效的8位数据，所以要读7个寄存器
    BMI088_ACCEL_NS_H();
}
void IMU::BMI088_ReadReg_GYRO() {
    uint8_t tx_data = 0x80 | 0x02;
    BMI088_GYRO_NS_L();
    HAL_SPI_Transmit(&hspi1,&tx_data, 1, 1000);
    HAL_SPI_Receive(&hspi1, rx_gyro, 6, 1000);
    BMI088_GYRO_NS_H();
}
void IMU::BMI088_WriteReg(uint8_t reg, uint8_t write_data) {
    uint8_t tx_data[2] = {reg, write_data};
    HAL_Delay(2);
    HAL_SPI_Transmit(&hspi1, tx_data, 2, 1000);

}

void IMU::get_accel(){
    BMI088_ReadReg_ACCEL();
    int16_t accX, accY, accZ;
    float k;
    accX = (rx_acc[2] << 8) | rx_acc[1];
    accY = (rx_acc[4] << 8) | rx_acc[3];
    accZ = (rx_acc[6] << 8) | rx_acc[5];//详见BMI说明书第19页
    if(acc_range == 0) k = 9.8/32767.0*3.0;
    else if(acc_range == 1) k = 9.8/32767.0*6.0;
    else if(acc_range == 2) k = 9.8/32767.0*12.0;
    else if(acc_range == 3) k = 9.8/32767.0*24.0;//见BMI说明书第23页的比例转换
    acc[0] = accX*k;
    acc[1] = accY*k;
    acc[2] = accZ*k;
}

void IMU::get_gyro(){
  BMI088_ReadReg_GYRO();
  int16_t gyroX, gyroY, gyroZ;
  float k;
  gyroX = (rx_gyro[1] << 8) | rx_gyro[0];
  gyroY = (rx_gyro[3] << 8) | rx_gyro[2];
  gyroZ = (rx_gyro[5] << 8) | rx_gyro[4];//BMI第26页
  if(gyro_range == 0) k = 0.061;//这里注意单位，有个m，*0.001
  else if(gyro_range == 1) k = 0.0305;
  else if(gyro_range == 2) k = 0.0153;
  else if(gyro_range == 3) k = 0.0076;
  else if(gyro_range == 4) k = 0.0038;//BMI第28页5.5.4
  gyro[0] = gyroX*k;
  gyro[1] = gyroY*k;
  gyro[2] = gyroZ*k;
}

void IMU::BMI088_Init(void) {
    // Soft Reset ACCEL
    BMI088_ACCEL_NS_L();
    BMI088_WriteReg(0x7E, 0xB6); // Write 0xB6 to ACC_SOFTRESET(0x7E)
    HAL_Delay(1);
    BMI088_ACCEL_NS_H();

    // Soft Reset GYRO
    BMI088_GYRO_NS_L();
    BMI088_WriteReg(0x14, 0xB6); // Write 0xB6 to GYRO_SOFTRESET(0x14)
    HAL_Delay(30);
    BMI088_GYRO_NS_H();

    // Switch ACCEL to Normal Mode
    BMI088_ACCEL_NS_L();
    HAL_Delay(1);
    BMI088_WriteReg(0x7D, 0x04); // Write 0x04 to ACC_PWR_CTRL(0x7D)
    HAL_Delay(1);
    BMI088_ACCEL_NS_H();
}
void IMU::calculate(){
    float roll_acc,pitch_acc;
    float roll_gyro,pitch_gyro,yaw_gyro;
    float roll_g,pitch_g,yaw_g;
    float delta_t = 0.001;
    float deg_rad = 3.1415926/180.0;//角度转化成弧度
    roll_acc = atan(acc[1] / acc[2]) / deg_rad;
    pitch_acc = -atan(acc[0] / sqrt(acc[1] * acc[1] + acc[2] * acc[2])) / deg_rad;
    roll_gyro = gyro[0] + sin(pitch)*sin(roll)/cos(pitch)*gyro[1] + sin(pitch)*cos(roll)/cos(pitch)*gyro[2];
    pitch_gyro = cos(roll)*gyro[1] - sin(roll)*gyro[2];
    yaw_gyro = sin(roll)/cos(pitch)*gyro[1] + cos(roll)/cos(pitch)*gyro[2];
    roll_g = roll + roll_gyro * delta_t;
    pitch_g = pitch + pitch_gyro * delta_t;//减小稳态误差
    yaw_g = yaw + yaw_gyro * delta_t;

    roll = K * roll_acc + roll_g * (1 - K);
    pitch = K * pitch_acc + pitch_g * (1 - K);
    yaw = yaw_g;
}

IMU imu;