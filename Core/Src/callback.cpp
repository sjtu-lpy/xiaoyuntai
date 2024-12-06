//
// Created by 26279 on 24-11-25.
//

#include <main.h>
#include <can.h>
#include <spi.h>
#include <tim.h>
#include "IMU.h"
#include "Motor.h"
#include "string.h"
#include <math.h>

#include "iwdg.h"

uint32_t TxMAilBox;
extern IMU imu;
uint8_t rxData[8],txData[8];
Motor pitch = Motor(1,1);
Motor yaw = Motor(3,1);
CAN_TxHeaderTypeDef txHeader = {0x1FF,0,0,0,8};
uint32_t TxMailBox;
int t;
uint8_t init_out[8];
int init_angle,init_last_angle;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim6.Instance) {
        HAL_IWDG_Refresh(&hiwdg);
        imu.get_accel();
        imu.get_gyro();
        imu.calculate();
        if (t < 1000) {
            init_out[2 * pitch.id - 1] = 0xFF;
            init_out[2 * pitch.id - 2] = 11;
            HAL_CAN_AddTxMessage(&hcan1,&txHeader,init_out,&TxMAilBox);
            pitch.get_message();
            init_last_angle = init_angle;
            init_angle = pitch.angle;
            if (abs(init_angle - init_last_angle) < 0.1) t++;
            if (t == 1000) pitch.angle_sum = -30;
        }
        //imu version
        // if (t < 1000) {
        //     init_out[2 * pitch.id - 1] = 0xFF;
        //     init_out[2 * pitch.id - 2] = 11;
        //     HAL_CAN_AddTxMessage(&hcan1,&txHeader,init_out,&TxMAilBox);
        //     init_last_angle = init_angle;
        //     init_angle = imu.pitch;
        //     if (abs(init_angle - init_last_angle) < 0.1) t++;
        //     if (t == 1000) pitch.angle_sum = -30;
        // }
        //uint16_t output_pitch = pitch.angle_regulation(imu.pitch) + pitch.feedforward(imu.pitch);
        uint16_t output_pitch = pitch.angle_regulation(pitch.angle) + pitch.feedforward(pitch.angle);
        uint16_t output_yaw = yaw.angle_regulation(yaw.angle);
        uint8_t output[8];
        output[2*pitch.id - 2] = output_pitch >> 8;
        output[2*pitch.id - 1] = output_pitch;
        output[2*yaw.id - 2] = output_yaw >> 8;
        output[2*yaw.id - 1] = output_yaw;
        HAL_CAN_AddTxMessage(&hcan1,&txHeader,output,&TxMAilBox);
    }
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == hspi1.Instance) {
        imu.get_accel();
        imu.get_gyro();
        imu.calculate();
    }
}
//imu version does not need can function
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == hcan1.Instance) {
        pitch.get_message();
        yaw.get_message();
    }
}