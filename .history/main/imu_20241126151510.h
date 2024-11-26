#pragma once

#include <stdio.h>
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "string.h"
#include "esp_timer.h"
//#include <cmath>
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_SCL_IO 15
#define I2C_MASTER_SDA_IO 7

#define I2C_SENSOR_NUM I2C_NUM_1
#define I2C_sensor_SDA_IO 5
#define I2C_sensor_SCLK_IO 4

//mpu config
#define MPU_ADDR 0x68             // MPU6050 设备地址
#define MPU_CMD_WHO_AM_I 0x75     // MPU6050 设备确认寄存器
#define MPU_CMD_PWR_MGMT_1 0x6B   // 电源管理寄存器
#define MPU_CMD_SMPLRT_DIV 0x19   // 陀螺仪采样率分频器寄存器
#define MPU_CMD_CONFIG 0x1A       // 数字低通滤波器配置寄存器
#define MPU_CMD_GYRO_CONFIG 0x1B  // 陀螺仪配置寄存器
#define MPU_CMD_ACCEL_CONFIG 0x1C // 加速度传感器配置寄存器
#define MPU_CMD_ACCEL_XOUT_H 0x3B // 加速计X轴高字节数据寄存器
#define MPU_CMD_ACCEL_YOUT_H 0x3D // 加速计Y轴高字节数据寄存器
#define MPU_CMD_ACCEL_ZOUT_H 0x3F // 加速计Z轴高字节数据寄存器
#define MPU_CMD_ANGLE_XOUT_H 0x43 // 陀螺仪X轴高字节数据寄存器
#define MPU_CMD_ANGLE_YOUT_H 0x45 // 陀螺仪Y轴高字节数据寄存器
#define MPU_CMD_ANGLE_ZOUT_H 0x47 // 陀螺仪Z轴高字节数据寄存器
#define MPU_CMD_TEM 0x41 // 陀螺仪Z轴高字节数据寄存器
#define FILTER_ORDER 5
#define USE_FILTER true
//imu队列
QueueHandle_t imu_data_queue;
#define imu_queue_size 32
//imu数据
typedef struct{
  float coeffs[FILTER_ORDER];
  float buffer[FILTER_ORDER];
  int index;
}FIRFilter;
typedef struct {
    int16_t x_acc, y_acc, z_acc;
    int16_t x_anl, y_anl, z_anl;
    FIRFilter x_filter, y_filter, z_filter;
    float displacement;  // Add this line
} imu_data;


static const char *TAG = "MPU6050";
void I2C_init();
void MPU6050_init();
int16_t get_accel_x();
int16_t get_accel_y();
int16_t get_accel_z();
int16_t get_angle_x();
int16_t get_angle_y();
int16_t get_angle_z();
imu_data imu_send_wifi();
void app_imu_get_data();
void app_data_receive();
void init_fir_filter(FIRFilter* filter);
float apply_fir_filter(FIRFilter* filter, float input);

// 状态向量结构体
typedef struct {
    float position;
    float velocity;
    float acceleration;
} State;

// 卡尔曼滤波器结构体
typedef struct {
    State state;
    float P[3][3];  // 协方差矩阵
    float Q[3][3];  // 过程噪声协方差
    float R;        // 测量噪声协方差
} KalmanFilter;

// 函数声明
void init_kalman_filter(KalmanFilter* kf);
void kalman_predict(KalmanFilter* kf, float dt);
void kalman_update(KalmanFilter* kf, float measurement);
bool detect_zero_velocity(float acceleration, float threshold);
float estimate_displacement(float acceleration);