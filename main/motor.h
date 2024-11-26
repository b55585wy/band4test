#pragma once
#include <stdio.h>
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "string.h"
#include "esp_timer.h"
#include "math.h"
#include "driver/mcpwm_timer.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "pump.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"


//define

//I2C def
#define I2C_MASTER_NUM I2C_NUM_1

//sensor def
//define for mt6701
#define MT6701_ADDR 0X06
#define angle_reg_1 0x03
#define angle_reg_2 0x04//MT6701的角度寄存器 0x03[13:6] 0x04[5:0]

//define for as5600
#define AS5600_ADDR 0X36
#define AS5600_ANG_REG 0X0E

//motor def
//mcpwm def 
#define mcm_unit MCPWM_UNIT_0
#define mcm_timer MCPWM_TIMER_0
#define mcm_a_gen MCPWM_GEN_A   
#define mcm_b_gen MCPWM_GEN_B

//motor gpio def 
#define motor_en_gpio GPIO_NUM_40
#define motor_A_gpio GPIO_NUM_36
#define motor_B_gpio GPIO_NUM_38

//motor status
#define motor_forward 1
#define motor_backward 0
#define motor_stop -1


#define Inhale_state 1//吸气
#define Exhale_state 0//呼气
#define motor_max_voltage 80.0
#define pump_max_voltage 50.0




//struct

typedef struct {
    int16_t raw_data;
    int16_t raw_data_prev;
    float angle;
    float angle_prev;
    float distance;

    bool init;
} sensor_data;
sensor_data sen;
//CMD



//queue
QueueHandle_t sensor_queue;//磁性编码器队列
QueueHandle_t breath_state_queue;//呼吸状态
#define Breathe_queue_size 32





//parameter
int exhale_time = 4000;//呼气时间，单位毫秒
int inhale_time = 4000;//吸气时间，单位毫秒
float motor_voltage = 80;//电机电压，占空比
float pump_voltage = 0.0;//气泵电压，占空比







//handle
TaskHandle_t exhale_handle = NULL;
TaskHandle_t inhale_handle = NULL;
TaskHandle_t App_breathe_handle = NULL;
TaskHandle_t state_gen = NULL;
#define BLE_ENCODER_handle 49

//func
void motor_operate(float duty,int dir);
esp_err_t set_float_as_attr_value(float float_value, int attr_handle);
void APP_SENSOR_Cal_the_angle(void* pvParameters);
void fake_state_gen(void* pvParameters);
void APP_breathe_train_soild_pattern(void* pvParameters);