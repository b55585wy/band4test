#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/ledc.h"
//CONST
#define Pump_gpio_num  GPIO_NUM_42
#define AirLock1_gpio_num  GPIO_NUM_35 //放气阀
#define AirLock2_gpio_num  GPIO_NUM_41  //充气阀
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE

#define LEDC_CHANNEL            LEDC_CHANNEL_1
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // PWM占空比的分辨率
#define LEDC_FREQUENCY          (200) 
//QUEUE







void Pump_Init();
void set_pump_on(float cycle);
void set_pump_off(void);
void pump_working(int mircosecond,float cycle);
void pump_test(void* pvParameters);

