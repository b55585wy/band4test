#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include "driver/adc.h"
#include "hal/adc_types.h"
#include "esp_log.h"

//关于ADC设置
//衰减倍数，ESP32 设计的 ADC 参考电压为 1100mV,只能测量 0-1100mV 之间的电压，如果要测量更大范围的电压
//需要设置衰减倍数
/*以下是对应可测量范围
ADC_ATTEN_DB_0 100 mV ~ 950 mV
ADC_ATTEN_DB_2_5 100 mV ~ 1250 mV
ADC_ATTEN_DB_6 150 mV ~ 1750 mV
ADC_ATTEN_DB_12 150 mV ~ 2450 mV
*/

#define ADC_channel ADC2_CHANNEL_5
