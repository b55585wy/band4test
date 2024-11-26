#include <stdio.h>
#include <stdlib.h>
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_mac.h"
#include "sdkconfig.h"

//子模块

#include "nvs.c"
#include "imu.c"
#include "motor.c"
#include "pump.c"
#include "ble.c"