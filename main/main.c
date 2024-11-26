#include "main.h"

void app_main(void)
{
    //init
    NVS_init();//初始化NVS
    I2C_init();//初始化I2C
    //MPU6050_init();//imu初始化
    //Motor_init();//电机初始化
    ble_Init();
    Pump_Init();
    //queue init
    imu_data_queue = xQueueCreate(imu_queue_size, sizeof(imu_data));//imu数据传输队列初始化
    breath_state_queue=xQueueCreate(Breathe_queue_size ,sizeof(int));//呼吸状态队列初始化



    //task
    xTaskCreatePinnedToCore(APP_SENSOR_Cal_the_angle,"angleread ",4096,NULL,3,NULL,1);


}
