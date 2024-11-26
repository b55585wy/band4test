#include "motor.h"



void Motor_init()
{
    mcpwm_gpio_init(mcm_unit, MCPWM0A, motor_A_gpio);  // 配置 MCPWM0A 引脚
    mcpwm_gpio_init(mcm_unit, MCPWM0B, motor_B_gpio);  // 配置 MCPWM0B 引脚

    // 初始化 MCPWM
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 1000Hz 1000赫兹频率(看样子，每个pwm定时器可以设置不同的频率，而其下的2个通道是要同频率的)
    pwm_config.cmpr_a = 0.0;       //duty cycle of PWMxA
    pwm_config.cmpr_b = 0.0;       //
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;  // 设定占空比类型 
    pwm_config.counter_mode = MCPWM_UP_COUNTER; //设定计数类型
    mcpwm_init(mcm_unit, mcm_timer, &pwm_config);  //初始化mcpwm
    // 配置EN GPIO
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE; // 禁用中断
    io_conf.mode = GPIO_MODE_OUTPUT;       // 设置为输出模式
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // 启用下拉电阻
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;     // 禁用上拉电阻
    io_conf.pin_bit_mask = (1ULL << motor_en_gpio); // 设置引脚掩码
    gpio_config(&io_conf);
    gpio_set_level(motor_en_gpio,1);

}

void motor_operate(float duty,int dir)
{
    mcpwm_start(mcm_unit, mcm_timer);
    if(dir==motor_forward)
    {
        mcpwm_set_duty(mcm_unit,mcm_timer,mcm_a_gen,duty);
        mcpwm_set_duty(mcm_unit,mcm_timer,mcm_b_gen,0);
    }
    else if(dir==motor_backward)
    {
        mcpwm_set_duty(mcm_unit,mcm_timer,mcm_a_gen,0);
        mcpwm_set_duty(mcm_unit,mcm_timer,mcm_b_gen,duty);
    }
    else if(dir == motor_stop)
    {
        mcpwm_set_duty(mcm_unit,mcm_timer,mcm_a_gen,0);
        mcpwm_set_duty(mcm_unit,mcm_timer,mcm_b_gen,0);
    }
}




int16_t I2C_read_angle_from_mt6701()
{
   union
    {
        uint8_t bytes[2];
        uint16_t value;
    } data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MT6701_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, angle_reg_1, true);               // 写入寄存器地址，这个寄存器是角度 的高位地址
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MT6701_ADDR<< 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &data.bytes[1], I2C_MASTER_ACK);            // 读取高位字节数据，放在后面
    i2c_master_read_byte(cmd, &data.bytes[0], I2C_MASTER_NACK);           // 读取低位字节数据，放在前面
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数
    i2c_cmd_link_delete(cmd);
    int16_t angle = (uint16_t)data.bytes[0]>>2;
    angle |= (uint16_t)data.bytes[1]<<6;
    return angle;
}

int16_t I2C_read_angle_from_as5600()
{
   union
    {
        uint8_t bytes[4];
        uint16_t value;
    } data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (AS5600_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, AS5600_ANG_REG, true);               // 写入寄存器地址，这个寄存器是角度 的高位地址
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (AS5600_ADDR<< 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &data.bytes[1], I2C_MASTER_ACK);            // 读取高位字节数据，放在后面
    i2c_master_read_byte(cmd, &data.bytes[0], I2C_MASTER_NACK);           // 读取低位字节数据，放在前面
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数
    i2c_cmd_link_delete(cmd);

    return data.value;
}

void TEST_READ_ANG(void* pvParameters)
{
    
    while (1)
    {
        int16_t ang = I2C_read_angle_from_mt6701();
        float anl_ang = ang*360/16384;
        ESP_LOGI("ANG","%f,%d",anl_ang,ang);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
}

sensor_data init_sen()
{
    sensor_data input_sen;
    input_sen.angle=0;
    input_sen.angle_prev=0;
    input_sen.raw_data=0;
    input_sen.raw_data_prev=0;
    input_sen.distance=0;
    input_sen.init=0;
    return input_sen;
}

float cal_the_distance(sensor_data sen)//计算转过的角度
{
    float distance1;
    if(sen.angle>345&&sen.angle_prev<10)//下溢出
    {
        sen.angle_prev+=360;

    }
    else if (sen.angle<10&&sen.angle_prev>345)//上溢出
    {
        sen.angle+=360;
    }
    
    distance1=sen.angle-sen.angle_prev;
    
    if(abs(distance1)<0.5)//处理抖动
    {
        distance1=0;
    }

    //ESP_LOGI("CAL","%f",distance1);
    return distance1;
    

}

esp_err_t set_float_as_attr_value(float float_value, int attr_handle) {
    uint8_t attr_value[4];  // 用于存储转换后的浮点数，4字节
    memcpy(attr_value, &float_value, sizeof(float_value));  // 将浮动值转换为字节数组

    // 设置 GATT 属性值
    esp_err_t ret = esp_ble_gatts_set_attr_value(attr_handle, sizeof(attr_value), attr_value);
    if (ret != ESP_OK) {
        ESP_LOGE("GATTS_TAG", "Failed to set attribute value: %d", ret);
        return ret;
    }
    //ESP_LOGI("ATTR","%hhu,%hhu,%hhu,%hhu",attr_value[0],attr_value[1],attr_value[2],attr_value[3]);
    
    //ESP_LOGI("GATTS_TAG", "Set float value %.2f as GATT attribute", float_value);
    return ESP_OK;
}

void APP_SENSOR_Cal_the_angle(void* pvParameters)
{
    
    sen=init_sen();
    
    
    while (1)
    { 
        sen.raw_data=I2C_read_angle_from_mt6701();
        sen.angle=sen.raw_data*360.0/16384.0;

        if(!sen.init)//如果未进行初始化则初始化
        {
            sen.distance=0;
            sen.init=1;
            
        }
        else{
            sen.distance += cal_the_distance(sen);
        }
        // ESP_LOGI("SEN","%f,%f,%f,%f",sen.distance,sen.angle,sen.angle_prev,mcpwm_get_duty(mcm_unit,mcm_timer,mcm_a_gen));
        //迭代
        sen.angle_prev=sen.angle;
        sen.raw_data_prev=sen.raw_data;
        set_float_as_attr_value(sen.distance,BLE_ENCODER_handle);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
}

void fake_state_gen_EXHALE(void* pvParameters)
{   
    
    while(1)
    {
    int breathe_state=Exhale_state;
    ESP_LOGI("BREATHE","EXHALE,distance:%f",sen.distance);
    xQueueSend(breath_state_queue,&breathe_state, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
    }
}
void fake_state_gen_INHALE(void* pvParameters)
{   
    
    while(1)
    {
    int breathe_state=Inhale_state;
    ESP_LOGI("BREATHE","inHALE,distance:%f",sen.distance);
    xQueueSend(breath_state_queue,&breathe_state, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));

    }
}

void fake_state_gen(void* pvParameters)
{
    while(1)
    {
        xTaskCreatePinnedToCore(fake_state_gen_INHALE,"inhale ",4096,NULL,3,&inhale_handle,1);
        vTaskDelay(pdMS_TO_TICKS(inhale_time));
        if(inhale_handle != NULL) 
        {
        vTaskDelete(inhale_handle);  // 删除指定任务
        inhale_handle = NULL;  // 任务句柄无效化
        }
        xTaskCreatePinnedToCore(fake_state_gen_EXHALE,"exhale ",4096,NULL,3,&exhale_handle,1);
        vTaskDelay(pdMS_TO_TICKS(exhale_time));
        if(exhale_handle != NULL) 
        {
        vTaskDelete(exhale_handle);  // 删除指定任务
        exhale_handle = NULL;  // 任务句柄无效化
        }
    }
}






void APP_breathe_train_soild_pattern(void* pvParameters)
{
    int breathe_state=0;


    while (1)
    {   

        if(xQueueReceive(breath_state_queue,&breathe_state, pdMS_TO_TICKS(1000)))//接收队列中的数据
        {
            if(breathe_state==Exhale_state)//如果目前是呼气状态
            {
                motor_operate(motor_voltage,motor_backward);  
                vTaskDelay(pdMS_TO_TICKS(400));
                set_pump_on(pump_voltage);
                vTaskDelay(pdMS_TO_TICKS(10));
                
            }
            if(breathe_state==Inhale_state)//如果目前是吸气状态
            {
                set_pump_off();
                motor_operate(motor_voltage,motor_forward);               
                vTaskDelay(pdMS_TO_TICKS(10));
                
            }

        }
        else
        {
            motor_operate(62.0,motor_stop);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    
}
