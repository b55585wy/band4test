#include "pump.h"


/*
气阀为气关型气阀，设0为开，设1为关

*/

void Pump_Init()
{
      // 配置LEDC定时器
    ledc_timer_config_t ledc_timer = {
        LEDC_MODE,
        LEDC_DUTY_RES,
         LEDC_TIMER,
        LEDC_FREQUENCY,  // 设置频率为200Hz
        LEDC_AUTO_CLK,
        false // 初始化所有的成员
    };
    ledc_timer_config(&ledc_timer);

    // 配置LEDC通道
    ledc_channel_config_t ledc_channel = {
        .gpio_num = Pump_gpio_num,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0, // 初始占空比为0

    };
    ledc_channel_config(&ledc_channel);

    
    
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE; // 禁用中断
    io_conf.mode = GPIO_MODE_OUTPUT;       // 设置为输出模式
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // 启用下拉电阻
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;     // 禁用上拉电阻
    io_conf.pin_bit_mask = (1ULL << AirLock2_gpio_num)|(1ULL << AirLock1_gpio_num); // 设置引脚掩码
    gpio_config(&io_conf);
    gpio_set_level(AirLock2_gpio_num,0);
    gpio_set_level(AirLock1_gpio_num,0);

}


void pump_working(int mircosecond,float cycle)
{
    uint32_t duty=(uint32_t) cycle*8191/100;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);//设置占空比
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);//更新占空比,启动气泵
    gpio_set_level(AirLock2_gpio_num,0);//打开充气阀
    gpio_set_level(AirLock1_gpio_num,0);//打开放气阀
    vTaskDelay(pdMS_TO_TICKS(mircosecond));
    gpio_set_level(AirLock2_gpio_num,1);//关闭充气阀
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(Pump_gpio_num,0);//关闭气泵
    vTaskDelay(pdMS_TO_TICKS(2000));
    gpio_set_level(AirLock1_gpio_num,0);//打开放气阀
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(AirLock2_gpio_num,0);//打开充气阀
    
    
}

void set_pump_on(float cycle)
{
    uint32_t duty=(uint32_t) cycle*8191/100;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);//设置占空比
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);//更新占空比,启动气泵
    gpio_set_level(Pump_gpio_num,1);//启动气泵
    gpio_set_level(AirLock2_gpio_num,0);//打开充气阀
    gpio_set_level(AirLock1_gpio_num,1);//关闭放气阀
}
void set_pump_off(void)
{
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);//设置占空比
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);//更新占空比,关闭气泵
    gpio_set_level(AirLock2_gpio_num,0);//打开充气阀
    gpio_set_level(AirLock1_gpio_num,0);//打开放气阀
}