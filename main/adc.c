#include "adc.h"

void ADC_init()
{
    adc2_config_channel_atten(ADC_channel, ADC_ATTEN_DB_12);//设置ADC2  


}

int adc_oneshot_trans()
{
    int adc_val;
    esp_err_t status = adc2_get_raw(ADC_channel, ADC_WIDTH_BIT_12, &adc_val);//进行
    if (status == ESP_OK) {
        // 成功读取
       return adc_val;

    } else {
        // 读取失败
        return 0;
    }
}
void app_adc_val_read_test(void* pvParameters)
{
    while(1){
        int adc=0;
        adc=adc_oneshot_trans();
        ESP_LOGI("ADC","%d",adc);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}