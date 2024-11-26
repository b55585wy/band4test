#include "imu.h"



void I2C_init(void)
{
    i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,            // 主机方式启动IIC
    .sda_io_num = I2C_MASTER_SDA_IO, 
    .scl_io_num = I2C_MASTER_SCL_IO,    // 设置数据引脚编号
    .sda_pullup_en = GPIO_PULLUP_ENABLE, // 使能 SDA 上拉电阻
    .scl_pullup_en = GPIO_PULLUP_ENABLE, // 使能 SCL 上拉电阻
    .master.clk_speed = I2C_MASTER_FREQ_HZ // 设置主频
};
    i2c_config_t sensor_conf = {
    .mode = I2C_MODE_MASTER,            // 主机方式启动IIC
    .sda_io_num = I2C_sensor_SDA_IO, 
    .scl_io_num = I2C_sensor_SCLK_IO,    // 设置数据引脚编号
    .sda_pullup_en = GPIO_PULLUP_ENABLE, // 使能 SDA 上拉电阻
    .scl_pullup_en = GPIO_PULLUP_ENABLE, // 使能 SCL 上拉电阻
    .master.clk_speed = I2C_MASTER_FREQ_HZ // 设置主频
};
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_param_config(I2C_SENSOR_NUM, &sensor_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_SENSOR_NUM, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TAG, "IIC 初始化完毕!");
}

void MPU6050_init()
{
    uint8_t check;
    /* 创建设备检测命令链，查看 0x75 寄存器返回的数据
     *  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8
     * ┌─┬─────────────┬─┬─┬───────────────┬─┬─┬─────────────┬─┬─┬───────────────┬─┬─┐
     * │S│  DEV-ADDR   │W│A│   WHO_AM_I    │A│S│  DEV-ADDR   │R│A│    RES DATA   │N│P│
     * └─┴─────────────┴─┴─┴───────────────┴─┴─┴─────────────┴─┴─┴───────────────┴─┴─┘
     *  1      7        1 1        8        1 1       7       1 1        8        1 1
     */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 发送地址，以及写指令，命令之后需要带ACK
    i2c_master_write_byte(cmd, MPU_CMD_WHO_AM_I, true);                   // 发送 WHO_MI_I 寄存器地址 0x75
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &check, I2C_MASTER_LAST_NACK);              // 读取数据
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);                                             // 删除指令
    if (check != 0x68)
    {
        ESP_LOGE(TAG, "MPU6050 不在线!( %02X )", check);
        return;
    }
    ESP_LOGI(TAG, "MPU6050 检测到在线，开始初始化...");
    /* 创建电源管理控制命令链，写入数据
     *  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8
     * ┌─┬─────────────┬─┬─┬───────────────┬─┬───────────────┬─┬─┐
     * │S│  DEV-ADDR   │W│A│  PWR_MGMT_1   │A│   SEND DATA   │A│P│
     * └─┴─────────────┴─┴─┴───────────────┴─┴───────────────┴─┴─┘
     *  1      7        1 1        8        1        8        1 1
     */
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_PWR_MGMT_1, true);                 // 写入电源管理和复位控制
    i2c_master_write_byte(cmd, 0x00, true);                               // 写入寄存器数据
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    // 初始化默认参数（设置陀螺仪采样率分频器）
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_SMPLRT_DIV, true);                 // 写入寄存器地址
    i2c_master_write_byte(cmd, 0x07, true);                               // 写入寄存器数据 Sample rate = 1kHz/(7+1) = 125Hz
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    // 初始化默认参数（数字低通滤波器配置）
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_CONFIG, true);                     // 写入寄存器地址
    i2c_master_write_byte(cmd, 0x00, true);                               // 写入寄存器数据 Gyroscope：260Hz 0ms，Accelerometer：256Hz 0.98ms 8Khz
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    // 初始化默认参数（陀螺仪配置）
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_GYRO_CONFIG, true);                // 写入寄存器地址
    i2c_master_write_byte(cmd, 0x00, true);                               // 写入寄存器数据 Gyroscope: +/- 250 dps
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    // 初始化默认参数（加速度传感器配置）
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_ACCEL_CONFIG, true);               // 写入寄存器地址
    i2c_master_write_byte(cmd, 0x00, true);                               // 写入寄存器数据 Accelerometer: +/- 2g
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    ESP_LOGI(TAG, "MPU6050 初始化完毕!");
}

int16_t get_accel_x()
{
    union
    {
        uint8_t bytes[4];
        uint16_t value;
    } data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_ACCEL_XOUT_H, true);               // 写入寄存器地址，这个寄存器是加速度传感器 X轴 的高位地址
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &data.bytes[1], I2C_MASTER_ACK);            // 读取高位字节数据，放在后面
    i2c_master_read_byte(cmd, &data.bytes[0], I2C_MASTER_NACK);           // 读取低位字节数据，放在前面
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    return data.value;
}

int16_t get_accel_y()
{
    union
    {
        uint8_t bytes[4];
        uint16_t value;
    } data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_ACCEL_YOUT_H, true);               // 写入寄存器地址，这个寄存器是加速度传感器 X轴 的高位地址
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &data.bytes[1], I2C_MASTER_ACK);            // 读取高位字节数据，放在后面
    i2c_master_read_byte(cmd, &data.bytes[0], I2C_MASTER_NACK);           // 读取低位字节数据，放在前面
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    return data.value;
}

int16_t get_accel_z()
{
    union
    {
        uint8_t bytes[4];
        uint16_t value;
    } data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_ACCEL_ZOUT_H, true);               // 写入寄存器地址，这个寄存是加速度传感器 X轴 的高位地址
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &data.bytes[1], I2C_MASTER_ACK);            // 读取高位字节数据，放在后面
    i2c_master_read_byte(cmd, &data.bytes[0], I2C_MASTER_NACK);           // 读取低位字节数据，放在前面
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
   return data.value;
}

int16_t get_angle_x()
{
    union
    {
        uint8_t bytes[4];
        uint16_t value;
    } data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_ANGLE_XOUT_H, true);               // 写入寄存器地址，这个寄存器是加速度传感器 X轴 的高位地址
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &data.bytes[1], I2C_MASTER_ACK);            // 读取高位字节数据，放在后面
    i2c_master_read_byte(cmd, &data.bytes[0], I2C_MASTER_NACK);           // 读取低位字节数据，放在前面
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
   return data.value;
}

int16_t get_angle_y()
{
    union
    {
        uint8_t bytes[4];
        uint16_t value;
    } data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_ANGLE_YOUT_H, true);               // 写入寄存器地址，这个寄存器是加速度传感器 X轴 的高位地址
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &data.bytes[1], I2C_MASTER_ACK);            // 读取高位字节数据，放在后面
    i2c_master_read_byte(cmd, &data.bytes[0], I2C_MASTER_NACK);           // 读取低位字节数据，放在前面
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
   return data.value;
}
int16_t get_angle_z()
{
    union
    {
        uint8_t bytes[4];
        uint16_t value;
    } data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_ANGLE_ZOUT_H, true);               // 写入寄存器地址，这个寄存器是加速度传感器 X轴 的高位地址
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &data.bytes[1], I2C_MASTER_ACK);            // 读取高位字节数据，放在后面
    i2c_master_read_byte(cmd, &data.bytes[0], I2C_MASTER_NACK);           // 读取低位字节数据，放在前面
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数
    i2c_cmd_link_delete(cmd);
   return data.value;
}
imu_data imu_send_wifi()
{
    imu_data imu;
    imu.x_acc=get_accel_x();
    imu.y_acc=get_accel_y();
    imu.z_acc=get_accel_z();
    imu.x_anl=get_angle_x();
    imu.y_anl=get_angle_y();
    imu.z_anl=get_angle_z();
    return imu;
}

float twoscomplementscale(int16_t data,int16_t scale,int16_t range)
{
    float data_f;
  //convert it to signed float after divide range
  data_f=data*1.0/range*scale;
  printf("%f",data_f);
  return data_f;
}

#define FILTER_ORDER 5

void init_fir_filter(FIRFilter* filter) {
    // 使用给定的滤波器系数
    filter->coeffs[0] = 0.0356953f;
    filter->coeffs[1] = 0.24106222f;
    filter->coeffs[2] = 0.44648496f;
    filter->coeffs[3] = 0.24106222f;
    filter->coeffs[4] = 0.0356953f;

    // 初始化滤波器缓冲区为0
    memset(filter->buffer, 0, sizeof(float) * FILTER_ORDER);
    // 初始化缓冲区索引为0
    filter->index = 0;
}

float apply_fir_filter(FIRFilter* filter, float input) {
    // 将新的输入放入缓冲区
    filter->buffer[filter->index] = input;
    
    float output = 0;
    int index = filter->index;
    
    // 计算滤波后的输出
    for (int i = 0; i < FILTER_ORDER; i++) {
        output += filter->coeffs[i] * filter->buffer[index];
        index = (index - 1 + FILTER_ORDER) % FILTER_ORDER;
    }
    
    // 更新索引
    filter->index = (filter->index + 1) % FILTER_ORDER;
    
    return output;
}

void init_kalman_filter(KalmanFilter* kf) {
    // 初始化状态
    kf->state.position = 0;
    kf->state.velocity = 0;
    kf->state.acceleration = 0;

    // 初始化协方差矩阵
    memset(kf->P, 0, sizeof(kf->P));
    kf->P[0][0] = kf->P[1][1] = kf->P[2][2] = 1000;  // 高初始不确定性

    // 设置过程噪声协方差
    memset(kf->Q, 0, sizeof(kf->Q));
    kf->Q[0][0] = 0.01;  // 位置过程噪声
    kf->Q[1][1] = 0.01;  // 速度过程噪声
    kf->Q[2][2] = 0.01;  // 加速度过程噪声

    // 设置测量噪声协方差
    kf->R = 0.1;  // 加速度测量噪声
}

void kalman_predict(KalmanFilter* kf, float dt) {
    // 预测状态
    kf->state.position += kf->state.velocity * dt + 0.5 * kf->state.acceleration * dt * dt;
    kf->state.velocity += kf->state.acceleration * dt;

    // 更新协方差矩阵
    float F[3][3] = {{1, dt, (float)(0.5*dt*dt)}, {0, 1, dt}, {0, 0, 1}};
    float FP[3][3];
    float FPFt[3][3];

    // 矩阵乘法 FP = F * P
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            FP[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                FP[i][j] += F[i][k] * kf->P[k][j];
            }
        }
    }

    // 矩阵乘法 FPFt = FP * F^T
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            FPFt[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                FPFt[i][j] += FP[i][k] * F[j][k];
            }
        }
    }

    // P = FPFt + Q
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            kf->P[i][j] = FPFt[i][j] + kf->Q[i][j];
        }
    }
}

void kalman_update(KalmanFilter* kf, float measurement) {
    // 计算卡尔曼增益
    float H[3] = {0, 0, 1};  // 测量矩阵
    float PH[3];
    float S;

    for (int i = 0; i < 3; i++) {
        PH[i] = kf->P[i][2];  // 因为H = [0, 0, 1]，所以只需要P的第三列
    }

    S = PH[2] + kf->R;  // S = HPH^T + R

    float K[3];
    for (int i = 0; i < 3; i++) {
        K[i] = PH[i] / S;
    }

    // 更新状态
    float innovation = measurement - kf->state.acceleration;
    kf->state.position += K[0] * innovation;
    kf->state.velocity += K[1] * innovation;
    kf->state.acceleration += K[2] * innovation;

    // 更新协方差矩阵
    float IKH[3][3] = {{1, 0, -K[0]}, {0, 1, -K[1]}, {0, 0, 1-K[2]}};
    float newP[3][3];

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            newP[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                newP[i][j] += IKH[i][k] * kf->P[k][j];
            }
        }
    }

    memcpy(kf->P, newP, sizeof(newP));
}

bool detect_zero_velocity(float acceleration, float threshold) {
    return fabs(acceleration) < threshold;
}

float estimate_displacement(float acceleration) {
    static KalmanFilter kf;
    static bool initialized = false;
    static float last_time = 0;
    float current_time = esp_timer_get_time() / 1000000.0;  // 转换为秒
    float dt = current_time - last_time;
    last_time = current_time;

    if (!initialized) {
        init_kalman_filter(&kf);
        initialized = true;
    }

    kalman_predict(&kf, dt);
    kalman_update(&kf, acceleration);

    if (detect_zero_velocity(acceleration, 0.05)) {  // 阈值可能需要调整
        kf.state.velocity = 0;
    }

    return kf.state.position;
}

void app_imu_get_data(void* pvParameters)
{
    imu_data imu;
    FIRFilter x_acc_filter, y_acc_filter, z_acc_filter;
    FIRFilter x_anl_filter, y_anl_filter, z_anl_filter;
    
    init_fir_filter(&x_acc_filter);
    init_fir_filter(&y_acc_filter);
    init_fir_filter(&z_acc_filter);
    init_fir_filter(&x_anl_filter);
    init_fir_filter(&y_anl_filter);
    init_fir_filter(&z_anl_filter);

    vTaskDelay(100);    // 等待一秒钟开始获取数据
    while (1)
    {
        int16_t x_acc = get_accel_x();
        int16_t y_acc = get_accel_y();
        int16_t z_acc = get_accel_z();
        int16_t x_anl = get_angle_x();
        int16_t y_anl = get_angle_y();
        int16_t z_anl = get_angle_z();
        }

        // 估算位移
        float displacement = estimate_displacement(imu.z_acc);  // 假设z轴是垂直于腹部的轴

        // 将位移添加到imu结构体中
        imu.displacement = displacement;

        xQueueSend(imu_data_queue, &imu, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(10));
    }


void app_data_receive(void* pvParameters)
{
    imu_data imu;
        while (1)
        {
        if (xQueueReceive(imu_data_queue, &imu, portMAX_DELAY)) 
        {
            //printf("%d,%d,%d,%d,%d,%d\n", imu.x_acc,imu.y_acc,imu.z_acc,imu.x_anl,imu.y_anl,imu.z_anl);
            printf("displacement: %f\n", imu.displacement);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
         }
}   

