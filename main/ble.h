/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_gatt_common_api.h"
#include "motor.h"


#define GATTS_TABLE_TAG "GATTS_TABLE_DEMO"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "SYNC"
#define SVC_INST_ID                 0

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
*  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
*/
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

static uint8_t raw_adv_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power*/
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF, 0x00,
        /* device name */
        0x05, 0x09, 'S', 'Y', 'N', 'C'
};
static uint8_t raw_scan_rsp_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power */
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF,0x00
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};
/* Service */
static const uint16_t GATTS_SERVICE_UUID_TEST      = '000000ff-0000-1000-8000-00805f9b34fb';
static const uint16_t GATTS_CHAR_UUID_TEST_A       = '0000ff01-0000-1000-8000-00805f9b34fb';
static const uint16_t Battery_UUID                 = '0000ff02-0000-1000-8000-00805f9b34fb';
static const uint16_t GATTS_WRITE_UUID             = '0000ff03-0000-1000-8000-00805f9b34fb';
static const uint16_t GATTS_ENCODER_READ_UUID      = '0000ff04-0000-1000-8000-00805f9b34fb';
static const uint16_t GATTS_MOTOR_I_READ_UUID      = '0000ff05-0000-1000-8000-00805f9b34fb';


static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t heart_measurement_ccc[2]      = {0x00, 0x00};
static const uint8_t char_value[4]                 = {0x11, 0x22, 0x33, 0x44};
/* Attributes State Machine */
enum
{  //service              handle
    IDX_SVC,                //40
    IDX_CHAR_A,             //41
    IDX_CHAR_VAL_A,         //42
    IDX_CHAR_CFG_A,         //43

    IDX_CHAR_B,             //44
    IDX_CHAR_VAL_B,         //45

    IDX_CHAR_C,             //46
    IDX_CHAR_VAL_C,         //47
    IDX_CHAR_ENCODER,       //48
    IDX_CHAR_VAL_ENCODER,   //49
    IDX_CHAR_MOTOR_I,       //50
    IDX_CHAR_VAL_MOTOR_I,   //51
    HRS_IDX_NB,             //52
};


//ble cmd
/*
BLE WRITE FORMAT


len==3 // START/STOP
BYTE[1]:CMD TYPE
BYTE[2]:Start/stop
BYTE[3]:CMD APP 
*/
#define cmd_motor 0x01
#define cmd_app_breathe 0x01
#define cmd_app_start 0x01
#define cmd_app_stop 0x00
/*
len==4 // PARAMETER SETTING
BYTE[1]:CMD TYPE
BYTE[2]:PAPRAMETER TYPE
BYTE[3]:byte1
BYTE[4]:byte2

*/
//parameter type
#define para_exhale_time 0x01 //cmd_motor
#define para_inhale_time 0x02 //cmd_motor
#define para_motor_voltage 0x03//cmd_motor
#define para_pump_voltage 0x04//cmd_motor





















