#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "sdkconfig.h"

#include "ble_advertiser.h"
#include "bme280.h"

#define BLE_ADVERTISER_APP_ID 0

static const char *TAG = "I483Sample_ble_advertiser";

typedef struct {
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;

static uint8_t adv_data[] = {
    0x02, 0x01, 0x06,
    0x03, 0x02, 0xf0, 0xff,
    0x04, 0x09, 0x73, 0x70, 0x73,
    0x0a, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x40,
    .adv_int_max        = 0x80,
    .adv_type           = ADV_TYPE_NONCONN_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void gap_event_handler(esp_gap_ble_cb_event_t xEvent,
                              esp_ble_gap_cb_param_t *pxParam)
{
    switch(xEvent) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        ESP_LOGI(TAG,
                 "%s ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT", __func__);
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        ESP_LOGI(TAG, "%s ESP_GAP_BLE_ADV_START_COMPLETE_EVT", __func__);
        if(pxParam->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "%s Failed to start advertisement", __func__);
        }
        break;
    default:
        ESP_LOGE(TAG, "%s Got unknown event (%d)", __func__, xEvent);
        break;
    }
}

static esp_err_t ble_advertiser_setup(void)
{
    esp_err_t xRes;

    xRes = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if(xRes != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to release BT controller memory: %s",
                 __func__, esp_err_to_name(xRes));
        return xRes;
    }
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    xRes = esp_bt_controller_init(&bt_cfg);
    if(xRes != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to initialize BT controller: %s",
                 __func__, esp_err_to_name(xRes));
        return xRes;
    }
    xRes = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if(xRes != ESP_OK) {
        ESP_LOGE(TAG, "%s feiled to enable BT controller: %s",
                 __func__, esp_err_to_name(xRes));
        return xRes;
    }
    xRes = esp_bluedroid_init();
    if(xRes != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to initialize bluedroid: %s",
                 __func__, esp_err_to_name(xRes));
        return xRes;
    }
    xRes = esp_bluedroid_enable();
    if(xRes != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to enable bluedroid: %s",
                 __func__, esp_err_to_name(xRes));
        return xRes;
    }
    xRes = esp_ble_gap_set_device_name(DEVICE_NAME);
    if(xRes != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to set device name: %s",
                __func__, esp_err_to_name(xRes));
        return xRes;
    }
    xRes = esp_ble_gap_register_callback(gap_event_handler);
    if(xRes != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to register gap callback: %s",
                 __func__, esp_err_to_name(xRes));
        return xRes;
    }
    return xRes;
}

void ble_advertiser_task(void *pvParameter)
{
    bme280_compensated_values xResults;
    int16_t sPres, sTemp, sHum;
    esp_err_t xRes;

    QueueHandle_t *pxQueueHandle = (QueueHandle_t *)pvParameter;

    ESP_LOGI(TAG, "%s BLE Advertiser task started", __func__);

    xRes = ble_advertiser_setup();
    if(xRes != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to initialize ble advertiser", __func__);
    } else {
        for(;;) {
            BaseType_t status;

            status = xQueueReceive(*pxQueueHandle, &xResults, portMAX_DELAY);
            if(status == pdPASS) {
                sPres = xResults.pres / 2560.0;
                sTemp = xResults.temp;
                sHum = xResults.hum / 10.24;
                adv_data[14] = (sTemp & 0x00ff) >> 0;
                adv_data[15] = (sTemp & 0xff00) >> 8;
                adv_data[16] = (sHum & 0x00ff) >> 0;
                adv_data[17] = (sHum & 0xff00) >> 8;
                adv_data[18] = 0;
                adv_data[19] = (sPres & 0x00ff) >> 0;
                adv_data[20] = (sPres & 0xff00) >> 8;
                adv_data[21] = 99;
                xRes = esp_ble_gap_config_adv_data_raw(adv_data,
                                                       sizeof(adv_data));
                if(xRes != ESP_OK) {
                    ESP_LOGE(TAG, "%s Failed to set advertisement data: %s",
                             __func__, esp_err_to_name(xRes));
                    continue;
                }
            } else {
                ESP_LOGE(TAG,
                         "%s Failed to receive measurement data", __func__);
            }
        }
    }
    for(;;) {
        vTaskDelay(1);
    }
}
