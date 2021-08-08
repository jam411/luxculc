#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "sdkconfig.h"

#include "bme280.h"
#include "ble_advertiser.h"
#include "wifi_uploader.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#define NUMBER_OF_SINKS (2)

static const char *TAG = "I483SensorDevice_main";

void app_main(void)
{
    int i;
    static uint8_t q_area[sizeof(queues_t)
                            + sizeof(QueueHandle_t) * NUMBER_OF_SINKS];
    queues_t *pxQs;
    esp_err_t xRes;

    pxQs = (queues_t *)q_area;
    pxQs->number_of_sinks = NUMBER_OF_SINKS;
    for(i = 0; i < pxQs->number_of_sinks; i++) {
        pxQs->handles[i] = xQueueCreate(8, sizeof(bme280_compensated_values));
        if(pxQs->handles[i] == NULL) {
            ESP_LOGE(TAG, "%s Failed to create queue", __func__);
            goto fin;
        }
    }
    xRes = nvs_flash_init();
    if(xRes == ESP_ERR_NVS_NO_FREE_PAGES
       || xRes == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        xRes = nvs_flash_erase();
        if(xRes != ESP_OK) {
            ESP_LOGE(TAG, "%s Failed to erase NVS: %s",
                     __func__, esp_err_to_name(xRes));
            goto fin;
        }
        xRes = nvs_flash_init();
        if(xRes != ESP_OK) {
            ESP_LOGE(TAG, "%s Failed to initialize NVS: %s",
                     __func__, esp_err_to_name(xRes));
            goto fin;
        }
    }
    if(xRes != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to initialize NVS: %s",
                 __func__, esp_err_to_name(xRes));
        goto fin;
    }
    xTaskCreatePinnedToCore(bme280_task, "bm2320_task",
                            4096, pxQs, 5, NULL, 1);
    ESP_LOGI(TAG, "%s BME280 task created", __func__);
    xTaskCreatePinnedToCore(wifi_uploader_task, "wifi_uploader_task",
                            4096, &(pxQs->handles[0]), 5, NULL, 1);
    ESP_LOGI(TAG, "%s WiFi uploader task created", __func__);
    xTaskCreatePinnedToCore(ble_advertiser_task, "ble_advertiser_task",
                            4096, &(pxQs->handles[1]), 5, NULL, 1);
    ESP_LOGI(TAG, "%s BLE advertiser task created", __func__);

fin:
    for(;;) {
        vTaskDelay(1);
    }
}
