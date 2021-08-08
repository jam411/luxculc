#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sys.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"

#include "sdkconfig.h"

#include "wifi_config.h"
#include "wifi_uploader.h"
#include "bme280.h"

#define MAXIMUM_RETRY_COUNT (8)
#define WIFI_CONNECTED_BIT  (BIT0)
#define WIFI_FAILED_BIT     (BIT1)

#define REQUEST_PACKET_SIZE (256)
#define REQUEST_BODY_SIZE   (32)

static const char *TAG = "I483Sample_wifi_uploader";

static int lRetryCount;
static EventGroupHandle_t xWifiEventGroup;

static void wifi_event_handler(void *arg, esp_event_base_t xEventBase,
                               int32_t xEventId, void* rxEventData)
{
    if (xEventBase == WIFI_EVENT && xEventId == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if ((xEventBase == WIFI_EVENT)
               && (xEventId == WIFI_EVENT_STA_DISCONNECTED)) {
        if (lRetryCount < MAXIMUM_RETRY_COUNT) {
            esp_wifi_connect();
            lRetryCount++;
            ESP_LOGI(TAG, "%s Retrying to join WiFi network", __func__);
        } else {
            xEventGroupSetBits(xWifiEventGroup, WIFI_FAILED_BIT);
        }
        ESP_LOGI(TAG,"%s Failed to join WiFi network", __func__);
    } else if (xEventBase == IP_EVENT && xEventId == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) rxEventData;
        ESP_LOGI(TAG, "%s Obtained IP address: %s",
                 __func__, ip4addr_ntoa(&event->ip_info.ip));
        lRetryCount = 0;
        xEventGroupSetBits(xWifiEventGroup, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t wifi_uploader_setup(void)
{
    wifi_init_config_t xConfig = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t xWiFiConfig = {
        .sta = {
            .ssid = WIFI_CONFIG_SSID,
            .password = WIFI_CONFIG_PASSWORD,
        },
    };
    EventBits_t events;
    esp_err_t xRes;

    xWifiEventGroup = xEventGroupCreate();
    if((xRes = esp_event_loop_create_default()) != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to create event loop (%s)",
                 __func__, esp_err_to_name(xRes));
        return xRes;
    }
    if((xRes = esp_wifi_init(&xConfig)) != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to initialize WiFi module (%s)",
                 __func__, esp_err_to_name(xRes));
        return xRes;
    }
    if((xRes = esp_event_handler_register(
                    WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL))
            != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to register handler for WiFi event (%s)",
                 __func__, esp_err_to_name(xRes));
        return xRes;
    }
    if((xRes = esp_event_handler_register(
                    IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL))
            != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to register handler for DHCP event (%s)",
                 __func__, esp_err_to_name(xRes));
        return xRes;
    }
    if((xRes = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to set WiFi module to station mode (%s)",
                    __func__, esp_err_to_name(xRes));
        return xRes;
    }
    if((xRes = esp_wifi_set_config(ESP_IF_WIFI_STA, &xWiFiConfig)) != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to set WiFi configuration (%s)",
                    __func__, esp_err_to_name(xRes));
        return xRes;
    }
    if((xRes = esp_wifi_start()) != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to start WiFi (%s)",
                    __func__, esp_err_to_name(xRes));
        return xRes;
    }
    events = xEventGroupWaitBits(
            xWifiEventGroup, WIFI_CONNECTED_BIT | WIFI_FAILED_BIT,
            pdFALSE, pdFALSE, portMAX_DELAY);
    if(events & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "%s Succeeded to join WiFi network", __func__);
    } else {
        ESP_LOGE(TAG, "%s Failed to join WiFi network (%s)",
                    __func__, esp_err_to_name(xRes));
        return ESP_FAIL;
    }
    return ESP_OK;
}

static int wifi_uploader_connect(char *rxHostname, short sPortNumber)
{
    int lSocket;
    struct hostent *xDestination;
    struct sockaddr_in xDestAddr;

    if ((xDestination = gethostbyname(rxHostname)) == NULL) {
        ESP_LOGE(TAG, "Failed to resolve %s", rxHostname);
        return -1;
    }

    bzero(&xDestAddr, sizeof(struct sockaddr_in));
    xDestAddr.sin_family = AF_INET;
    xDestAddr.sin_addr = *((struct in_addr *)xDestination->h_addr);
    xDestAddr.sin_port = htons(sPortNumber);

    if((lSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        return -1;
    }
    if(connect(lSocket,
               (struct sockaddr *)&xDestAddr, sizeof(struct sockaddr)) < 0) {
        ESP_LOGE(TAG, "Failed to connect %s", rxHostname);
        close(lSocket);
        return -1;
    }
    return lSocket;
}

void wifi_uploader_task(void *pvParameter)
{
    int lSocket, lError;
    esp_err_t xRes;

    QueueHandle_t *pxQueueHandle = (QueueHandle_t *)pvParameter;

    ESP_LOGI(TAG, "%s WiFi Uploader task started", __func__);

    if((xRes = wifi_uploader_setup()) != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to initialize WiFi uploader", __func__);
        goto fin;
    }

    for(lSocket = 0;;) {
        BaseType_t status;
        bme280_compensated_values xResults;

        if(lSocket == 0) {
            if((lSocket = wifi_uploader_connect(FIWARE_HOST,
                                                FIWARE_PORT)) < 0) {
                ESP_LOGE(TAG, "%s Failed to connect to server", __func__);
                continue;
            }
        }

        status = xQueueReceive(*pxQueueHandle, &xResults, portMAX_DELAY);
        if(status == pdPASS) {
            static char rxBuffer[REQUEST_PACKET_SIZE], rxBody[REQUEST_BODY_SIZE];

            snprintf(rxBody, REQUEST_BODY_SIZE,
                     "t|%.2f|h|%.2f|p|%.2f\r\n",
                     xResults.temp / 100.0,
                     xResults.hum / 1024.0,
                     xResults.pres / 25600.0);
            snprintf(rxBuffer, REQUEST_PACKET_SIZE,
                     "POST %s HTTP/1.1\r\n"
                     "Content-Type: text/plain\r\n"
                     "Content-Length: %d\r\n\r\n%s",
                     FIWARE_PATH, strlen(rxBody), rxBody);
            ESP_LOGI(TAG, "%s sending request '%s'", __func__, rxBuffer);
            if((lError = send(lSocket, rxBuffer, strlen(rxBuffer), 0)) < 0) {
                ESP_LOGE(TAG, "%s Failed to send request: %d",
                         __func__, lError);
                close(lSocket);
                lSocket = 0;
                continue;
            }
            for(;;) {
                int lRes;

                lRes = recv(lSocket, rxBuffer, sizeof(rxBuffer) - 1, 0);
                if(lRes > 0) {
                    rxBuffer[lRes] = '\0';
                    ESP_LOGI(TAG, "%s received %d bytes:\n'%s'\n",
                             __func__, lRes, rxBuffer);
                    break;
                } else if (lRes == 0) {
                    ESP_LOGE(TAG, "Connection closed by peer");
                    break;
                } else {
                    ESP_LOGE(TAG, "Failed to receive response: %s",
                             esp_err_to_name(lRes));
                    break;
                }
            }
            close(lSocket);
            lSocket = 0;
        }
    }
fin:
    for(;;) {
        vTaskDelay(1);
    }
}
