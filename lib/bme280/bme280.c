#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "bme280.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

static const char *TAG = "I483Sample_bme280";

static esp_err_t i2c_master_driver_initialize(void)
{
    i2c_config_t xConf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 22,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 1000000,
    };
    return i2c_param_config(I2C_NUM_0, &xConf);
}

static esp_err_t bme280_get_register(uint8_t ucAddr,
                                     uint8_t *pucVal, uint8_t ucLen)
{
    esp_err_t xRes;

    i2c_cmd_handle_t xCmd = i2c_cmd_link_create();

    i2c_master_start(xCmd);
    i2c_master_write_byte(xCmd,
                          (BME280_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(xCmd, ucAddr, true);
    i2c_master_start(xCmd);
    i2c_master_write_byte(xCmd,(BME280_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(xCmd, pucVal, ucLen, I2C_MASTER_LAST_NACK);
    i2c_master_stop(xCmd);

    xRes = i2c_master_cmd_begin(I2C_NUM_0, xCmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(xCmd);
    return xRes;
}

static esp_err_t bme280_set_register(uint8_t ucAddr, uint8_t ucVal)
{
    esp_err_t xRes;

    i2c_cmd_handle_t xCmd = i2c_cmd_link_create();

    i2c_master_start(xCmd);
    i2c_master_write_byte(xCmd,
                          (BME280_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(xCmd, ucAddr, true);
    i2c_master_write_byte(xCmd, ucVal, true);
    i2c_master_stop(xCmd);

    xRes = i2c_master_cmd_begin(I2C_NUM_0, xCmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(xCmd);
    return xRes;
}

static esp_err_t bme280_read_status(uint8_t *pucStatus)
{
    esp_err_t xRes;

    xRes = bme280_get_register(BME280_REGISTER_STATUS, pucStatus, 1);
    return xRes;
}

static esp_err_t bme280_read_calib_val(bme280_calib_val_t *pxParam)
{
    esp_err_t xRes;
    uint8_t ucCalib[32];

    xRes = bme280_get_register(0x88, ucCalib + 0, 24);
    if(xRes != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to read calibration data (%s)",
                 __func__, esp_err_to_name(xRes));
        return xRes;
    }

    xRes = bme280_get_register(0xa1, ucCalib + 24, 1);
    if(xRes != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to read calibration data (%s)",
                 __func__, esp_err_to_name(xRes));
        return xRes;
    }

    xRes = bme280_get_register(0xe1, ucCalib + 25, 7);
    if(xRes != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to read calibration data (%s)",
                 __func__, esp_err_to_name(xRes));
        return xRes;
    }

    pxParam->dig_T1 = (((uint16_t)ucCalib[1] << 8) | ucCalib[0]);
    pxParam->dig_T2 = (((int16_t)ucCalib[3] << 8) | ucCalib[2]);
    pxParam->dig_T3 = (((int16_t)ucCalib[5] << 8) | ucCalib[4]);
    pxParam->dig_P1 = (((uint16_t)ucCalib[7] << 8) | ucCalib[6]);
    pxParam->dig_P2 = (((int16_t)ucCalib[9] << 8) | ucCalib[8]);
    pxParam->dig_P3 = (((int16_t)ucCalib[11] << 8) | ucCalib[10]);
    pxParam->dig_P4 = (((int16_t)ucCalib[13] << 8) | ucCalib[12]);
    pxParam->dig_P5 = (((int16_t)ucCalib[15] << 8) | ucCalib[14]);
    pxParam->dig_P6 = (((int16_t)ucCalib[17] << 8) | ucCalib[16]);
    pxParam->dig_P7 = (((int16_t)ucCalib[19] << 8) | ucCalib[18]);
    pxParam->dig_P8 = (((int16_t)ucCalib[21] << 8) | ucCalib[20]);
    pxParam->dig_P9 = (((int16_t)ucCalib[23] << 8) | ucCalib[22]);
    pxParam->dig_H1 = (uint8_t)ucCalib[24];
    pxParam->dig_H2 = (((int16_t)ucCalib[26] << 8) | ucCalib[25]);
    pxParam->dig_H3 = (uint8_t)ucCalib[27];
    pxParam->dig_H4 = (((int16_t)ucCalib[28] << 4) | (ucCalib[29] & 0x0f));
    pxParam->dig_H5 = (((int16_t)ucCalib[30] << 4)
                       | ((ucCalib[29] & 0xf0) >> 4));
    pxParam->dig_H6 = (int8_t)ucCalib[31];

    return ESP_OK;
}

static esp_err_t bme280_reset(void)
{
    esp_err_t xRes;

    xRes = bme280_set_register(BME280_REGISTER_RESET, BME280_RESET_WORD);
    if(xRes != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to reset BME280 (%s)",
                 __func__, esp_err_to_name(xRes));
        return xRes;
    }

    for(;;) {
        uint8_t cStatus;

        xRes = bme280_read_status(&cStatus);
        if(xRes != ESP_OK) {
            ESP_LOGE(TAG, "%s Failed to read status (%s)",
                     __func__, esp_err_to_name(xRes));
            return xRes;
        }

        if(((cStatus & BME280_STATUS_IM_UPDATE) == 0)
                &&((cStatus & BME280_STATUS_MEASURING) == 0)) {
            break;
        }
    }
    return xRes;
}

static esp_err_t bme280_measure_once(bme280_measurements_t *pxMeasurements)
{
    uint8_t cStatus, *pucMeasurements =(uint8_t *)pxMeasurements;
    esp_err_t xRes;

    xRes = bme280_set_register(BME280_REGISTER_CTRL_HUM,
                              (BME280_HUMIDITY_OVERSAMPLING_1 << 0));
    if(xRes != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to set configuration (%s)",
                 __func__, esp_err_to_name(xRes));
        return xRes;
    }

    xRes = bme280_set_register(BME280_REGISTER_CTRL_MEAS,
                              (BME280_TEMPERATURE_OVERSAMPLING_1 << 5)
                              | (BME280_PRESSURE_OVERSAMPLING_1 << 2)
                              | (BME280_SAMPLING_MODE_FORCE << 0));
    if(xRes != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to set configuration (%s)",
                 __func__, esp_err_to_name(xRes));
        return xRes;
    }

    for(;;) {
        xRes = bme280_read_status(&cStatus);
        if(xRes != ESP_OK) {
            ESP_LOGE(TAG, "%s Failed to read status (%s)",
                     __func__, esp_err_to_name(xRes));
            return xRes;
        }

        if((cStatus & BME280_STATUS_MEASURING) == BME280_STATUS_MEASURING) {
            break;
        }
    }

    for(;;) {
        xRes = bme280_get_register(BME280_REGISTER_STATUS, &cStatus, 1);
        if(xRes != ESP_OK) {
            ESP_LOGE(TAG, "%s Failed to read status (%s)",
                     __func__, esp_err_to_name(xRes));
            return xRes;
        }

        if(((cStatus & BME280_STATUS_IM_UPDATE) == 0)
           && ((cStatus & BME280_STATUS_MEASURING) == 0)) {
            break;
        }
    }

    xRes = bme280_get_register(BME280_REGISTER_PRESS_MSB, pucMeasurements, 8);
    if(xRes != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to read measurements (%s)",
                 __func__, esp_err_to_name(xRes));
    }
    return xRes;
}

static void bme280_compensate_measurements(bme280_calib_val_t *pxCd,
                                           bme280_compensated_values *pxCm,
                                           int32_t cRawPres,
                                           int32_t cRawTemp,
                                           int32_t cRawHum)
{
    int32_t lTVar1, lTVar2, lVX1U32r, lTemp, lTFine;
    uint32_t luHum;
    int64_t llPVar1, llPVar2, llPres;

    lTVar1 = ((((cRawTemp >> 3)
                - ((int32_t)pxCd->dig_T1 << 1)))
              * ((int32_t)pxCd->dig_T2)) >> 11;
    lTVar2 = (((((cRawTemp >> 4) - ((int32_t)pxCd->dig_T1))
                * ((cRawTemp >> 4) - ((int32_t)pxCd->dig_T1))) >> 12)
              * ((int32_t)pxCd->dig_T3)) >> 14;
    lTFine = lTVar1 + lTVar2;
    lTemp = (lTFine * 5 + 128) >> 8;
    pxCm->temp = lTemp;

    llPVar1 = ((int64_t)lTFine) - 128000;
    llPVar2 = llPVar1 * llPVar1 * (int64_t)pxCd->dig_P6;
    llPVar2 = llPVar2 + ((llPVar1 * (int64_t)pxCd->dig_P5) << 17);
    llPVar2 = llPVar2 + (((int64_t)pxCd->dig_P4) << 35);
    llPVar1 = ((llPVar1 * llPVar1 * (int64_t)pxCd->dig_P3) >> 8);
    llPVar1 = (((((int64_t)1) << 47) + llPVar1))
              * ((int64_t)pxCd->dig_P1) >> 33;

    if (llPVar1 == 0) {
        llPres = 0;
    } else {
        llPres = 1048576 - cRawPres;
        llPres = (((llPres << 31) - llPVar2) * 3125) / llPVar1;
        llPVar1 = (((int64_t)pxCd->dig_P9)
                   * (llPres >> 13) * (llPres >> 13)) >> 25;
        llPVar2 = (((int64_t)pxCd->dig_P8) * llPres) >> 19;
        llPres = ((llPres + llPVar1 + llPVar2) >> 8)
                 + (((int64_t)pxCd->dig_P7) << 4);
    }
    pxCm->pres = (uint32_t)llPres;

    lVX1U32r = lTFine - ((int32_t)76800);
    lVX1U32r = (((((cRawHum << 14) - (((int32_t)pxCd->dig_H4) << 20)
                                   - (((int32_t)pxCd->dig_H5) * lVX1U32r))
                  + ((int32_t)16384)) >> 15)
                * (((((((lVX1U32r * ((int32_t)pxCd->dig_H6)) >> 10)
                       * (((lVX1U32r * ((int32_t)pxCd->dig_H3)) >> 11)
                          + ((int32_t)32768))) >> 10) + ((int32_t)2097152))
                    * ((int32_t)pxCd->dig_H2) + 8192) >> 14));
    lVX1U32r = (lVX1U32r - (((((lVX1U32r >> 15) * (lVX1U32r >> 15)) >> 7)
                             * ((int32_t)pxCd->dig_H1)) >> 4));
    lVX1U32r = (lVX1U32r < 0 ? 0 : lVX1U32r);
    lVX1U32r = (lVX1U32r > 419430400 ? 419430400 : lVX1U32r);
    luHum = (uint32_t) (lVX1U32r >> 12);
    pxCm->hum = luHum;
}

void bme280_task(void *pvParameter)
{
    bme280_measurements_t xMeasurements;
    static bme280_calib_val_t xCalibVal = {
        .dig_T1 = 0,
        .dig_T2 = 0,
        .dig_T3 = 0,
        .dig_P1 = 0,
        .dig_P2 = 0,
        .dig_P3 = 0,
        .dig_P4 = 0,
        .dig_P5 = 0,
        .dig_P6 = 0,
        .dig_P7 = 0,
        .dig_P8 = 0,
        .dig_P9 = 0,
        .dig_H1 = 0,
        .dig_H2 = 0,
        .dig_H3 = 0,
        .dig_H4 = 0,
        .dig_H5 = 0,
        .dig_H6 = 0,
    };
    bme280_compensated_values xResults;
    uint8_t ucBme280Id;
    TickType_t xLastWakeTime;
    esp_err_t xRes;

    queues_t *queues = (queues_t *)pvParameter;

    ESP_LOGI(TAG, "%s BME280 task started", __func__);

    i2c_master_driver_initialize();
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    bme280_reset();

    xRes = bme280_read_calib_val(&xCalibVal);
    if(xRes != ESP_OK) {
        ESP_LOGE(TAG, "%s Failed to read calibration data", __func__);
    }

    xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        if((xRes = bme280_measure_once(&xMeasurements)) == ESP_OK) {
            int i;

            bme280_compensate_measurements(&xCalibVal, &xResults,
                                           (xMeasurements.pres_msb << 12)
                                           | (xMeasurements.pres_lsb << 4)
                                           | (xMeasurements.pres_xlsb >> 4),
                                           (xMeasurements.temp_msb << 12)
                                           | (xMeasurements.temp_lsb << 4)
                                           | (xMeasurements.temp_xlsb >> 4),
                                           (xMeasurements.hum_msb << 8)
                                           | (xMeasurements.hum_lsb << 0));
            for(i = 0; i < queues->number_of_sinks; i++) {
                xQueueSendToFront(queues->handles[i],
                                  (void *)&xResults, pdMS_TO_TICKS(1000));
            }
        } else {
            ESP_LOGE(TAG, "%s Failed to read measurements (%s)",
                     __func__, esp_err_to_name(xRes));
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(60000));
    }
}
