#ifndef INCLUDE_BME280_H
#define INCLUDE_BME280_H

#define BME280_I2C_ADDR                     (0x76)

#define BME280_REGISTER_ID                  (0xd0)
#define BME280_REGISTER_RESET               (0xe0)
#define BME280_REGISTER_CTRL_HUM            (0xf2)
#define BME280_REGISTER_STATUS              (0xf3)
#define BME280_REGISTER_CTRL_MEAS           (0xf4)
#define BME280_REGISTER_CONFIG              (0xf5)
#define BME280_REGISTER_PRESS_MSB           (0xf7)
#define BME280_REGISTER_PRESS_LSB           (0xf8)
#define BME280_REGISTER_PRESS_XLSB          (0xf9)
#define BME280_REGISTER_TEMP_MSB            (0xfa)
#define BME280_REGISTER_TEMP_LSB            (0xfb)
#define BME280_REGISTER_TEMP_XLSB           (0xfc)
#define BME280_REGISTER_HUM_MSB             (0xfd)
#define BME280_REGISTER_HUM_LSB             (0xfe)

#define BME280_RESET_WORD                   (0xb6)

#define BME280_STATUS_IM_UPDATE             (0x01)
#define BME280_STATUS_MEASURING             (0x08)

#define BME280_SAMPLING_MODE_SLEEP          (0x00)
#define BME280_SAMPLING_MODE_FORCE          (0x01)
#define BME280_SAMPLING_MODE_NORMAL         (0x03)

#define BME280_TEMPERATURE_OVERSAMPLING_0   (0x00)
#define BME280_TEMPERATURE_OVERSAMPLING_1   (0x01)
#define BME280_TEMPERATURE_OVERSAMPLING_2   (0x02)
#define BME280_TEMPERATURE_OVERSAMPLING_4   (0x03)
#define BME280_TEMPERATURE_OVERSAMPLING_8   (0x04)
#define BME280_TEMPERATURE_OVERSAMPLING_16  (0x05)

#define BME280_PRESSURE_OVERSAMPLING_0      (0x00)
#define BME280_PRESSURE_OVERSAMPLING_1      (0x01)
#define BME280_PRESSURE_OVERSAMPLING_2      (0x02)
#define BME280_PRESSURE_OVERSAMPLING_4      (0x03)
#define BME280_PRESSURE_OVERSAMPLING_8      (0x04)
#define BME280_PRESSURE_OVERSAMPLING_16     (0x05)

#define BME280_HUMIDITY_OVERSAMPLING_0      (0x00)
#define BME280_HUMIDITY_OVERSAMPLING_1      (0x01)
#define BME280_HUMIDITY_OVERSAMPLING_2      (0x02)
#define BME280_HUMIDITY_OVERSAMPLING_4      (0x03)
#define BME280_HUMIDITY_OVERSAMPLING_8      (0x04)
#define BME280_HUMIDITY_OVERSAMPLING_16     (0x05)

typedef struct {
    uint8_t pres_msb;
    uint8_t pres_lsb;
    uint8_t pres_xlsb;
    uint8_t temp_msb;
    uint8_t temp_lsb;
    uint8_t temp_xlsb;
    uint8_t hum_msb;
    uint8_t hum_lsb;
} bme280_measurements_t;

typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
} bme280_calib_val_t;

typedef struct {
    int32_t pres;
    int32_t temp;
    int32_t hum;
} bme280_compensated_values;

typedef struct {
    uint32_t number_of_sinks;
    QueueHandle_t *handles[0];
} queues_t;

void bme280_task(void *pvParameter);

#endif /* INCLUDE_BME280_H */
